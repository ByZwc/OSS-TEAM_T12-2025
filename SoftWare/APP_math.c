/*
    @author: 张文超
    @file: "APP_math.c"
    @date: 2025_9_10
    @attention: 以下函数（包括滤波、温度曲线）只适用本硬件系统，不得移植到其他系统
    @version：1.0
 */
#include "main.h"

float32_t firCoeffs[NUM_TAPS] = {0.25f, 0.25f, 0.25f, 0.25f}; // 止频率1Hz

//*****************************************************************************************/
float32_t firState_Ntc[NUM_TAPS + BLOCK_SIZE - 1] = {0};

arm_fir_instance_f32 fir_instance_Ntc = {
    .numTaps = NUM_TAPS,
    .pState = firState_Ntc,
    .pCoeffs = firCoeffs,
};

float32_t APP_FirFilter_NtcTemp(float32_t Src)
{
    float32_t Dst;

    arm_fir_f32(&fir_instance_Ntc, &Src, &Dst, BLOCK_SIZE);

    return Dst;
}

//*****************************************************************************************/

//*****************************************************************************************/
float32_t firState_adc[NUM_TAPS + BLOCK_SIZE - 1] = {0};

arm_fir_instance_f32 fir_instance_adc = {
    .numTaps = NUM_TAPS,
    .pState = firState_adc,
    .pCoeffs = firCoeffs,
};

float32_t APP_FirFilter_ADC(float32_t Src)
{
    float32_t Dst;

    arm_fir_f32(&fir_instance_adc, &Src, &Dst, BLOCK_SIZE);

    return Dst;
}

//*****************************************************************************************/
#define KALMAN_Q_MIN_TEMP 0.3f
#define KALMAN_Q_MAX_TEMP 3.0f
#define KALMAN_R_MIN_TEMP 0.8f

static TYPEDEF_KALMAN_S kalman_temp_inst = {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f};

#define KALMAN_R_MAX_TEMP 6.0f
#define KALMAN_DIFF_THRESH_TEMP 25.0f

float32_t APP_kalmanFilter_solderingTemp(float32_t input, float32_t target)
{
    if (input > SOLDERING_TEMP_OPEN)
    {
        // 温度超限，直接重置滤波结果
        kalman_temp_inst.x = input;
        kalman_temp_inst.p = 1.0f;
        kalman_temp_inst.diff = 0.0f;
        kalman_temp_inst.q = 0.0f;
        kalman_temp_inst.r = 0.0f;
        kalman_temp_inst.k = 0.0f;
        return kalman_temp_inst.x;
    }

    // 采样与设定温度差异
    kalman_temp_inst.diff = fabsf(input - target);

    if (kalman_temp_inst.diff < KALMAN_DIFF_THRESH_TEMP)
    {
        kalman_temp_inst.q = KALMAN_Q_MIN_TEMP + (KALMAN_Q_MAX_TEMP - KALMAN_Q_MIN_TEMP) * (kalman_temp_inst.diff / KALMAN_DIFF_THRESH_TEMP);
        kalman_temp_inst.r = KALMAN_R_MIN_TEMP + (KALMAN_R_MAX_TEMP - KALMAN_R_MIN_TEMP) * (kalman_temp_inst.diff / KALMAN_DIFF_THRESH_TEMP);
    }
    else
    {
        kalman_temp_inst.q = KALMAN_Q_MAX_TEMP;
        kalman_temp_inst.r = KALMAN_R_MAX_TEMP;
    }

    kalman_temp_inst.p += kalman_temp_inst.q;
    kalman_temp_inst.k = kalman_temp_inst.p / (kalman_temp_inst.p + kalman_temp_inst.r);
    kalman_temp_inst.x += kalman_temp_inst.k * (input - kalman_temp_inst.x);
    kalman_temp_inst.p *= (1.0f - kalman_temp_inst.k);

    return kalman_temp_inst.x;
}

//********************************************************************************************************************/
// 中位值滤波缓冲区
#define MEDIAN_FILTER_SIZE 5
static uint32_t median_buffer[MEDIAN_FILTER_SIZE] = {0};
static uint8_t median_index = 0;

// 中位值滤波函数
static uint32_t median_filter(uint32_t input)
{
    uint32_t temp_buf[MEDIAN_FILTER_SIZE];

    // 更新缓冲区
    median_buffer[median_index] = input;
    median_index = (median_index + 1) % MEDIAN_FILTER_SIZE;

    // 复制到临时数组进行排序
    for (int i = 0; i < MEDIAN_FILTER_SIZE; i++)
    {
        temp_buf[i] = median_buffer[i];
    }

    // 简单冒泡排序
    for (int i = 0; i < MEDIAN_FILTER_SIZE - 1; i++)
    {
        for (int j = 0; j < MEDIAN_FILTER_SIZE - 1 - i; j++)
        {
            if (temp_buf[j] > temp_buf[j + 1])
            {
                uint32_t temp = temp_buf[j];
                temp_buf[j] = temp_buf[j + 1];
                temp_buf[j + 1] = temp;
            }
        }
    }

    // 返回中位值
    return temp_buf[MEDIAN_FILTER_SIZE / 2];
}

//********************************************************************************************************************/
#define ADC_RESOLUTION 4095   // 12位ADC最大值
#define ADC_REF_VOLTAGE 3.3f  // ADC基准电压(V)
#define SAMPLE_RESISTOR 0.01f // 采样电阻(Ω)

#define MAX_LIMITED_DIFF 30
#define MAX_LIMITED_DIFF_LAST 0.66f

// 限幅滤波函数：抑制大幅脉冲干扰
// 输入：当前ADC原始值；输出：限幅后的值
uint16_t LimitFilter(uint16_t current_adc)
{
    static uint16_t last_adc = 0;                  // 保存上一次滤波后的值
    int32_t diff = current_adc / MAX_LIMITED_DIFF; // 限幅到上次值的±50%

    if (current_adc > last_adc)
    {
        last_adc = diff;
        return diff * MAX_LIMITED_DIFF_LAST;
    }
    else
    {
        last_adc = current_adc * MAX_LIMITED_DIFF_LAST;
        return current_adc;
    }
}

#define FIR_FILTER_LEN 5
static float fir_coeffs[FIR_FILTER_LEN] = {
    0.15f, 0.20f, 0.30f, 0.20f, 0.15f}; // 加权平均
static uint16_t fir_buffer[FIR_FILTER_LEN] = {0};
static uint8_t fir_index = 0;

// 输入：限幅后的ADC值；输出：滤波后的值（返回uint16_t匹配ADC范围）
uint16_t FIR_LowpassFilter(uint16_t limit_adc)
{
    fir_buffer[fir_index] = limit_adc;
    fir_index = (fir_index + 1) % FIR_FILTER_LEN;

    float sum = 0.0f;
    for (uint8_t i = 0; i < FIR_FILTER_LEN; i++)
    {
        sum += fir_coeffs[i] * fir_buffer[(fir_index + i) % FIR_FILTER_LEN];
    }

    // 防止溢出，钳位到ADC合法范围（0~4095）
    if (sum < 0.0f)
        return 0;
    else if (sum > ADC_RESOLUTION)
        return ADC_RESOLUTION;
    else
        return (uint16_t)sum;
}

// 电流卡尔曼滤波器参数宏定义
#define KALMAN_ELE_Q_MIN 0.05f  // 最小过程噪声
#define KALMAN_ELE_R_MAX 200.0f // 最大测量噪声

// 电流卡尔曼滤波器
static TYPEDEF_KALMAN_S kalman_ele_inst = {0.0f, 1.0f, 0.0f, KALMAN_ELE_Q_MIN, KALMAN_ELE_R_MAX, 0.0f};

float32_t APP_KalmanFilter_Ele(float32_t measured_ele)
{
    // 预测
    kalman_ele_inst.p += kalman_ele_inst.q;

    // 计算卡尔曼增益并更新
    kalman_ele_inst.k = kalman_ele_inst.p / (kalman_ele_inst.p + kalman_ele_inst.r);
    kalman_ele_inst.x += kalman_ele_inst.k * (measured_ele - kalman_ele_inst.x);
    kalman_ele_inst.p *= (1.0f - kalman_ele_inst.k);

    return kalman_ele_inst.x;
}

float GetLoadCurrent(uint16_t adcVlaue)
{
    uint16_t adc_raw = 0;      // ADC原始采样值
    uint16_t adc_limit = 0;    // 限幅后的值
    uint16_t adc_filtered = 0; // 卡尔曼滤波后的值
    uint16_t adc_iir = 0;      // FIR滤波后的值
    uint16_t adc_median = 0;   // 中位值滤波后的值
    float voltage = 0.0f;      // 采样电阻两端电压(V)
    float current = 0.0f;      // 负载电流(A)

    // 步骤1：读取ADC原始值
    adc_raw = adcVlaue;

    // 步骤2：限幅滤波
    adc_limit = LimitFilter(adc_raw);

    // 步骤3：中位值滤波
    adc_median = median_filter(adc_limit);

    // 步骤4：卡尔曼滤波
    adc_filtered = APP_KalmanFilter_Ele(adc_median);

    // 步骤5：FIR低通滤波
    adc_iir = FIR_LowpassFilter(adc_filtered);

    // 步骤6：计算真实电流
    voltage = (float)adc_iir * ADC_REF_VOLTAGE / ADC_RESOLUTION; // 电压=ADC值×基准/分辨率
    current = voltage / SAMPLE_RESISTOR;                         // 电流=电压/采样电阻

    return current;
}

//********************************************************************************************************************/

#define PARAM_RANGE_MIN 0
#define PARAM_RANGE_MAX 10000
#define PARAM_OUTPUT_MIN 0.5f
#define PARAM_OUTPUT_MAX 1.0f

float32_t APP_ParamToRatio(uint32_t param, float32_t scale)
{
    if (param <= PARAM_RANGE_MIN)
        return PARAM_OUTPUT_MAX;
    if (param >= PARAM_RANGE_MAX)
        return PARAM_OUTPUT_MIN;
    float32_t ratio = PARAM_OUTPUT_MAX - (PARAM_OUTPUT_MAX - PARAM_OUTPUT_MIN) * ((float32_t)param / (float32_t)PARAM_RANGE_MAX);
    // 可选缩放
    ratio *= scale;
    if (ratio < PARAM_OUTPUT_MIN)
        ratio = PARAM_OUTPUT_MIN;
    if (ratio > PARAM_OUTPUT_MAX)
        ratio = PARAM_OUTPUT_MAX;
    return ratio;
}

// 功率卡尔曼滤波器参数宏定义
#define KALMAN_POWER_Q_MIN 0.2f // 最小过程噪声
#define KALMAN_POWER_Q_MAX 2.0f // 最大过程噪声
#define KALMAN_POWER_R_MIN 0.5f // 最小测量噪声
#define KALMAN_POWER_R_MAX 3.0f // 最大测量噪声

// 功率卡尔曼滤波器
static TYPEDEF_KALMAN_S kalman_power_inst = {0.0f, 1.0f, 0.0f, KALMAN_POWER_Q_MAX, KALMAN_POWER_R_MIN, 0.0f};

float32_t APP_KalmanFilter_Power(float32_t measured_power, float32_t target_power)
{
    // 记录测量与目标差异（备用）
    kalman_power_inst.diff = fabsf(measured_power - target_power);

    // 保持与原实现一致的固定噪声设定
    kalman_power_inst.q = KALMAN_POWER_Q_MIN;
    kalman_power_inst.r = KALMAN_POWER_R_MAX;

    // 预测
    kalman_power_inst.p += kalman_power_inst.q;

    // 更新
    kalman_power_inst.k = kalman_power_inst.p / (kalman_power_inst.p + kalman_power_inst.r);
    kalman_power_inst.x += kalman_power_inst.k * (measured_power - kalman_power_inst.x);
    kalman_power_inst.p *= (1.0f - kalman_power_inst.k);

    return kalman_power_inst.x;
}

float32_t APP_GetElectricity_soft_Task()
{
    static float32_t V0 = 24.0f;
    // static uint8_t StaticPower_flag = 0;
    float32_t R0 = AllStatus_S.r0;
    // float32_t electricity_inv;
    float32_t Power;
    Power = ((V0 / R0) * (AllStatus_S.pid_s.pid_out / 10000.0f) * V0 * 0.98f); // 估计功率W
    /* if ((Power < 10.0f) && (AllStatus_S.data_filter_prev[SOLDERING_TEMP210_NUM] <= (AllStatus_S.flashSave_s.TarTemp + 1)))
        StaticPower_flag = 0;
    else
        StaticPower_flag = 0;
    electricity_inv = (AllStatus_S.PowerStatic * (AllStatus_S.flashSave_s.TarTemp / 450.0f) * StaticPower_flag); // 静态功率损耗 */
    if (AllStatus_S.SolderingState == 0)
        return APP_KalmanFilter_Power(Power, 0.0f);
    else
        return 0.0f;
}

void app_Samp_electricity_Task(void)
{
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
    {
        AllStatus_S.adc_value[SOLDERING_ELECTRICITY_NUM] = Drive_ADCConvert(SOLDERING_ELECTRICITY_NUM);
        AllStatus_S.data_filter_prev[SOLDERING_ELECTRICITY_NUM] = 24.0 * GetLoadCurrent(AllStatus_S.adc_value[SOLDERING_ELECTRICITY_NUM]) * 0.9f;
    }
    else
    {
        AllStatus_S.adc_value[SOLDERING_ELECTRICITY_NUM] = 0;
        AllStatus_S.data_filter_prev[SOLDERING_ELECTRICITY_NUM] = 24.0 * GetLoadCurrent(AllStatus_S.adc_value[SOLDERING_ELECTRICITY_NUM]);
    }
}

void app_GetAdcVlaue_electricity_Task(void)
{
    AllStatus_S.data_filter[SOLDERING_ELECTRICITY_NUM] = APP_GetElectricity_soft_Task();
}

// 互补滤波器权重宏定义
#define COMPLEMENTARY_FILTER_LOW_POWER_THRESH 10.0f
#define COMPLEMENTARY_FILTER_HIGH_POWER_THRESH 200.0f
#define COMPLEMENTARY_FILTER_EST_WEIGHT_LOW 0.5f
#define COMPLEMENTARY_FILTER_MEAS_WEIGHT_HIGH 0.5f

#define COMPLEMENTARY_FILTER_EST_WEIGHT_HIGH 0.8f
#define COMPLEMENTARY_FILTER_MEAS_WEIGHT_LOW 0.2f

#define COMPLEMENTARY_FILTER_VOLTAGE 24.0f

// 互补滤波器函数：输入估计功率、测量功率，返回滤波结果
// 参数：estPower-估计功率（单位：W），measPower-测量功率（单位：W）
float APP_Power_complementaryFilter_Task(void)
{
    float estPower = AllStatus_S.data_filter[SOLDERING_ELECTRICITY_NUM];       // 估计功率
    float measPower = AllStatus_S.data_filter_prev[SOLDERING_ELECTRICITY_NUM]; // 测量功率
    float weight_est = 0.0f;                                                   // 估计功率权重
    float weight_meas = 0.0f;                                                  // 测量功率权重
    float filterResult;                                                        // 滤波结果

    // 1. 根据功率范围动态分配权重（信任度）
    if (measPower > COMPLEMENTARY_FILTER_HIGH_POWER_THRESH) // 测量功率>200W：提高测量信任度
    {
        weight_est = COMPLEMENTARY_FILTER_EST_WEIGHT_LOW;    // 估计权重降低
        weight_meas = COMPLEMENTARY_FILTER_MEAS_WEIGHT_HIGH; // 测量权重提高
    }
    else // 其余情况：估计功率信任度最高
    {
        weight_est = COMPLEMENTARY_FILTER_EST_WEIGHT_HIGH;  // 估计权重提高
        weight_meas = COMPLEMENTARY_FILTER_MEAS_WEIGHT_LOW; // 测量权重降低
    }

    // 2. 互补滤波核心计算（权重加权求和）
    filterResult = estPower * weight_est + measPower * weight_meas;

    AllStatus_S.Power = filterResult;                                      // 更新全局功率值
    AllStatus_S.Electricity = filterResult / COMPLEMENTARY_FILTER_VOLTAGE; // 电流值A

    return filterResult; // 返回滤波结果
}
//*****************************************************************************************/

static float32_t square_2(float32_t value)
{
    return (value * value);
}

static float32_t square_3(float32_t value)
{
    return (value * value * value);
}

#define SOLDERING_TEMP115_P1 (4.674e-08)
#define SOLDERING_TEMP115_P2 (-0.0003252)
#define SOLDERING_TEMP115_P3 (0.9733)
#define SOLDERING_TEMP115_P4 (-602.7)                                      //(-582.7)
float32_t app_solderingT115_adcTurnToTemp(uint32_t solderingT115_AdcVlaue) // T115温度曲线函数（OK）
{
    return (SOLDERING_TEMP115_P1 * (square_3((float32_t)solderingT115_AdcVlaue)) + SOLDERING_TEMP115_P2 * (square_2((float32_t)solderingT115_AdcVlaue)) + SOLDERING_TEMP115_P3 * solderingT115_AdcVlaue + SOLDERING_TEMP115_P4); // ℃
}

#define SOLDERING_TEMP210_P1 (7.156e-08)
#define SOLDERING_TEMP210_P2 (-0.0003044)
#define SOLDERING_TEMP210_P3 (0.7673)
#define SOLDERING_TEMP210_P4 (-268.8)
float32_t app_solderingT210_adcTurnToTemp(uint32_t solderingT210_AdcVlaue) // T210温度曲线函数（）
{
    return (SOLDERING_TEMP210_P1 * (square_3((float32_t)solderingT210_AdcVlaue)) + SOLDERING_TEMP210_P2 * (square_2((float32_t)solderingT210_AdcVlaue)) + SOLDERING_TEMP210_P3 * solderingT210_AdcVlaue + SOLDERING_TEMP210_P4);
}

#define SOLDERING_TEMP245_P1 (1.023e-08)
#define SOLDERING_TEMP245_P2 (-5.828e-05)
#define SOLDERING_TEMP245_P3 (0.2462)
#define SOLDERING_TEMP245_P4 (-75.67)                                     //(-70.67)
float32_t app_solderingT245_adcTurnToTemp(uint32_t solderingT45_AdcVlaue) // T245温度曲线函数（OK）
{
    return (SOLDERING_TEMP245_P1 * (square_3((float32_t)solderingT45_AdcVlaue)) + SOLDERING_TEMP245_P2 * (square_2((float32_t)solderingT45_AdcVlaue)) + SOLDERING_TEMP245_P3 * solderingT45_AdcVlaue + SOLDERING_TEMP245_P4); // ℃
}

#define SOLDERING_TEMP_T12_P1 (0.1938)
#define SOLDERING_TEMP_T12_P2 (26.1030)
#define SOLDERING_TEMP_T12_P3 (0)
#define SOLDERING_TEMP_T12_P4 (0)
float32_t app_soldering_T12_adcTurnToTemp(uint32_t solderingT45_AdcVlaue) // T12温度曲线函数（OK）
{
    return (SOLDERING_TEMP_T12_P1 * (float32_t)solderingT45_AdcVlaue + SOLDERING_TEMP_T12_P2); // ℃
}

float32_t APP_solderingElectricity_Get(uint32_t solderingElectricity_AdcVlaue) // 电流值获取
{
    return ((330.0f / 4095.0f) * (float32_t)solderingElectricity_AdcVlaue); // 0.1V
}

// 20K串联 10K NTC 系数
#define NTC10K_P1 (-6.022e-09f)
#define NTC10K_P2 (4.09e-05f)
#define NTC10K_P3 (-0.1058f)
#define NTC10K_P4 (104.6f)

// 20K串联 100K NTC 系数
#define NTC100K_P1 (-8.348e-09f)
#define NTC100K_P2 (5.805e-05f)
#define NTC100K_P3 (-0.1557f)
#define NTC100K_P4 (212.3f)

float32_t APP_PcbNtc_adcTurnToTemp(uint32_t pcbNtc_AdcVlaue) // PCB板载NTC温度曲线函数
{
    return (NTC100K_P1 * (square_3((float32_t)pcbNtc_AdcVlaue)) + NTC100K_P2 * (square_2((float32_t)pcbNtc_AdcVlaue)) + NTC100K_P3 * pcbNtc_AdcVlaue + NTC100K_P4); // ℃
}

float32_t APP_solderingID_Check(uint32_t solderingID_AdcVlaue) // ID引脚识别
{
    return solderingID_AdcVlaue;
}

// 1、获取ADC原始值
// 2、处理ADC原始值并输出（NTC）
static void app_GetAdcVlaue_NTC(void)
{
    AllStatus_S.adc_value[PCB_NTC_NUM] = Drive_ADCConvert(PCB_NTC_NUM);
    AllStatus_S.adc_conversionValue[PCB_NTC_NUM] = APP_PcbNtc_adcTurnToTemp(AllStatus_S.adc_value[PCB_NTC_NUM]);
    AllStatus_S.data_filter[PCB_NTC_NUM] = APP_FirFilter_NtcTemp(AllStatus_S.adc_conversionValue[PCB_NTC_NUM]);
}

// PCB温度过高保护
void app_PcbTempProtect_Task(void)
{
    app_GetAdcVlaue_NTC();
    if (AllStatus_S.data_filter[PCB_NTC_NUM] > PCB_PROTECT_TEMP) // PCB温度过高保护
    {
        AllStatus_S.SolderingState = SOLDERING_STATE_NTC_ERROR;
        APP_ErrorHandler();
    }
}

// 1、获取ADC原始值
// 2、处理ADC原始值并输出（电流）
static void app_GetAdcVlaue_electricity(void)
{
    /* Enable ADC */
    AllStatus_S.adc_value[SOLDERING_ELECTRICITY_NUM] = Drive_ADCConvert(SOLDERING_ELECTRICITY_NUM);
    AllStatus_S.adc_conversionValue[SOLDERING_ELECTRICITY_NUM] = APP_solderingElectricity_Get(AllStatus_S.adc_value[SOLDERING_ELECTRICITY_NUM]);
}

// 1、获取ADC原始值
// 2、处理ADC原始值并输出（TID）
static void app_GetAdcVlaue_TID(void)
{
    AllStatus_S.adc_value[SOLDERING_TID_NUM] = Drive_ADCConvert(SOLDERING_TID_NUM);
}

static uint8_t app_GetSolderingTid(void)
{
    app_GetAdcVlaue_TID();
    if (((uint32_t)AllStatus_S.adc_value[SOLDERING_TID_NUM]) > SOLDERING_TID_T245)
        return SOLDERING_MODEL_T245;
    if (((uint32_t)AllStatus_S.adc_value[SOLDERING_TID_NUM]) < SOLDERING_TID_T210)
        return SOLDERING_MODEL_T210;
    if (((uint32_t)AllStatus_S.adc_value[SOLDERING_TID_NUM]) > SOLDERING_TID_T115_MIX && ((uint32_t)AllStatus_S.adc_value[SOLDERING_TID_NUM]) < SOLDERING_TID_T115_MAX)
        return SOLDERING_MODEL_T115;
    return 0;
}

static uint8_t find_max_in_array(const uint32_t *array, uint8_t len)
{
    if (len == 0)
        return 0;
    uint32_t max_val = array[0];
    uint8_t max_idx = 0;
    for (uint8_t i = 1; i < len; i++)
    {
        if (array[i] > max_val)
        {
            max_val = array[i];
            max_idx = i;
        }
    }
    return max_idx + 1; // 返回编号（从1开始）
}

// 短路&开路&型号判断
void APP_shortCircuitProtection(void)
{
    uint32_t tid_array[3] = {0};
    uint8_t tid;
    for (int i = 0; i < 10; i++)
    {
        tid = app_GetSolderingTid();
        switch (tid)
        {
        case SOLDERING_MODEL_T115:
            tid_array[0]++;
            break;
        case SOLDERING_MODEL_T210:
            tid_array[1]++;
            break;
        case SOLDERING_MODEL_T245:
            tid_array[2]++;
            break;
        case 0:
            return;
        }
    }

    tid = find_max_in_array(tid_array, 3);

    switch (tid)
    {
    case 1:
        AllStatus_S.SolderingModelNumber = SOLDERING_MODEL_T115;
        break;
    case 2:
        AllStatus_S.SolderingModelNumber = SOLDERING_MODEL_T210;
        break;
    case 3:
        AllStatus_S.SolderingModelNumber = SOLDERING_MODEL_T245;
        break;
    }
    Drive_MosSwitch_OFF();
    if (AllStatus_S.SolderingModelNumber == SOLDERING_MODEL_T115 || AllStatus_S.SolderingModelNumber == SOLDERING_MODEL_T210)
        HAL_GPIO_WritePin(T210_T115_GPIO_PORT, T210_T115_GPIO_PIN, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(T245_GPIO_PORT, T245_GPIO_PIN, GPIO_PIN_SET);
    app_GetAdcVlaue_electricity();
    HAL_GPIO_WritePin(T210_T115_GPIO_PORT, T210_T115_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(T245_GPIO_PORT, T245_GPIO_PIN, GPIO_PIN_RESET);

    AllStatus_S.SolderingState = SOLDERING_STATE_OK;
    if (((uint32_t)AllStatus_S.adc_conversionValue[SOLDERING_ELECTRICITY_NUM]) > SOLDERING_ELECTRICITY_THRESHOLD)
    {
        AllStatus_S.SolderingState = SOLDERING_STATE_SHORTCIR_ERROR;
        Drive_Buz_OnOff(BUZ_1S, BUZ_FREQ_CHANGE_OFF, USE_BUZ_TYPE);
        return;
        // APP_ErrorHandler();
    }
    if (((uint32_t)AllStatus_S.adc_conversionValue[SOLDERING_ELECTRICITY_NUM]) < SOLDERING_ELECTRICITY_OPEN)
    {
        AllStatus_S.SolderingState = SOLDERING_STATE_PULL_OUT_ERROR;
        return;
        // APP_ErrorHandler();
    }
    switch (AllStatus_S.SolderingModelNumber)
    {
    case SOLDERING_MODEL_T115:
        AllStatus_S.pid_s.pid_pCoef = 7.5f;
        AllStatus_S.pid_s.pid_iCoef = 0.1f;
        AllStatus_S.pid_s.pid_dCoef = 0.0f;
        AllStatus_S.pid_s.pid_integration_max = T115_MAX_PID_I;
        AllStatus_S.pid_s.pid_iItemCmd = 0.0f;
        AllStatus_S.pid_s.outPriod = T115_PID_MIX_CHANGE_PRIOD;
        AllStatus_S.pid_s.outPriod_max = T115_PID_MAX_CHANGE_PRIOD;
        AllStatus_S.pid_s.diffTempOutMaxPWM = T115_SOLDERING_MAX_PID;
        AllStatus_S.pid_s.pid_iItemJoinTemp = 15;
        AllStatus_S.pid_s.pid_iItemQuitTemp = 20;
        AllStatus_S.r0 = 3.05f;         // 设置T115阻值
        AllStatus_S.PowerStatic = 4.5f; // 115静态功率
        Drive_MosSwitch210_PWMOut();    // 开启115PWM输出
        break;
    case SOLDERING_MODEL_T210:
        AllStatus_S.pid_s.pid_pCoef = 15.0f;
        AllStatus_S.pid_s.pid_iCoef = 0.15f;
        AllStatus_S.pid_s.pid_dCoef = 0.0f;
        AllStatus_S.pid_s.pid_integration_max = T210_MAX_PID_I;
        AllStatus_S.pid_s.pid_iItemCmd = 0.0f;
        AllStatus_S.pid_s.outPriod = T210_PID_MIX_CHANGE_PRIOD;
        AllStatus_S.pid_s.outPriod_max = T210_PID_MAX_CHANGE_PRIOD;
        AllStatus_S.pid_s.diffTempOutMaxPWM = T210_SOLDERING_MAX_PID;
        AllStatus_S.pid_s.pid_iItemJoinTemp = 15;
        AllStatus_S.pid_s.pid_iItemQuitTemp = 15;
        AllStatus_S.r0 = 2.1f;          // 设置T210阻值
        AllStatus_S.PowerStatic = 6.0f; // 210静态功率
        Drive_MosSwitch210_PWMOut();    // 开启210PWM输出
        break;
    case SOLDERING_MODEL_T245:
        AllStatus_S.pid_s.pid_pCoef = 135.0f;
        AllStatus_S.pid_s.pid_iCoef = 10.5f;
        AllStatus_S.pid_s.pid_dCoef = 0.0f;
        AllStatus_S.pid_s.pid_integration_max = T245_MAX_PID_I;
        AllStatus_S.pid_s.pid_iItemCmd = 0.0f;
        AllStatus_S.pid_s.outPriod = T12_PID_MIX_CHANGE_PRIOD;
        AllStatus_S.pid_s.outPriod_max = T12_PID_MAX_CHANGE_PRIOD;
        AllStatus_S.pid_s.diffTempOutMaxPWM = T245_SOLDERING_MAX_PID;
        AllStatus_S.pid_s.pid_iItemJoinTemp = 25;
        AllStatus_S.pid_s.pid_iItemQuitTemp = 15;
        // AllStatus_S.r0 = 2.55f;         // T245阻值
        AllStatus_S.r0 = 7.800f;        // T12阻值
        AllStatus_S.PowerStatic = 9.0f; // 245静态功率
        Drive_MosSwitch245_PWMOut();    // 开启245PWM输出
        break;
    }
}

// 1、获取ADC原始值
// 2、处理ADC原始值并输出（T210温度、T245温度）
static void app_GetAdcVlaue_soldering(void)
{
    Drive_MosSwitch_OFF(); // 关断MOS输出

    // HAL_Delay(1);
    for (uint16_t i = 0; i < 0x8FF; i++)
    {
        __NOP();
    }

    AllStatus_S.adc_value[SOLDERING_TEMP210_NUM] = Drive_ADCConvert(SOLDERING_TEMP210_NUM);
    AllStatus_S.adc_filter_value = (uint16_t)APP_FirFilter_ADC((float32_t)AllStatus_S.adc_value[SOLDERING_TEMP210_NUM]);
    switch (AllStatus_S.SolderingModelNumber)
    {
    case SOLDERING_MODEL_T115:
        AllStatus_S.adc_conversionValue[SOLDERING_TEMP210_NUM] = app_solderingT115_adcTurnToTemp(AllStatus_S.adc_value[SOLDERING_TEMP210_NUM]);
        break;
    case SOLDERING_MODEL_T210:
        AllStatus_S.adc_conversionValue[SOLDERING_TEMP210_NUM] = app_solderingT210_adcTurnToTemp(AllStatus_S.adc_value[SOLDERING_TEMP210_NUM]);
        break;
    case SOLDERING_MODEL_T245:
        AllStatus_S.adc_conversionValue[SOLDERING_TEMP210_NUM] = app_soldering_T12_adcTurnToTemp(AllStatus_S.adc_value[SOLDERING_TEMP210_NUM]);
        break;
    }

    if (AllStatus_S.adc_conversionValue[SOLDERING_TEMP210_NUM] > SOLDERING_TEMP_OPEN)
    {
        AllStatus_S.adc_conversionValue[SOLDERING_TEMP210_NUM] = SOLDERING_TEMP_OPEN;
    }

    AllStatus_S.data_filter[SOLDERING_TEMP210_NUM] = APP_kalmanFilter_solderingTemp(AllStatus_S.adc_conversionValue[SOLDERING_TEMP210_NUM], AllStatus_S.flashSave_s.TarTemp);
    AllStatus_S.CurTemp = AllStatus_S.data_filter[SOLDERING_TEMP210_NUM];

    if (AllStatus_S.SolderingState == SOLDERING_STATE_PULL_OUT_ERROR)
    {
        AllStatus_S.data_filter_prev[SOLDERING_TEMP210_NUM] = app_DisplayFilter_RC(app_DisplayFilter_kalman(0, AllStatus_S.flashSave_s.TarTemp), AllStatus_S.flashSave_s.TarTemp);
    }
    else
    {
        AllStatus_S.data_filter_prev[SOLDERING_TEMP210_NUM] = app_DisplayFilter_RC(app_DisplayFilter_kalman(AllStatus_S.CurTemp, AllStatus_S.flashSave_s.TarTemp), AllStatus_S.flashSave_s.TarTemp);
    }
}

void app_pid_Task(void)
{
    if (AllStatus_S.SolderingState != SOLDERING_STATE_SLEEP_DEEP)
    {
        app_GetAdcVlaue_soldering(); // 获取烙铁头温度
        app_pidControl(AllStatus_S.flashSave_s.TarTemp + AllStatus_S.flashSave_s.calibration_temp, AllStatus_S.CurTemp);
    }
}

#define DISPLAY_FILTER_BASE_ALPHA 0.05f // 基础滤波系数
#define DISPLAY_FILTER_MIN_ALPHA 0.01f  // 最小滤波系数
#define DISPLAY_FILTER_MAX_ALPHA 0.10f  // 最大滤波系数
#define DISPLAY_FILTER_MUM 6            // 阶数
#define RC_FILTER_DIFF_MAX 25.0f
#define RC_FILTER_DIFF_MIN 10.0f
// 自适应RC滤波
float32_t app_DisplayFilter_RC(float32_t Cur, float32_t Tar)
{
    static float32_t filtered[DISPLAY_FILTER_MUM] = {0};
    static uint8_t oneState = 0;
    float32_t cur_temp = Cur;
    float32_t tar_temp = Tar;

    float32_t alpha = DISPLAY_FILTER_BASE_ALPHA;
    float32_t diff = fabsf(cur_temp - tar_temp);

    if (AllStatus_S.OneState_TempOk)
    {
        if ((uint32_t)AllStatus_S.pid_s.pid_out < AllStatus_S.pid_s.outPriod && ((cur_temp - tar_temp) < SOLDERING_TEMP_DISPLAY_REDUCE))
        {
            filtered[DISPLAY_FILTER_MUM - 1] = Tar; // 阶数切换
            filtered[DISPLAY_FILTER_MUM - 2] = Tar; // 阶数切换
            filtered[DISPLAY_FILTER_MUM - 3] = Tar; // 阶数切换
            filtered[DISPLAY_FILTER_MUM - 4] = Tar; // 阶数切换
        }
        if (!oneState)
        {
            filtered[DISPLAY_FILTER_MUM - 1] = Tar; // 阶数切换
            filtered[DISPLAY_FILTER_MUM - 2] = Tar; // 阶数切换
            filtered[DISPLAY_FILTER_MUM - 3] = Tar; // 阶数切换
            filtered[DISPLAY_FILTER_MUM - 4] = Tar; // 阶数切换
            filtered[DISPLAY_FILTER_MUM - 5] = Tar; // 阶数切换
            filtered[DISPLAY_FILTER_MUM - 6] = Tar; // 阶数切换
            oneState = 1;
        }

        if (diff > RC_FILTER_DIFF_MAX)
            alpha = DISPLAY_FILTER_MAX_ALPHA;
        else if (diff < RC_FILTER_DIFF_MIN)
            alpha = DISPLAY_FILTER_MIN_ALPHA;
        else
            alpha = DISPLAY_FILTER_BASE_ALPHA;
    }
    else
    {
        oneState = 0;
    }

    filtered[0] = filtered[0] * (1.0f - alpha) + cur_temp * alpha;
    for (int i = 1; i < DISPLAY_FILTER_MUM; i++)
    {
        filtered[i] = filtered[i] * (1.0f - alpha) + filtered[i - 1] * alpha;
    }

    if (AllStatus_S.OneState_TempOk && ((uint32_t)AllStatus_S.pid_s.pid_out < AllStatus_S.pid_s.outPriod_max))
        return filtered[DISPLAY_FILTER_MUM - 1];
    else
        return filtered[3];
}

#define KALMAN_BASE_Q 4.0f       // 过程噪声协方差（加热时）
#define KALMAN_MIN_Q 0.5f        // 过程噪声协方差（到达目标温度时）
#define KALMAN_MAX_Q 8.0f        // 最大过程噪声协方差
#define KALMAN_R_BASE 4.0f       // 基础测量噪声协方差
#define KALMAN_R_MIN 1.0f        // 最小测量噪声协方差
#define KALMAN_R_MAX 8.0f        // 最大测量噪声协方差
#define KALMAN_DIFF_THRESH 10.0f // 目标温度±范围，进入稳态

// 卡尔曼滤波（自适应参数）
static TYPEDEF_KALMAN_S display_kalman_inst = {
    .x = 0.0f,
    .p = 1.0f,
    .diff = 0.0f,
    .q = KALMAN_BASE_Q,
    .r = KALMAN_R_BASE,
    .k = 0.0f};

float32_t app_DisplayFilter_kalman(float32_t Cur, float32_t Tar)
{
    float32_t cur_temp = Cur;
    float32_t tar_temp = Tar;

    display_kalman_inst.diff = fabsf(cur_temp - tar_temp);
    float32_t diff = display_kalman_inst.diff;

    // 自适应调整过程噪声协方差Q
    float32_t q;
    if (diff < KALMAN_DIFF_THRESH)
        q = KALMAN_MIN_Q + (KALMAN_BASE_Q - KALMAN_MIN_Q) * (diff / KALMAN_DIFF_THRESH);
    else
        q = KALMAN_BASE_Q + (KALMAN_MAX_Q - KALMAN_BASE_Q) * ((diff - KALMAN_DIFF_THRESH) / KALMAN_DIFF_THRESH);
    if (q < KALMAN_MIN_Q)
        q = KALMAN_MIN_Q;
    if (q > KALMAN_MAX_Q)
        q = KALMAN_MAX_Q;
    display_kalman_inst.q = q;

    // 自适应调整测量噪声协方差R
    float32_t r;
    if (diff < KALMAN_DIFF_THRESH)
        r = KALMAN_R_MIN + (KALMAN_R_BASE - KALMAN_R_MIN) * (diff / KALMAN_DIFF_THRESH);
    else
        r = KALMAN_R_BASE + (KALMAN_R_MAX - KALMAN_R_BASE) * ((diff - KALMAN_DIFF_THRESH) / KALMAN_DIFF_THRESH);
    if (r < KALMAN_R_MIN)
        r = KALMAN_R_MIN;
    if (r > KALMAN_R_MAX)
        r = KALMAN_R_MAX;
    display_kalman_inst.r = r;

    // 预测
    display_kalman_inst.p += display_kalman_inst.q;
    // 更新
    display_kalman_inst.k = display_kalman_inst.p / (display_kalman_inst.p + display_kalman_inst.r);
    display_kalman_inst.x += display_kalman_inst.k * (cur_temp - display_kalman_inst.x);
    display_kalman_inst.p *= (1.0f - display_kalman_inst.k);

    return display_kalman_inst.x;
}

//*****************************************************************************************/
