#include "main.h"

// 休眠检测相关宏定义
#define SLEEP_ADC_STABLE_RANGE 0x200 // ADC稳定范围
#define SLEEP_ADC_TASK_PERIOD_MS 500 // 任务调用周期（ms）

// 获取ADC值
uint32_t APP_Sleep_GetAdcValue(void)
{
    AllStatus_S.adc_value[SLEEP_NUM] = Drive_ADCConvert(SLEEP_NUM);
    return AllStatus_S.adc_value[SLEEP_NUM];
}

// 休眠控制任务，每500ms调用一次
void APP_Sleep_Control_Task(void)
{
    static uint32_t last_adc_value = 0;
    static uint32_t stable_time_ms = 0;
    uint32_t cur_adc_value = APP_Sleep_GetAdcValue();

    // 若当前已处于休眠状态，则跳过温度波动判定块
    if (AllStatus_S.SolderingState != SOLDERING_STATE_SLEEP)
    {
        // 若温度波动超出目标 ±2，清空计时并置为OK
        {
            float temp_val = (float)AllStatus_S.data_filter_prev[SOLDERING_TEMP210_NUM];
            float tar = (float)AllStatus_S.flashSave_s.TarTemp;
            if ((temp_val > tar + 2.0f) || (temp_val < tar - 2.0f))
            {
                stable_time_ms = 0;
            }
        }
    }

    // 首次赋值
    if (last_adc_value == 0)
        last_adc_value = cur_adc_value;

    // 判断ADC值是否在稳定范围内
    if ((cur_adc_value >= last_adc_value - SLEEP_ADC_STABLE_RANGE) &&
        (cur_adc_value <= last_adc_value + SLEEP_ADC_STABLE_RANGE))
    {
        // 累计稳定时间
        if (stable_time_ms < AllStatus_S.flashSave_s.SleepDelayTime * 1000)
            stable_time_ms += SLEEP_ADC_TASK_PERIOD_MS;

        // 达到休眠判定时间，进入休眠
        if (stable_time_ms >= AllStatus_S.flashSave_s.SleepDelayTime * 1000)
        {
            AllStatus_S.SolderingState = SOLDERING_STATE_SLEEP;
        }
    }
    else
    {
        // ADC变动，退出休眠
        stable_time_ms = 0;
        AllStatus_S.SolderingState = SOLDERING_STATE_OK;
        last_adc_value = cur_adc_value;
    }
}

// PWM输出3阶RC滤波相关宏
#define PWM_FILTER_ORDER 3
#define PWM_FILTER_DEFAULT_RC_MS 100.0f           // 默认RC时间常数（ms）
#define PWM_FILTER_DT_MS SLEEP_ADC_TASK_PERIOD_MS // 采样周期（ms），与任务周期一致

uint32_t APP_Sleep_PwmOutFilter(void)
{
    // 3阶级联一阶RC低通滤波，使用AllStatus_S.pid_s.pid_out作为RC时间常数（ms）
    static float stage[PWM_FILTER_ORDER];
    static uint8_t initialized = 0;

    // 使用ADC当前值作为滤波输入（也可根据实际需求改为其他信号）
    uint32_t input_u32 = AllStatus_S.adc_value[SLEEP_NUM];
    float input = (float)input_u32;

    // 取pid_out作为RC时间常数，若为0则使用默认值
    float rc_ms = (AllStatus_S.pid_s.pid_out > 0.0f) ? (float)AllStatus_S.pid_s.pid_out : PWM_FILTER_DEFAULT_RC_MS;
    float dt = (float)PWM_FILTER_DT_MS;

    // 计算一次积分器的alpha系数：alpha = dt / (RC + dt)
    float alpha = dt / (rc_ms + dt);
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 1.0f)
        alpha = 1.0f;

    // 首次调用时将各级初始化为输入值，避免上电突变
    if (!initialized)
    {
        for (int i = 0; i < PWM_FILTER_ORDER; ++i)
            stage[i] = input;
        initialized = 1;
    }

    // 级联更新：每级都是一阶RC的离散形式 x += alpha * (u - x)
    stage[0] += alpha * (input - stage[0]);
    for (int i = 1; i < PWM_FILTER_ORDER; ++i)
        stage[i] += alpha * (stage[i - 1] - stage[i]);

    // 返回最后一级滤波结果（四舍五入）
    return (uint32_t)(stage[PWM_FILTER_ORDER - 1] + 0.5f);
}
