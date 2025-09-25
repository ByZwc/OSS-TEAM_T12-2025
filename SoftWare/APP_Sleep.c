#include "main.h"

// 休眠检测相关宏定义
#define SLEEP_ADC_STABLE_RANGE 0x200 // ADC稳定范围
#define SLEEP_ADC_TASK_PERIOD_MS 250 // 任务调用周期（ms）

// 获取ADC值
static uint32_t APP_Sleep_GetAdcValue(void)
{
    AllStatus_S.adc_value[SLEEP_NUM] = Drive_ADCConvert(SLEEP_NUM);
    return AllStatus_S.adc_value[SLEEP_NUM];
}

static float32_t APP_Sleep_PowerCheck(void)
{
    float tar = (float)AllStatus_S.flashSave_s.TarTemp;
    static float minT = 100.0f;
    static float maxT = 450.0f;
    static float minP = 8.0f;
    static float maxP = 20.0f;

    if (tar <= minT)
        return minP;
    if (tar >= maxT)
        return maxP;

    return minP + (tar - minT) * (maxP - minP) / (maxT - minT);
}

static float32_t APP_Sleep_TempCheck(void)
{
    float tar = (float)AllStatus_S.flashSave_s.TarTemp;
    static float minT = 100.0f;
    static float maxT = 450.0f;
    static float minRange = 2.0f;
    static float maxRange = 3.0f;

    if (tar <= minT)
        return minRange;
    if (tar >= maxT)
        return maxRange;

    return minRange + (tar - minT) * (maxRange - minRange) / (maxT - minT);
}

// 休眠控制任务，每500ms调用一次
void APP_Sleep_Control_Task(void)
{
    static uint32_t last_adc_value = 0;
    static uint32_t stable_time_ms = 0;
    static uint8_t oneState = 0;
    uint32_t cur_adc_value = APP_Sleep_GetAdcValue();

    if (AllStatus_S.SolderingState == SOLDERING_STATE_PULL_OUT_ERROR || AllStatus_S.SolderingState == SOLDERING_STATE_SHORTCIR_ERROR)
    {
        stable_time_ms = 0;
        return;
    }

    if (AllStatus_S.SolderingState == SOLDERING_STATE_OK)
    {
        if (AllStatus_S.Power > APP_Sleep_PowerCheck())
        {
            stable_time_ms = 0;
        }
    }

    if (AllStatus_S.SolderingState == SOLDERING_STATE_OK)
    {
        float temp_diff = AllStatus_S.data_filter_prev[SOLDERING_TEMP210_NUM] - (float32_t)AllStatus_S.flashSave_s.TarTemp;
        if (temp_diff < 0.0f)
            temp_diff = -temp_diff;

        if (temp_diff > APP_Sleep_TempCheck())
        {
            stable_time_ms = 0;
        }
    }

    // 首次赋值（使用初始化标志，避免与合法0值混淆）
    static uint8_t last_initialized = 0;
    if (!last_initialized)
    {
        last_adc_value = cur_adc_value;
        last_initialized = 1;
    }

    // 判断ADC值是否在稳定范围内（先计算绝对差，避免无符号下溢）
    {
        uint32_t diff = (cur_adc_value >= last_adc_value) ? (cur_adc_value - last_adc_value) : (last_adc_value - cur_adc_value);
        if (diff <= SLEEP_ADC_STABLE_RANGE)
        {
            // 累计稳定时间
            if (stable_time_ms < AllStatus_S.flashSave_s.SleepDelayTime * 1000)
                stable_time_ms += SLEEP_ADC_TASK_PERIOD_MS;

            // 达到休眠判定时间，进入休眠
            if (stable_time_ms >= AllStatus_S.flashSave_s.SleepDelayTime * 1000)
            {
                AllStatus_S.SolderingState = SOLDERING_STATE_SLEEP;
                if (!oneState)
                {
                    Lcd_icon_onOff(icon_soldering, 1);
                    app_pidOutCmd();
                    Drive_Buz_OnOff(BUZ_20MS, BUZ_FREQ_CHANGE_OFF, USE_BUZ_TYPE);
                    if (!AllStatus_S.Seting.SetingPage)
                        Lcd_icon_onOff(icon_temp, 1); // 点亮℃图标
                    oneState = 1;
                }
                if (AllStatus_S.CurTemp < SLEEP_DEEP_TEMP_RANGE)
                    AllStatus_S.SolderingState = SOLDERING_STATE_SLEEP_DEEP;
            }
        }
        else
        {
            // ADC变动，退出休眠
            stable_time_ms = 0;
            if (oneState)
            {
                oneState = 0;
                Drive_BackLed_OnOff(1); // 点亮背光
                AllStatus_S.SolderingState = SOLDERING_STATE_OK;
                Drive_Buz_OnOff(BUZ_20MS, BUZ_FREQ_CHANGE_OFF, USE_BUZ_TYPE);
                Lcd_icon_onOff(icon_soldering, 0);
                if (!AllStatus_S.Seting.SetingPage)
                {
                    if (AllStatus_S.flashSave_s.DisplayPowerOnOff)
                        Lcd_icon_onOff(icon_temp, 0); // 熄灭℃图标
                    else
                        Lcd_icon_onOff(icon_temp, 1); // 点亮℃图标
                }
            }
            last_adc_value = cur_adc_value;
        }
    }
}

#define PWM_FILTER_ORDER 3
#define PWM_FILTER_DEFAULT_RC_MS 5.0f             // 默认RC时间常数（ms）
#define PWM_FILTER_DT_MS SLEEP_ADC_TASK_PERIOD_MS // 采样周期（ms），与任务周期一致

uint32_t APP_Sleep_PwmOutFilter(void)
{
    static float stage[PWM_FILTER_ORDER];
    static uint8_t initialized = 0;

    float input = AllStatus_S.pid_s.pid_out;

    float rc_ms = PWM_FILTER_DEFAULT_RC_MS;
    if (rc_ms <= 0.0f)
        rc_ms = PWM_FILTER_DEFAULT_RC_MS;

    float dt = (float)PWM_FILTER_DT_MS;

    // 计算一次积分器的alpha系数：alpha = dt / (RC + dt)
    float alpha = dt / (rc_ms + dt);
    if (alpha < 0.0f)
        alpha = 0.0f;
    else if (alpha > 1.0f)
        alpha = 1.0f;

    // 首次调用时将各级初始化为输入值，避免上电突变
    if (!initialized)
    {
        for (int i = 0; i < PWM_FILTER_ORDER; ++i)
            stage[i] = input;
        initialized = 1;
    }

    // 一阶RC的离散形式 x += alpha * (u - x)
    stage[0] += alpha * (input - stage[0]);
    for (int i = 1; i < PWM_FILTER_ORDER; ++i)
        stage[i] += alpha * (stage[i - 1] - stage[i]);

    float out = stage[PWM_FILTER_ORDER - 1];
    if (out < 0.0f)
        return 0u;
    return (uint32_t)(out + 0.5f);
}
