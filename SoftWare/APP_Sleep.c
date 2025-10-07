#include "main.h"

// 休眠检测相关宏定义
#define SLEEP_ADC_STABLE_RANGE 0x400 // ADC稳定范围
#define SLEEP_ADC_TASK_PERIOD_MS 250 // 任务调用周期（ms）

// 获取ADC值
static uint32_t APP_Sleep_GetAdcValue(void)
{
    /* 采样率: 4Hz (任务周期250ms)
       目标截止频率: 0.5Hz
    */
    uint32_t raw = Drive_ADCConvert(SLEEP_NUM);
    AllStatus_S.adc_value[SLEEP_NUM] = raw;

    static uint8_t initialized = 0;
    static float filtered = 0.0f;

    static const float dt = 0.25f; // 1 / 4Hz
    static const float fc = 0.5f;  // 截止频率
    static const float RC = 1.0f / (2.0f * 3.1415926f * fc);
    static const float alpha = dt / (RC + dt);

    if (!initialized)
    {
        filtered = (float)raw;
        initialized = 1;
    }
    else
    {
        filtered += alpha * ((float)raw - filtered);
    }

    return (uint32_t)(filtered + 0.5f);
}

static uint32_t APP_Sleep_PowerCheck(void)
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
    static float maxRange = 4.0f;

    if (tar <= minT)
        return minRange;
    if (tar >= maxT)
        return maxRange;

    return minRange + (tar - minT) * (maxRange - minRange) / (maxT - minT);
}

// 休眠控制任务，每250ms调用一次
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
        AllStatus_S.Sleep_PowerFilter = APP_Sleep_PowerFilter();
        if (AllStatus_S.Sleep_PowerFilter > APP_Sleep_PowerCheck())
        {
            stable_time_ms = 0;
        }
    }

    if (AllStatus_S.SolderingState == SOLDERING_STATE_OK)
    {
        float temp_diff = (float32_t)AllStatus_S.flashSave_s.TarTemp - AllStatus_S.data_filter_prev[SOLDERING_TEMP210_NUM];

        if (temp_diff > APP_Sleep_TempCheck())
        {
            stable_time_ms = 0;
        }
    }

    static uint8_t last_initialized = 0;
    if (!last_initialized)
    {
        last_adc_value = cur_adc_value;
        last_initialized = 1;
    }

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
                if (!oneState)
                {
                    AllStatus_S.SolderingState = SOLDERING_STATE_SLEEP;
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

#define PWM_FILTER_ORDER 3                        // 滤波器阶数
#define PWM_FILTER_DEFAULT_RC_MS 500.0f           // RC时间常数（ms）
#define PWM_FILTER_DT_MS SLEEP_ADC_TASK_PERIOD_MS // 采样周期（ms）
/*
- 截止频率约 0.32Hz
- 截止点总衰减约 -9dB
*/
uint32_t APP_Sleep_PowerFilter(void)
{
    static float stage[PWM_FILTER_ORDER];
    static uint8_t initialized = 0;

    float input = AllStatus_S.Power;

    float rc_ms = PWM_FILTER_DEFAULT_RC_MS;
    if (rc_ms <= 0.0f)
        rc_ms = PWM_FILTER_DEFAULT_RC_MS;

    float dt = (float)PWM_FILTER_DT_MS;

    // alpha = dt / (RC + dt)
    float alpha = dt / (rc_ms + dt);
    if (alpha < 0.0f)
        alpha = 0.0f;
    else if (alpha > 1.0f)
        alpha = 1.0f;

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

void APP_SleepBackLight_Task(void)
{
    static uint8_t last_state = 0;

    if (AllStatus_S.SolderingState == SOLDERING_STATE_SLEEP_DEEP &&
        AllStatus_S.flashSave_s.BackgroundLightOnoff)
    {
        if (last_state != 1)
        {
            Drive_BackLed_PWMOut();
            last_state = 1;
        }
    }
    else
    {
        if (last_state != 2)
        {
            Drive_BackLed_Init(); // 常亮背光
            last_state = 2;
        }
    }
}
