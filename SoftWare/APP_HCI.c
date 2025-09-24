#include "main.h"

// 进入设置模式
void app_joinSeting_Lcd(void)
{
    Lcd_icon_onOff(icon_400Temp, 0);
    Lcd_icon_onOff(icon_350Temp, 0);
    Lcd_icon_onOff(icon_300Temp, 0);
    Lcd_smgUp3_SetPNum(AllStatus_S.Seting.PNumber);
    switch (AllStatus_S.Seting.PNumber)
    {
    case SMG_P01:
        Lcd_icon_onOff(icon_temp, 0); // 熄灭℃图标
        Lcd_smgDowm3_DisplayOnOff(AllStatus_S.flashSave_s.BuzOnOff, 1);
        break;
    case SMG_P02:
        Lcd_icon_onOff(icon_temp, 0); // 熄灭℃图标
        Lcd_smgDowm3_DisplayOnOff(AllStatus_S.flashSave_s.PreinstallTempOnOff, 1);
        break;
    case SMG_P03:
        Lcd_icon_onOff(icon_temp, 1); // 点亮℃图标
        Lcd_smgDowm3_SetNimus(AllStatus_S.flashSave_s.calibration_temp, 1);
        break;
    case SMG_P04:
        Lcd_icon_onOff(icon_temp, 0); // 熄灭℃图标
        Lcd_smgDowm3_DisplayOnOff(AllStatus_S.flashSave_s.BackgroundLightOnoff, 1);
        break;
    case SMG_P05:
        Lcd_icon_onOff(icon_temp, 0); // 熄灭℃图标
        Lcd_smgDowm3_SetNimus(AllStatus_S.flashSave_s.SleepDelayTime, 1);
        break;
    case SMG_P06:
        Lcd_icon_onOff(icon_temp, 0); // 熄灭℃图标
        Lcd_smgDowm3_DisplayOnOff(AllStatus_S.flashSave_s.DisplayPowerOnOff, 1);
        break;
    }
}

// 退出设置模式
void app_exitSeting_Lcd(void)
{
    Lcd_icon_onOff(icon_400Temp, 1);
    Lcd_icon_onOff(icon_350Temp, 1);
    Lcd_icon_onOff(icon_300Temp, 1);
    if (AllStatus_S.flashSave_s.PreinstallTempOnOff)
    {
        switch (AllStatus_S.flashSave_s.PreinstallTempNum)
        {
        case PREINSTALL_TMEP300:
            AllStatus_S.flashSave_s.TarTemp = 300;
            break;
        case PREINSTALL_TMEP350:
            AllStatus_S.flashSave_s.TarTemp = 350;
            break;
        case PREINSTALL_TMEP400:
            AllStatus_S.flashSave_s.TarTemp = 400;
            break;
        }
    }
    AllStatus_S.Seting.PNumber = 1;
    Lcd_smgUp3_SetNum(AllStatus_S.flashSave_s.TarTemp); // 恢复温度显示
    if (!AllStatus_S.flashSave_s.DisplayPowerOnOff)
        Lcd_icon_onOff(icon_temp, 1); // 点亮℃图标
}

// 编码器快速调温相关宏定义
#define ENCODER_FASTSET_TIME_WINDOW_MS 1000 // 基础时间窗口（ms），每增加一个等级扩展一个窗口
#define ENCODER_FASTSET_BUCKET_SIZE 3       // 每个等级所需的次数（3次/等级）
#define ENCODER_FASTSET_STEP_UNIT 5         // 步进单位（5,10,15,20,25）
#define ENCODER_FASTSET_MAX_LEVEL 5         // 最大等级（1..5），对应最大返回值25

uint8_t app_Encoder_FastSetTemp(void)
{
    // 记录连续旋转的起始时间与次数；只要间隔不超过 MAX_LEVEL * WINDOW，就认为是一次连续操作序列
    static uint32_t window_start_tick = 0;
    static uint16_t count_in_window = 0;
    uint32_t now_tick = uwTick;

    uint32_t max_continuous_window = ENCODER_FASTSET_TIME_WINDOW_MS * ENCODER_FASTSET_MAX_LEVEL;

    // 首次或超出最长连续时间则重置序列
    if (window_start_tick == 0 || (uint32_t)(now_tick - window_start_tick) > max_continuous_window)
    {
        window_start_tick = now_tick;
        count_in_window = 1;
    }
    else
    {
        // 在连续序列内，增加计数并保持起始时间不变（以保证随次数增长等级上升）
        if (count_in_window < 0xFFFF)
            count_in_window++;
    }

    // 计算等级：每 ENCODER_FASTSET_BUCKET_SIZE 次提升一级
    uint16_t level = (count_in_window + (ENCODER_FASTSET_BUCKET_SIZE - 1)) / ENCODER_FASTSET_BUCKET_SIZE;
    if (level < 1)
        level = 1;
    if (level > ENCODER_FASTSET_MAX_LEVEL)
        level = ENCODER_FASTSET_MAX_LEVEL;

    uint8_t step = (uint8_t)(level * ENCODER_FASTSET_STEP_UNIT);
    return step; // 返回 5,10,15,20 或 25
}

// 普通模式或预设温度模式
void app_LcdCommonMode(uint8_t addOrSub)
{
    if (AllStatus_S.flashSave_s.PreinstallTempOnOff)
    {
        if (addOrSub)
        {
            if (AllStatus_S.flashSave_s.PreinstallTempNum < 3)
                AllStatus_S.flashSave_s.PreinstallTempNum++;
        }
        else
        {
            if (AllStatus_S.flashSave_s.PreinstallTempNum > 1)
                AllStatus_S.flashSave_s.PreinstallTempNum--;
        }
        Lcd_icon_onOff(icon_400Temp, 1);
        Lcd_icon_onOff(icon_350Temp, 1);
        Lcd_icon_onOff(icon_300Temp, 1);
        AllStatus_S.flashSave_s.TarTemp = 300 + (AllStatus_S.flashSave_s.PreinstallTempNum - 1) * 50;
        Lcd_smgUp3_SetNum(AllStatus_S.flashSave_s.TarTemp); // 恢复温度显示
    }
    else
    {
        if (addOrSub)
        {
            if (AllStatus_S.flashSave_s.TarTemp < MAX_TAR_TEMP)
                AllStatus_S.flashSave_s.TarTemp += app_Encoder_FastSetTemp();
            if (AllStatus_S.flashSave_s.TarTemp > MAX_TAR_TEMP)
                AllStatus_S.flashSave_s.TarTemp = MAX_TAR_TEMP;
        }
        else
        {
            if (AllStatus_S.flashSave_s.TarTemp >= MIN_TAR_TEMP)
                AllStatus_S.flashSave_s.TarTemp -= app_Encoder_FastSetTemp();
            if (AllStatus_S.flashSave_s.TarTemp < MIN_TAR_TEMP - 5)
                AllStatus_S.flashSave_s.TarTemp = MIN_TAR_TEMP - 5;
        }
        Lcd_smgUp3_SetNum(AllStatus_S.flashSave_s.TarTemp); // 恢复温度显示
    }
}

// 设置模式，编码器左右转
void app_EncoderSetData_LcdSettingPage(uint8_t addOrSub)
{
    switch (AllStatus_S.Seting.PNumber)
    {
    case SMG_P01:
        if (AllStatus_S.flashSave_s.BuzOnOff)
            AllStatus_S.flashSave_s.BuzOnOff = 0;
        else
            AllStatus_S.flashSave_s.BuzOnOff = 1;
        Lcd_smgDowm3_DisplayOnOff(AllStatus_S.flashSave_s.BuzOnOff, 1);
        break;
    case SMG_P02:
        if (AllStatus_S.flashSave_s.PreinstallTempOnOff)
            AllStatus_S.flashSave_s.PreinstallTempOnOff = 0;
        else
            AllStatus_S.flashSave_s.PreinstallTempOnOff = 1;
        Lcd_smgDowm3_DisplayOnOff(AllStatus_S.flashSave_s.PreinstallTempOnOff, 1);
        break;
    case SMG_P03:
        if (addOrSub)
        {
            if (AllStatus_S.flashSave_s.calibration_temp < CALIBRATION_TEMP_MAX)
                AllStatus_S.flashSave_s.calibration_temp++;
        }
        else
        {
            if (AllStatus_S.flashSave_s.calibration_temp > (CALIBRATION_TEMP_MIN))
                AllStatus_S.flashSave_s.calibration_temp--;
        }
        Lcd_smgDowm3_SetNimus(AllStatus_S.flashSave_s.calibration_temp, 1);
        break;
    case SMG_P04:
        if (AllStatus_S.flashSave_s.BackgroundLightOnoff)
            AllStatus_S.flashSave_s.BackgroundLightOnoff = 0;
        else
            AllStatus_S.flashSave_s.BackgroundLightOnoff = 1;
        Lcd_smgDowm3_DisplayOnOff(AllStatus_S.flashSave_s.BackgroundLightOnoff, 1);
        break;
    case SMG_P05:
        if (addOrSub)
        {
            if (AllStatus_S.flashSave_s.SleepDelayTime < SLEEP_DELAY_TIME_MAX)
                AllStatus_S.flashSave_s.SleepDelayTime += app_Encoder_FastSetTemp();
            if (AllStatus_S.flashSave_s.SleepDelayTime > SLEEP_DELAY_TIME_MAX)
                AllStatus_S.flashSave_s.SleepDelayTime = SLEEP_DELAY_TIME_MAX;
        }
        else
        {
            if (AllStatus_S.flashSave_s.SleepDelayTime > SLEEP_DELAY_TIME_MIN + 5)
                AllStatus_S.flashSave_s.SleepDelayTime -= app_Encoder_FastSetTemp();
            if (AllStatus_S.flashSave_s.SleepDelayTime < SLEEP_DELAY_TIME_MIN)
                AllStatus_S.flashSave_s.SleepDelayTime = SLEEP_DELAY_TIME_MIN;
        }
        Lcd_smgDowm3_SetNimus(AllStatus_S.flashSave_s.SleepDelayTime, 1);
        break;
    case SMG_P06:
        if (AllStatus_S.flashSave_s.DisplayPowerOnOff)
            AllStatus_S.flashSave_s.DisplayPowerOnOff = 0;
        else
            AllStatus_S.flashSave_s.DisplayPowerOnOff = 1;
        Lcd_smgDowm3_DisplayOnOff(AllStatus_S.flashSave_s.DisplayPowerOnOff, 1);
        break;
    }
}

/*
 * @brief 显示当前参数编号
 * @param addOrSub: 1-加，0-减
 * @return
 */
void app_Lcd_DisplayPNumber_SettingPage(uint8_t addOrSub)
{
    if (addOrSub)
    {
        AllStatus_S.Seting.PNumber++;
        if (AllStatus_S.Seting.PNumber > OPTION_NUM)
            AllStatus_S.Seting.PNumber = 1;
    }
    else
    {
        AllStatus_S.Seting.PNumber--;
        if (AllStatus_S.Seting.PNumber < 1)
            AllStatus_S.Seting.PNumber = OPTION_NUM;
    }

    Lcd_smgUp3_SetPNum(AllStatus_S.Seting.PNumber);
    switch (AllStatus_S.Seting.PNumber)
    {
    case SMG_P01:
        Lcd_icon_onOff(icon_temp, 0); // 熄灭℃图标
        Lcd_smgDowm3_DisplayOnOff(AllStatus_S.flashSave_s.BuzOnOff, 1);
        break;
    case SMG_P02:
        Lcd_icon_onOff(icon_temp, 0); // 熄灭℃图标
        Lcd_smgDowm3_DisplayOnOff(AllStatus_S.flashSave_s.PreinstallTempOnOff, 1);
        break;
    case SMG_P03:
        Lcd_icon_onOff(icon_temp, 1); // 点亮℃图标
        Lcd_smgDowm3_SetNimus(AllStatus_S.flashSave_s.calibration_temp, 1);
        break;
    case SMG_P04:
        Lcd_icon_onOff(icon_temp, 0); // 熄灭℃图标
        Lcd_smgDowm3_DisplayOnOff(AllStatus_S.flashSave_s.BackgroundLightOnoff, 1);
        break;
    case SMG_P05:
        Lcd_icon_onOff(icon_temp, 0); // 熄灭℃图标
        Lcd_smgDowm3_SetNimus(AllStatus_S.flashSave_s.SleepDelayTime, 1);
        break;
    case SMG_P06:
        Lcd_icon_onOff(icon_temp, 0); // 熄灭℃图标
        Lcd_smgDowm3_DisplayOnOff(AllStatus_S.flashSave_s.DisplayPowerOnOff, 1);
        break;
    }
}

// 宏定义：点亮时长（ms）、最快闪烁次数（Hz）、最慢闪烁次数（Hz）
#define SOLDER_ICON_ON_TIME_MS 100          // 点亮时长
#define SOLDER_ICON_BLINK_FAST_HZ 5         // 最快闪烁次数
#define SOLDER_ICON_BLINK_SLOW_HZ 0.25f     // 最慢闪烁次数
#define SOLDER_ICON_BLINK_PERIOD 100        // 调用周期（50ms）
void app_Lcd_SolderingIcon_Blink_Task(void) // 函数调用周期（50ms）
{
    static uint16_t blink_cnt = 0;
    static uint8_t icon_state = 0; // 0: off, 1: on

    if (AllStatus_S.pid_s.pid_out == 0 || AllStatus_S.pid_s.outCmd == 0)
    {
        blink_cnt = 0;
        icon_state = 0;
        Lcd_icon_onOff(icon_cup, 0); // 熄灭
        return;
    }

    // 计算占空比
    float duty = (float)AllStatus_S.pid_s.pid_out / (float)AllStatus_S.pid_s.pid_integration_max;
    if (duty > 1.0f)
        duty = 1.0f;
    if (duty < 0.0f)
        duty = 0.0f;

    // 计算周期（ms）
    float period_fast = 1000.0f / SOLDER_ICON_BLINK_FAST_HZ;
    float period_slow = 1000.0f / SOLDER_ICON_BLINK_SLOW_HZ;
    float period_ms = period_slow - (period_slow - period_fast) * duty;
    if (period_ms < period_fast)
        period_ms = period_fast;
    if (period_ms > period_slow)
        period_ms = period_slow;

    uint16_t on_cnt = SOLDER_ICON_ON_TIME_MS / SOLDER_ICON_BLINK_PERIOD;
    uint16_t off_cnt = (uint16_t)((period_ms - SOLDER_ICON_ON_TIME_MS) / SOLDER_ICON_BLINK_PERIOD);

    if (icon_state == 0) // off
    {
        blink_cnt++;
        if (blink_cnt >= off_cnt)
        {
            blink_cnt = 0;
            icon_state = 1;
            Lcd_icon_onOff(icon_cup, 1); // 点亮
        }
        else
        {
            Lcd_icon_onOff(icon_cup, 0); // 熄灭
        }
    }
    else // on
    {
        blink_cnt++;
        if (blink_cnt >= on_cnt)
        {
            blink_cnt = 0;
            icon_state = 0;
            Lcd_icon_onOff(icon_cup, 0); // 熄灭
        }
        else
        {
            Lcd_icon_onOff(icon_cup, 1); // 保持点亮
        }
    }
}

void APP_SolderingOpenStateCheck_Task(void) // 函数调用周期（500ms）
{
    if (AllStatus_S.SolderingState == SOLDERING_STATE_OK)
    {
        if (AllStatus_S.CurTemp >= SOLDERING_TEMP_OPEN)
        {
            AllStatus_S.SolderingState = SOLDERING_STATE_PULL_OUT_ERROR;
            Drive_Buz_OnOff(BUZ_1S, BUZ_FREQ_CHANGE_OFF, USE_BUZ_TYPE);
        }
    }
    else
    {
        if (AllStatus_S.SolderingState == SOLDERING_STATE_PULL_OUT_ERROR || AllStatus_S.SolderingState == SOLDERING_STATE_SHORTCIR_ERROR)
            APP_shortCircuitProtection();
    }
}

static void app_LcdBlink(uint8_t onOff)
{
    switch (AllStatus_S.flashSave_s.PreinstallTempNum)
    {
    case PREINSTALL_TMEP300:
        Lcd_icon_onOff(icon_300Temp, onOff);
        break;
    case PREINSTALL_TMEP350:
        Lcd_icon_onOff(icon_350Temp, onOff);
        break;
    case PREINSTALL_TMEP400:
        Lcd_icon_onOff(icon_400Temp, onOff);
        break;
    }
}

void app_IconBlink_Task(void)
{
    static uint8_t onOff = 0;
    onOff = !onOff;
    if ((!AllStatus_S.Seting.SetingPage) && (!(AllStatus_S.SolderingState == SOLDERING_STATE_SLEEP)) && (!(AllStatus_S.SolderingState == SOLDERING_STATE_SLEEP_DEEP)))
    {
        if (AllStatus_S.flashSave_s.PreinstallTempOnOff)
        {
            app_LcdBlink(onOff);
        }
        else
        {
            app_LcdBlink(1);
        }
    }
}

void app_SolderingTempDisplay(void)
{
    static uint32_t last_display_temp = 0.0f; // 睡眠防抖动

    static uint32_t display_temp_cnt = 0;
    static uint32_t display_temp_interval = 0;
    static uint32_t last_display_temp_tick = 0;
    float32_t diff = fabsf(AllStatus_S.data_filter_prev[SOLDERING_TEMP210_NUM] - (float32_t)AllStatus_S.flashSave_s.TarTemp);
    last_display_temp_tick++;

    if (AllStatus_S.OneState_TempOk && ((uint32_t)AllStatus_S.pid_s.pid_out < AllStatus_S.pid_s.outPriod_max))
        display_temp_interval = 10;
    else
        display_temp_interval = 1;

    if (last_display_temp_tick - display_temp_cnt >= display_temp_interval)
    {
        display_temp_cnt += display_temp_interval;
        if (!AllStatus_S.Seting.SetingPage)
        {
            switch (AllStatus_S.SolderingState)
            {
            case SOLDERING_STATE_SHORTCIR_ERROR: // 短路
                Lcd_smgDowm3_SetErrorNum(ERROR_E0, 1);
                AllStatus_S.OneState_TempOk = 0;
                break;
            case SOLDERING_STATE_PULL_OUT_ERROR: // 拔出手柄
                Lcd_smgDowm3_SetErrorNum(ERROR_E3, 1);
                AllStatus_S.OneState_TempOk = 0;
                break;
            case SOLDERING_STATE_SLEEP: // 进入睡眠
                if ((uint32_t)AllStatus_S.data_filter_prev[SOLDERING_TEMP210_NUM] < last_display_temp)
                {
                    Lcd_smgDowm3_SetNum((uint16_t)AllStatus_S.data_filter_prev[SOLDERING_TEMP210_NUM], 1);
                    last_display_temp = AllStatus_S.data_filter_prev[SOLDERING_TEMP210_NUM];
                }
                AllStatus_S.OneState_TempOk = 0;
                break;
            case SOLDERING_STATE_SLEEP_DEEP: // 进入深度睡眠
                Lcd_smgDowm3_SetErrorNum(DRIVE_SLEEP, 1);
                break;
            case SOLDERING_STATE_OK: // 正常状态

                // Lcd_smgDowm3_SetHex(AllStatus_S.adc_filter_value);    //温度原始ADC值，带FIR滤波
                // Lcd_smgDowm3_SetNum((uint16_t)AllStatus_S.data_filter[SOLDERING_ELECTRICITY_NUM], 1);    //估计值
                // Lcd_smgDowm3_SetNum((uint16_t)AllStatus_S.data_filter_prev[SOLDERING_ELECTRICITY_NUM], 1);   // 实时值
                // Lcd_smgDowm3_SetNum((uint16_t)AllStatus_S.Power, 1); // 功率值(互补滤波)
                // Lcd_smgDowm3_SetHex(AllStatus_S.adc_value[SLEEP_NUM]); // 睡眠ADC值

                if (AllStatus_S.flashSave_s.DisplayPowerOnOff)
                    Lcd_smgDowm3_SetNum((uint16_t)AllStatus_S.Power, 1);
                else
                    Lcd_smgDowm3_SetNum((uint16_t)AllStatus_S.data_filter_prev[SOLDERING_TEMP210_NUM], 1);

                if (diff < 1.0f) // 首次到达温度蜂鸣器响应
                {
                    if (!AllStatus_S.OneState_TempOk)
                        Drive_Buz_OnOff(BUZ_20MS, BUZ_FREQ_CHANGE_OFF, USE_BUZ_TYPE);
                    AllStatus_S.OneState_TempOk = 1;
                }
                last_display_temp = MAX_TAR_TEMP;
                break;
            }
        }
    }
}

void APP_SleepCloseBackLight_Task(void)
{
    if (AllStatus_S.SolderingState == SOLDERING_STATE_SLEEP_DEEP)
    {
        if (AllStatus_S.sleep_cnt < 61)
            AllStatus_S.sleep_cnt++;

        if (AllStatus_S.sleep_cnt == TIME_TO_CLOSE_BACKLIGHT)
        {
            if (AllStatus_S.flashSave_s.BackgroundLightOnoff)
            {
                Drive_BackLed_OnOff(0); // 关闭背光
                Drive_Buz_OnOff(BUZ_1S, BUZ_FREQ_CHANGE_OFF, USE_BUZ_TYPE);
            }
        }
    }
    else
    {
        AllStatus_S.sleep_cnt = 0;
    }
}
