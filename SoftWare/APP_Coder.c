#include "main.h"

#define BUTTON_SCAN_INTERVAL_MS 100
#define BUTTON_LONG_PRESS_MS 1500
#define BUTTON_LONG_PRESS_CNT (BUTTON_LONG_PRESS_MS / BUTTON_SCAN_INTERVAL_MS)

static void app_Encoder_TurnRightOrLeft(uint8_t addOrSub)
{
    if (AllStatus_S.Seting.SetingPage)
    {
        app_EncoderSetData_LcdSettingPage(addOrSub); // 进入设设置页面进入项目，旋转设置P1~P3参数
        Drive_Buz_OnOff(BUZ_20MS, BUZ_FREQ_CHANGE_OFF, USE_BUZ_TYPE);
    }
    else
    {
        app_LcdCommonMode(addOrSub); // 未进入设设置页面，旋转设置温度
        Drive_Buz_OnOff(BUZ_20MS, BUZ_FREQ_CHANGE_ON, USE_BUZ_TYPE);
        AllStatus_S.OneState_TempOk = 0;
    }
}

static uint8_t TarTempSaveInFlash_temp = 0;
void APP_TarTempSaveInFlash_Task(void)
{

    if (AllStatus_S.Old_TarTemp != AllStatus_S.flashSave_s.TarTemp)
    {

        TarTempSaveInFlash_temp++;
        if (TarTempSaveInFlash_temp > NEW_TEMP_SAVE_TIME)
        {
            Drive_FlashSaveData();

            AllStatus_S.Old_TarTemp = AllStatus_S.flashSave_s.TarTemp;
            TarTempSaveInFlash_temp = 0;
        }
    }
}

// 编码器任务
void app_Encoder_Task(void)
{
    if (AllStatus_S.encoder_s.EncoderIsRun)
    {
        Drive_BackLed_OnOff(1); // 点亮背光
        AllStatus_S.sleep_cnt = 0;
        AllStatus_S.encoder_s.EncoderIsRun = 0;
        TarTempSaveInFlash_temp = 0;
        app_Encoder_TurnRightOrLeft(AllStatus_S.encoder_s.TurnleftOrRight);
    }
}

static void app_ButtonDispose(void)
{
    if (AllStatus_S.encoder_s.ButtonIsLongPress) // 长按事件
    {
        AllStatus_S.encoder_s.ButtonIsLongPress = 0;
        if (AllStatus_S.Seting.SetingPage)
        {
            app_exitSeting_Lcd(); // 退出设置界面
            Drive_Buz_OnOff(BUZ_1S, BUZ_FREQ_CHANGE_OFF, USE_BUZ_TYPE);
            Drive_FlashSaveData(); // 保存数据
            AllStatus_S.Seting.SetingPage = 0;
        }
        else
        {
            Drive_Buz_OnOff(BUZ_DOROMI, BUZ_FREQ_CHANGE_OFF, USE_BUZ_TYPE);
            app_joinSeting_Lcd(); // 进入设置界面
            AllStatus_S.Seting.SetingPage = 1;
        }
    }
    if (AllStatus_S.Seting.SetingPage) // 短按事件
    {
        if (AllStatus_S.encoder_s.ButtonIsTrigeer)
        {
            AllStatus_S.encoder_s.ButtonIsTrigeer = 0;
            app_Lcd_DisplayPNumber_SettingPage(1);
        }
    }
}

void app_ButtonScan(void)
{
    static uint16_t press_cnt = 0;
    static uint8_t long_press_flag = 0;
    static uint8_t released_after_long = 0;
    uint32_t btn_state = Drive_BUTTON_GetState();

    if (btn_state) // 按下
    {
        if (press_cnt < BUTTON_LONG_PRESS_CNT)
            press_cnt++;
        if (press_cnt >= BUTTON_LONG_PRESS_CNT && !long_press_flag)
        {
            AllStatus_S.encoder_s.ButtonIsLongPress = 1;
            long_press_flag = 1;
            released_after_long = 0;
        }
    }
    else // 松开
    {
        if (press_cnt > 0 && press_cnt < BUTTON_LONG_PRESS_CNT)
        {
            AllStatus_S.encoder_s.ButtonIsTrigeer = 1;
        }
        if (press_cnt >= BUTTON_LONG_PRESS_CNT && long_press_flag && !released_after_long)
        {
            // 长按松开后只允许置1一次
            released_after_long = 1;
        }
        press_cnt = 0;
        long_press_flag = 0;
    }
}

// 编码器按键任务
void app_ButtonCheck_Task(void)
{
    static uint32_t buttonFlag = 0;
    uint32_t curButton = Drive_BUTTON_GetState();

    if (curButton != buttonFlag) // 按键蜂鸣器提升声
    {
        buttonFlag = curButton;
        if (curButton)
        {
            Drive_Buz_OnOff(BUZ_20MS, BUZ_FREQ_CHANGE_OFF, USE_BUZ_TYPE);
            Drive_BackLed_OnOff(1); // 点亮背光
            AllStatus_S.sleep_cnt = 0;
        }
    }
    app_ButtonScan();
    app_ButtonDispose();
}
