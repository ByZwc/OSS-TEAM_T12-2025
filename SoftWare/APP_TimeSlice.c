#include "main.h"

#define PID_TASK_INTERVAL 10
#define ENCODER_TASK_INTERVAL 50
#define BUTTON_TASK_INTERVAL 100
#define SLEEP_TASK_INTERVAL 250
#define ICON_BLINK_TASK_INTERVAL 1000
void app_timeSlice_Task(void)
{
    static uint32_t last_pidTick = 0;
    static uint32_t last_sleepTick = 0;
    static uint32_t last_encoderTick = 0;
    static uint32_t last_IconBlinkTick = 0;
    static uint32_t last_ButtonTick = 0;

    if (uwTick - last_pidTick >= PID_TASK_INTERVAL)
    {
        last_pidTick += PID_TASK_INTERVAL;
        app_pid_Task();                     // PID任务
        app_GetAdcVlaue_electricity_Task(); // 估计电流任务
        app_Buz_Task();                     // 蜂鸣器任务
    }

    if (uwTick - last_encoderTick >= ENCODER_TASK_INTERVAL)
    {
        last_encoderTick += ENCODER_TASK_INTERVAL;
        app_Encoder_Task(); // 编码器任务
    }

    if (uwTick - last_sleepTick >= SLEEP_TASK_INTERVAL)
    {
        last_sleepTick += SLEEP_TASK_INTERVAL;
        APP_SolderingOpenStateCheck_Task(); // 拔出烙铁头或短路检测任务
        APP_Sleep_Control_Task();           // 休眠ADC采样任务
        HAL_IWDG_Refresh(&IwdgHandle);      // 清看门狗
    }

    if (uwTick - last_IconBlinkTick >= ICON_BLINK_TASK_INTERVAL)
    {
        last_IconBlinkTick += ICON_BLINK_TASK_INTERVAL;
        app_IconBlink_Task();           // 选项闪烁
        app_PcbTempProtect_Task();      // PCB温度过高保护
        APP_TarTempSaveInFlash_Task();  // 保存历史温度（非预设模式下）
        APP_SleepCloseBackLight_Task(); // 休眠关闭背光
    }

    if (uwTick - last_ButtonTick >= BUTTON_TASK_INTERVAL)
    {
        last_ButtonTick += BUTTON_TASK_INTERVAL;
        app_ButtonCheck_Task();               // 按钮检测任务
        APP_Power_complementaryFilter_Task(); // 功率互补滤波任务
        app_SolderingTempDisplay();           // 烙铁头温度显示任务
        app_Lcd_SolderingIcon_Blink_Task();   // 烙铁头图标闪烁任务
        Drive_DisplayLcd_sendData_Task();     // LCD显示信息发送任务
    }
    app_Samp_electricity_Task(); // 采样负载电流任务
}
