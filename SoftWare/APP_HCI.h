#ifndef __APP_HCI_H__
#define __APP_HCI_H__

void app_Buz_Task(void);

void app_joinSeting_Lcd(void);
void app_exitSeting_Lcd(void);

void app_LcdCommonMode(uint8_t addOrSub);

void app_EncoderSetData_LcdSettingPage(uint8_t addOrSub);
void app_Lcd_DisplayPNumber_SettingPage(uint8_t addOrSub);

void app_Lcd_SolderingIcon_Blink_Task(void); // 函数调用周期（50ms）
void APP_SolderingOpenStateCheck_Task(void);     // 函数调用周期（500ms）
void app_IconBlink_Task(void);               // 选项闪烁
void app_SolderingTempDisplay(void);         // 实时温度显示
void APP_SleepCloseBackLight_Task(void);     // 休眠关闭背光
#endif
