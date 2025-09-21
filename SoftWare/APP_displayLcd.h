#ifndef __APP_DISPLAYLCD_H__
#define __APP_DISPLAYLCD_H__

typedef enum
{
    icon_400Temp = 0,
    icon_350Temp,
    icon_300Temp,
    icon_SET,
    icon_soldering,
    icon_cup,
    icon_temp,

} LCD_ICON_TYPE;

void Lcd_icon_onOff(LCD_ICON_TYPE lcdIcon, uint8_t onOff);
void Lcd_smgDowm3_SetNum(uint16_t num, uint8_t OnOff);             // 显示0-999整数
void Lcd_smgDowm3_SetHex(uint16_t num);                            // 显示16进制
void Lcd_smgDowm3_DisplayOnOff(uint8_t OnOff_Icon, uint8_t OnOff); // 显示ON/OFF
void Lcd_smgDowm3_SetNimus(int16_t Nimus, uint8_t OnOff);          // 显示负数-50~50
void Lcd_smgDowm3_SetErrorNum(int16_t ErrorNum, uint8_t OnOff);    // 显示错误码

void Lcd_smgUp3_SetNum(uint16_t num);     // 显示0-999整数
void Lcd_smgUp3_SetPNum(uint16_t Number); // 显示P码
void Drive_Lcd_AllIcon_init(void);

#endif
