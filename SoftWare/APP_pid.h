#ifndef __APP_PID_H__
#define __APP_PID_H__



void app_pidControl(uint16_t TarTemp, float32_t CurTemp);
void app_pidOutCmd(void);
void app_pidOutMaxControl(void);


#endif


