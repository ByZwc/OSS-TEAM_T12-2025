#ifndef __APP_BUZMUSIC_H__
#define __APP_BUZMUSIC_H__

#define HZL1 262 - 1
#define HZL2 294 - 1
#define HZL3 330 - 1
#define HZL4 350 - 1
#define HZL5 392 - 1
#define HZL6 440 - 1
#define HZL7 494 - 1

#define HZM1 524 - 1
#define HZM2 588 - 1
#define HZM3 660 - 1
#define HZM4 700 - 1
#define HZM5 784 - 1
#define HZM6 880 - 1
#define HZM7 988 - 1

#define HZH_L1 1048 - 1
#define HZH_L2 1176 - 1
#define HZH_L3 1320 - 1
#define HZH_L4 1480 - 1
#define HZH_L5 1640 - 1
#define HZH_L6 1760 - 1
#define HZH_L7 1976 - 1

#define HZH_M1 2093 - 1
#define HZH_M2 2349 - 1
#define HZH_M3 2637 - 1
#define HZH_M4 2794 - 1
#define HZH_M5 3136 - 1
#define HZH_M6 3520 - 1
#define HZH_M7 3951 - 1

#define HZH_H1 4186 - 1
#define HZH_H2 4699 - 1
#define HZH_H3 5274 - 1
#define HZH_H4 5587 - 1
#define HZH_H5 6272 - 1
#define HZH_H6 7040 - 1
#define HZH_H7 7902 - 1

#define S 0 // 停止音

#define BUZ_ON 1
#define BUZ_OFF 0

#define BUZ_MUSIC_OFF 0
#define BUZ_DOROMI 1
#define BUZ_MIRODO 2
#define BUZ_DOROMIFASOLASI 3
#define BUZ_SILASOFAMIRODO 4
#define BUZ_DINGDONG 5
#define BUZ_DONGDING 6
#define BUZ_DINGDONGDIANGDANG 7

#define BUZ_ALARM 14
#define BUZ_1S 15
#define BUZ_20MS 16

#define BUZ_BEAT_TIME 23       // 音符节拍时间，单位10ms
#define BUZ_BEAT_TIME_CLOSE 25 // 音符节拍时间，单位10ms

#define BUZ_FREQ_CHANGE_ON 1
#define BUZ_FREQ_CHANGE_OFF 0

#define BUZ_MODE_MUSIC 1 // 无源蜂鸣器
#define BUZ_MODE_BASIC 0 // 有源蜂鸣器

void set_buzHz(uint16_t hz);

extern void app_Buz_Task(void);
extern void touchBuzzerSpeak_music(void);
void touchBuzzerSpeak_alarm(void);
void Drive_Buz_OnOff(uint8_t BuzmusicNum, uint8_t ChangeFreqCmd, uint8_t BuzMode);
#endif
