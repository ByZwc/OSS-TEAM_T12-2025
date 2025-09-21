#include "main.h"

static void touchBuzzerSpeak_doromi(void);
static void touchBuzzerSpeak_mirodo(void);
static void touchBuzzerSpeak_1S(void);
// 设置蜂鸣器频率
static void set_buzHz(uint16_t hz)
{
    // 24MHz主频，分频6后为4MHz
    // PWM周期 = 4MHz / hz
    // ARR = (4000000 / hz) - 1
    // CCR1 = ARR / 2
    if (hz == 0)
        return;
    uint32_t arr = (4000000UL / hz) - 1;
    TIM16->ARR = arr;
    TIM16->CCR1 = arr / 2;
    AllStatus_S.Buz_ccr = TIM16->CCR1;
}

uint16_t Castle_in_the_Sky_music[] =
    {
        S,
        S,
        HZH_M6,
        HZH_M7,
        HZH_H1,
        HZH_M7,
        HZH_H1,
        HZH_H3,
        HZH_M7,
        S,
        S,
        S,
        HZH_M3,
        HZH_M3,
        HZH_M6,
        HZH_M5,
        HZH_M6,
        HZH_H1,
        HZH_M5,
        S,
        S,
        S,
        HZH_M3,
        HZH_M3,
        HZH_M4,
        HZH_M3,
        HZH_M4,
        HZH_H1,
        HZH_M3,
        S,
        S,
        S,
        S,
        HZH_H1,
        HZH_H1,
        HZH_H1,
        HZH_M7,
        HZH_M4,
        HZH_M4,
        HZH_M7,
        HZH_M7,
        S,
        S,
        S,
        S,
        HZH_M6,
        HZH_M7,
        HZH_H1,
        HZH_M7,
        HZH_H1,
        HZH_H3,
        HZH_M7,
        S,
        S,
        S,
        S,
        HZH_M3,
        HZH_M3,
        HZH_M6,
        HZH_M5,
        HZH_M6,
        HZH_H1,
        HZH_M5,
        S,
        S,
        S,
        S,
        HZH_M2,
        HZH_M3,
        HZH_M4,
        HZH_H1,
        HZH_M7,
        HZH_H1,
        HZH_H2,
        HZH_H2,
        HZH_H3,
        HZH_H1,
        S,
        S,
        S,
        S,
        HZH_H1,
        HZH_M7,
        HZH_M6,
        HZH_M6,
        HZH_M7,
        HZH_M5,
        HZH_M6,
        S,
        S,
        S,
};

static void set_beep(uint16_t f)
{
    if (f == 0)
    {
#if (USE_BUZ_TYPE == 0)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
#else
        // 如果f=0则不发出声音
        TIM16->CCR1 = 0;
        AllStatus_S.Buz_ccr = 0;
#endif
    }
    else
    { // 发出指定频率的声音
        set_buzHz(f);
    }
}

static void touchBuzzerSpeak_Freq(void)
{
#if (USE_BUZ_TYPE == 0)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
#else
    set_buzHz(AllStatus_S.BuzMusicFreq);
#endif
}

static void touchBuzzerSpeak_L1(void)
{
#if (USE_BUZ_TYPE == 0)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
#else
    set_buzHz(HZH_M7);
#endif
}

static void touchBuzzer_overtone(uint32_t Buz_ccr, uint16_t temp, uint16_t temp_max)
{
    if (Buz_ccr == 0)
        return;
    // 占空比随temp递减
    TIM16->CCR1 = (uint32_t)((Buz_ccr * (temp_max - temp)) / temp_max);
}

#define DONGDING_NUM 3
uint16_t dongding[DONGDING_NUM] = {HZH_L3, HZH_M2};
static void touchBuzzerSpeak_dongding(void)
{
    static uint16_t i = 0, temp = 0;
    if (temp == 0)
    {
        if (i >= DONGDING_NUM)
        {
            i = 0;
            AllStatus_S.BuzmusicNum = BUZ_MUSIC_OFF;
            set_beep(0);
            return;
        }

        set_beep(dongding[i]); // 发出指定音调
        if (i < DONGDING_NUM)
            i++;
    }
    temp++;
    touchBuzzer_overtone(AllStatus_S.Buz_ccr, temp, BUZ_BEAT_TIME);
    if (temp == BUZ_BEAT_TIME)
    {
        set_beep(0);
    }

    if (temp >= BUZ_BEAT_TIME_CLOSE)
        temp = 0;
}

#define DINGDONG_NUM 2
uint16_t dingdong[DINGDONG_NUM] = {HZH_M2, HZH_L3};
static void touchBuzzerSpeak_dingdong(void)
{
    static uint16_t i = 0, temp = 0;
    if (temp == 0)
    {
        if (i >= DINGDONG_NUM)
        {
            i = 0;
            AllStatus_S.BuzmusicNum = BUZ_MUSIC_OFF;
            set_beep(0);
            return;
        }

        set_beep(dingdong[i]); // 发出指定音调
        if (i < DINGDONG_NUM)
            i++;
    }
    temp++;
    touchBuzzer_overtone(AllStatus_S.Buz_ccr, temp, BUZ_BEAT_TIME);
    if (temp == BUZ_BEAT_TIME)
    {
        set_beep(0);
    }

    if (temp >= BUZ_BEAT_TIME_CLOSE)
        temp = 0;
}

#define DINGDONGDIANGDANG_NUM 6
uint16_t dingdongdiangdang[DINGDONGDIANGDANG_NUM] = {S, S, HZH_M3, HZH_M1, HZH_M2, HZH_M5};
static void touchBuzzerSpeak_dingdongdiangdang(void)
{
    static uint16_t i = 0, temp = 0;
    if (temp == 0)
    {
        if (i >= DINGDONGDIANGDANG_NUM)
        {
            i = 0;
            AllStatus_S.BuzmusicNum = BUZ_MUSIC_OFF;
            set_beep(0);
            return;
        }

        set_beep(dingdongdiangdang[i]); // 发出指定音调
        if (i < DINGDONGDIANGDANG_NUM)
            i++;
    }
    temp++;
    touchBuzzer_overtone(AllStatus_S.Buz_ccr, temp, BUZ_BEAT_TIME);
    if (temp == BUZ_BEAT_TIME)
    {
        set_beep(0);
    }

    if (temp >= BUZ_BEAT_TIME_CLOSE)
        temp = 0;
}

#define DOROMI_NUM 3
uint16_t doromi[DOROMI_NUM] = {HZH_M1, HZH_M2, HZH_M3};
static void touchBuzzerSpeak_doromi(void)
{
    static uint16_t i = 0, temp = 0;
    if (temp == 0)
    {
        if (i >= DOROMI_NUM)
        {
            i = 0;
            AllStatus_S.BuzmusicNum = BUZ_MUSIC_OFF;
            set_beep(0);
            return;
        }

        set_beep(doromi[i]); // 发出指定音调
        if (i < DOROMI_NUM)
            i++;
    }
    temp++;
    touchBuzzer_overtone(AllStatus_S.Buz_ccr, temp, BUZ_BEAT_TIME);
    if (temp == BUZ_BEAT_TIME)
    {
        set_beep(0);
    }

    if (temp >= BUZ_BEAT_TIME_CLOSE)
        temp = 0;
}

#define MIRODO_NUM 3
uint16_t mirodo[MIRODO_NUM] = {HZH_M3, HZH_M2, HZH_M1};
static void touchBuzzerSpeak_mirodo(void)
{
    static uint16_t i = 0, temp = 0;
    if (temp == 0)
    {
        if (i >= MIRODO_NUM)
        {
            i = 0;
            AllStatus_S.BuzmusicNum = BUZ_MUSIC_OFF;
            set_beep(0);
            return;
        }
        set_beep(mirodo[i]); // 发出指定音调
        if (i < MIRODO_NUM)
            i++;
    }
    temp++;
    touchBuzzer_overtone(AllStatus_S.Buz_ccr, temp, BUZ_BEAT_TIME);
    if (temp == BUZ_BEAT_TIME)
    {
        set_beep(0);
    }

    if (temp >= BUZ_BEAT_TIME_CLOSE)
        temp = 0;
}

#define DOROMIFASOLASI_NUM 9
uint16_t doromifasolasi[DOROMIFASOLASI_NUM] = {S, S, HZH_M1, HZH_M2, HZH_M3, HZH_M4, HZH_M5, HZH_M6, HZH_M7};
static void touchBuzzerSpeak_doromifasolasi(void)
{
    static uint16_t i = 0, temp = 0;
    if (temp == 0)
    {
        if (i >= DOROMIFASOLASI_NUM)
        {
            i = 0;
            AllStatus_S.BuzmusicNum = BUZ_MUSIC_OFF;
            set_beep(0);
            return;
        }
        set_beep(doromifasolasi[i]); // 发出指定音调
        if (i < DOROMIFASOLASI_NUM)
            i++;
    }
    temp++;
    touchBuzzer_overtone(AllStatus_S.Buz_ccr, temp, BUZ_BEAT_TIME);
    if (temp == BUZ_BEAT_TIME)
    {
        set_beep(0);
    }

    if (temp >= BUZ_BEAT_TIME_CLOSE)
        temp = 0;
}

#define SILASOFAMIRODO_NUM 7
uint16_t silasofamirodo[SILASOFAMIRODO_NUM] = {HZH_M7, HZH_M6, HZH_M5, HZH_M4, HZH_M3, HZH_M2, HZH_M1};
static void touchBuzzerSpeak_silasofamirodo(void)
{
    static uint16_t i = 0, temp = 0;
    if (temp == 0)
    {
        if (i >= SILASOFAMIRODO_NUM)
        {
            i = 0;
            AllStatus_S.BuzmusicNum = BUZ_MUSIC_OFF;
            set_beep(0);
            return;
        }
        set_beep(silasofamirodo[i]); // 发出指定音调
        if (i < SILASOFAMIRODO_NUM)
            i++;
    }
    temp++;
    touchBuzzer_overtone(AllStatus_S.Buz_ccr, temp, BUZ_BEAT_TIME);
    if (temp == BUZ_BEAT_TIME)
    {
        set_beep(0);
    }

    if (temp >= BUZ_BEAT_TIME_CLOSE)
        temp = 0;
}

static void touchBuzzerSpeak_1S(void)
{
    static uint16_t temp = 0;
    temp++;
    if (temp == 1)
        touchBuzzerSpeak_L1();

    if (temp >= 80)
    {
        AllStatus_S.BuzmusicNum = BUZ_MUSIC_OFF;
        set_beep(0);
        temp = 0;
    }
}

static void touchBuzzerSpeak_20Ms(void)
{
    static uint16_t temp = 0;
    temp++;
    if (temp == 1)
        touchBuzzerSpeak_Freq();

    if (temp >= 5)
    {
        AllStatus_S.BuzmusicNum = BUZ_MUSIC_OFF;
        set_beep(0);
        temp = 0;
    }
}

static uint16_t get_buzzer_freq_by_temp(int16_t TarTemp, int16_t TarMin, int16_t TarMax)
{
    // 音调从低到高
    static uint16_t freq_table[] = {
        HZH_M1,
        HZH_M2,
        HZH_M3,
        HZH_M4,
        HZH_M5,
        HZH_M6,
        HZH_M7,
    };
    const uint8_t freq_num = sizeof(freq_table) / sizeof(freq_table[0]);

    if (TarTemp <= TarMin)
        return freq_table[0];
    if (TarTemp >= TarMax)
        return freq_table[freq_num - 1];

    // 线性映射温度到频率表索引
    uint32_t idx = ((TarTemp - TarMin) * (freq_num - 1)) / (TarMax - TarMin);
    return freq_table[idx];
}

static void touchBuzzer_set(uint8_t num)
{
    switch (num)
    {
    case BUZ_DOROMI:
        AllStatus_S.BuzmusicNum = BUZ_DOROMI;
        break;
    case BUZ_MIRODO:
        AllStatus_S.BuzmusicNum = BUZ_MIRODO;
        break;
    case BUZ_DOROMIFASOLASI:
        AllStatus_S.BuzmusicNum = BUZ_DOROMIFASOLASI;
        break;
    case BUZ_SILASOFAMIRODO:
        AllStatus_S.BuzmusicNum = BUZ_SILASOFAMIRODO;
        break;
    case BUZ_1S:
        AllStatus_S.BuzmusicNum = BUZ_1S;
        break;
    case BUZ_20MS:
        AllStatus_S.BuzmusicNum = BUZ_20MS;
        break;
    case BUZ_ALARM:
        AllStatus_S.BuzmusicNum = BUZ_ALARM;
        break;
    case BUZ_DINGDONG:
        AllStatus_S.BuzmusicNum = BUZ_DINGDONG;
        break;
    case BUZ_DONGDING:
        AllStatus_S.BuzmusicNum = BUZ_DONGDING;
        break;
    case BUZ_DINGDONGDIANGDANG:
        AllStatus_S.BuzmusicNum = BUZ_DINGDONGDIANGDANG;
        break;

    default:
        break;
    }
}

void Drive_Buz_OnOff(uint8_t BuzmusicNum, uint8_t ChangeFreqCmd, uint8_t BuzMode)
{
    if (AllStatus_S.flashSave_s.BuzOnOff && (AllStatus_S.BuzmusicNum == BUZ_MUSIC_OFF))
    {
        if (BuzMode == BUZ_MODE_BASIC)
        {
            if (BuzmusicNum != BUZ_1S)
                touchBuzzer_set(BUZ_20MS);
            else
                touchBuzzer_set(BUZ_1S);
            AllStatus_S.BuzMusicFreq = HZH_M7;
            return;
        }

        switch (BuzmusicNum)
        {
        case BUZ_DOROMI:
            touchBuzzer_set(BUZ_DOROMI);
            break;
        case BUZ_MIRODO:
            touchBuzzer_set(BUZ_MIRODO);
            break;
        case BUZ_DOROMIFASOLASI:
            touchBuzzer_set(BUZ_DOROMIFASOLASI);
            break;
        case BUZ_SILASOFAMIRODO:
            touchBuzzer_set(BUZ_SILASOFAMIRODO);
            break;
        case BUZ_1S:
            touchBuzzer_set(BUZ_1S);
            break;
        case BUZ_20MS:
            touchBuzzer_set(BUZ_20MS);
            if (ChangeFreqCmd)
                AllStatus_S.BuzMusicFreq = get_buzzer_freq_by_temp(AllStatus_S.flashSave_s.TarTemp, MIN_TAR_TEMP - 5, MAX_TAR_TEMP);
            else
            {
                if (!AllStatus_S.Seting.SetingPage)
                    AllStatus_S.BuzMusicFreq = HZH_M7;
                else
                {
                    switch (AllStatus_S.Seting.PNumber)
                    {
                    case SMG_P01:
                        AllStatus_S.BuzMusicFreq = HZH_M1;
                        break;
                    case SMG_P02:
                        AllStatus_S.BuzMusicFreq = HZH_M2;
                        break;
                    case SMG_P03:
                        AllStatus_S.BuzMusicFreq = HZH_M3;
                        break;
                    case SMG_P04:
                        AllStatus_S.BuzMusicFreq = HZH_M4;
                        break;
                    case SMG_P05:
                        AllStatus_S.BuzMusicFreq = HZH_M5;
                        break;
                    default:
                        break;
                    }
                }
            }
            break;
        case BUZ_ALARM:
            touchBuzzer_set(BUZ_ALARM);
            break;
        case BUZ_DINGDONG:
            touchBuzzer_set(BUZ_DINGDONG);
            break;
        case BUZ_DONGDING:
            touchBuzzer_set(BUZ_DONGDING);
            break;
        case BUZ_DINGDONGDIANGDANG:
            touchBuzzer_set(BUZ_DINGDONGDIANGDANG);
            break;

        default:
            break;
        }
    }
}

static void touchBuzzerSpeak_alarm(void)
{
    static uint16_t temp = 0;
    static uint8_t up = 1;
    static uint16_t hz = 400;

    // 防空警报音调在400Hz~1000Hz之间上下扫频
    if (up)
    {
        hz += 5;
        if (hz >= 1000)
            up = 0;
    }
    else
    {
        hz -= 5;
        if (hz <= 400)
            up = 1;
    }

    set_beep(hz);

    temp++;
    if (temp >= 200) // 持续一段时间后关闭
    {
        set_beep(0);
        temp = 0;
        hz = 400;
        up = 1;
        AllStatus_S.BuzmusicNum = BUZ_MUSIC_OFF;
    }
}

void app_Buz_Task(void)
{
    if (AllStatus_S.BuzmusicNum)
    {
        switch (AllStatus_S.BuzmusicNum)
        {
        case BUZ_DOROMI:
            touchBuzzerSpeak_doromi();
            break;
        case BUZ_MIRODO:
            touchBuzzerSpeak_mirodo();
            break;
        case BUZ_DOROMIFASOLASI:
            touchBuzzerSpeak_doromifasolasi();
            break;
        case BUZ_SILASOFAMIRODO:
            touchBuzzerSpeak_silasofamirodo();
            break;
        case BUZ_1S:
            touchBuzzerSpeak_1S();
            break;
        case BUZ_20MS:
            touchBuzzerSpeak_20Ms();
            break;
        case BUZ_ALARM:
            touchBuzzerSpeak_alarm();
            break;
        case BUZ_DINGDONG:
            touchBuzzerSpeak_dingdong();
            break;
        case BUZ_DONGDING:
            touchBuzzerSpeak_dongding();
            break;
        case BUZ_DINGDONGDIANGDANG:
            touchBuzzerSpeak_dingdongdiangdang();
            break;

        default:
            break;
        }
    }
}

void touchBuzzerSpeak_music(void)
{
    static uint16_t i = 0, temp = 0;

    if (temp == 0)
    {
        if (i >= (sizeof(Castle_in_the_Sky_music) / sizeof(Castle_in_the_Sky_music[0]) - 1))
        {
            i = 0;
            // buz_onOff(0);
            TIM16->CCR1 = 0;
            return; // 如果音乐播放完毕，则退出
        }

        if (i < sizeof(Castle_in_the_Sky_music) / sizeof(Castle_in_the_Sky_music[0]))
            i++;
        set_beep(Castle_in_the_Sky_music[i]); // 发出指定音调
    }
    temp++;
    touchBuzzer_overtone(AllStatus_S.Buz_ccr, temp, BUZ_BEAT_TIME);
    if (temp == BUZ_BEAT_TIME)
    {
        set_beep(0);
    }

    if (temp >= BUZ_BEAT_TIME_CLOSE)
        temp = 0;
}
