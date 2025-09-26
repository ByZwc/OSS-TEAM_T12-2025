#include "main.h"

#define PID_PCMD_DIFF_THRESHOLD 15.0f
// P系数补偿
static float32_t app_pid_PCmd(uint16_t TarTemp, float32_t CurTemp)
{
    float32_t diff = (float32_t)TarTemp - CurTemp;
    float32_t addCoef = 0.0f;

    if (diff <= 0.0f)
    {
        addCoef = 0.0f;
    }
    else
    {
        addCoef = AllStatus_S.pid_s.pid_pCoef * (diff - PID_PCMD_DIFF_THRESHOLD) / PID_PCMD_DIFF_THRESHOLD;
    }

    return addCoef;
}

#define PID_ISET_MIN_TEMP 100
#define PID_ISET_MAX_TEMP 450
#define PID_ISET_MIN_COEF 0.05f
#define PID_ISET_MAX_COEF 3.0f
// 自适应I系数
static float32_t app_pid_iSetRange(uint16_t TarTemp)
{
    // TarTemp: PID_ISET_MIN_TEMP -> PID_ISET_MIN_COEF, PID_ISET_MAX_TEMP -> PID_ISET_MAX_COEF, linear mapping
    if (TarTemp <= PID_ISET_MIN_TEMP)
        return PID_ISET_MIN_COEF;
    if (TarTemp >= PID_ISET_MAX_TEMP)
        return PID_ISET_MAX_COEF;
    // Linear interpolation
    return PID_ISET_MIN_COEF + (PID_ISET_MAX_COEF - PID_ISET_MIN_COEF) * (TarTemp - PID_ISET_MIN_TEMP) / (float)(PID_ISET_MAX_TEMP - PID_ISET_MIN_TEMP);
}

// 积分引入引出
static void app_pid_iCmd(uint16_t TarTemp, float32_t CurTemp)
{
    float32_t diff = fabsf(CurTemp - TarTemp);

    switch (AllStatus_S.SolderingModelNumber)
    {
    case SOLDERING_MODEL_T115:
        Drive_MosSwitch210_PWMOut(); // 开启115PWM输出
        break;
    case SOLDERING_MODEL_T210:
        Drive_MosSwitch210_PWMOut(); // 开启210PWM输出
        break;
    case SOLDERING_MODEL_T245:
        Drive_MosSwitch245_PWMOut(); // 开启245PWM输出
        break;
    }

    if (CurTemp > (TarTemp + AllStatus_S.pid_s.pid_iItemQuitTemp))
    {
        AllStatus_S.pid_s.pid_iItem = 0.0f;
        AllStatus_S.pid_s.pid_iCoef = 0.0f;
        AllStatus_S.pid_s.pid_iItemCmd = 0.0f;
    }
    else
    {
        if (CurTemp > (TarTemp - AllStatus_S.pid_s.pid_iItemJoinTemp) && (CurTemp < TarTemp))
        {
            AllStatus_S.pid_s.pid_iCoef = app_pid_iSetRange(TarTemp);
            AllStatus_S.pid_s.pid_iItemCmd = 1.0f;
        }
    }

    app_pidOutCmd();
}

/* #define MAX_POWER_MIN_TEMP 100
#define MAX_POWER_MAX_TEMP 450
#define MAX_POWER_MIN_VALUE 1000
#define MAX_POWER_MAX_VALUE 10000

static uint16_t app_maxPowerControl(uint16_t TarTemp)
{
    // TarTemp: MAX_POWER_MIN_TEMP -> MAX_POWER_MIN_VALUE, MAX_POWER_MAX_TEMP -> MAX_POWER_MAX_VALUE, linear mapping
    if (TarTemp <= MAX_POWER_MIN_TEMP)
        return MAX_POWER_MIN_VALUE;
    if (TarTemp >= MAX_POWER_MAX_TEMP)
        return MAX_POWER_MAX_VALUE;
    // Linear interpolation
    return MAX_POWER_MIN_VALUE + (uint16_t)(((TarTemp - MAX_POWER_MIN_TEMP) * (MAX_POWER_MAX_VALUE - MAX_POWER_MIN_VALUE)) / (MAX_POWER_MAX_TEMP - MAX_POWER_MIN_TEMP));
} */

void app_pidControl(uint16_t TarTemp, float32_t CurTemp)
{
    float32_t difValue;

    app_pid_iCmd(TarTemp, CurTemp);
    float32_t diff = app_pid_PCmd(TarTemp, CurTemp);

    if (AllStatus_S.pid_s.outCmd)
    {
        difValue = (float32_t)TarTemp - CurTemp;
        AllStatus_S.pid_s.pid_pItem = (AllStatus_S.pid_s.pid_pCoef + diff) * difValue;
        AllStatus_S.pid_s.pid_iItem += AllStatus_S.pid_s.pid_iCoef * difValue * AllStatus_S.pid_s.pid_iItemCmd;
        AllStatus_S.pid_s.pid_dItem = AllStatus_S.pid_s.pid_dCoef * (difValue - AllStatus_S.pid_s.pid_lastDif);
        AllStatus_S.pid_s.pid_lastDif = difValue;

        if (AllStatus_S.pid_s.pid_iItem > AllStatus_S.pid_s.pid_integration_max)
            AllStatus_S.pid_s.pid_iItem = AllStatus_S.pid_s.pid_integration_max;
        /* if (AllStatus_S.pid_s.pid_iItem < (-AllStatus_S.pid_s.pid_integration_max))
            AllStatus_S.pid_s.pid_iItem = -AllStatus_S.pid_s.pid_integration_max; */
        if (AllStatus_S.pid_s.pid_iItem < 0.0f)
            AllStatus_S.pid_s.pid_iItem = 0.0f;

        AllStatus_S.pid_s.pid_out = AllStatus_S.pid_s.pid_pItem + AllStatus_S.pid_s.pid_iItem + AllStatus_S.pid_s.pid_dItem;

        switch (AllStatus_S.SolderingModelNumber)
        {
        case SOLDERING_MODEL_T115:
            AllStatus_S.pid_s.pid_outMax = T115_MAX_PID_O;
            break;
        case SOLDERING_MODEL_T210:
            AllStatus_S.pid_s.pid_outMax = T210_MAX_PID_O;
            break;
        case SOLDERING_MODEL_T245:
            // AllStatus_S.pid_s.pid_outMax = T245_MAX_PID_O + (AllStatus_S.flashSave_s.calibration_temp * T245_POWER_COMPENSATION_STEP);
            AllStatus_S.pid_s.pid_outMax = T12_MAX_PID_O;
            break;
        }
        if (AllStatus_S.pid_s.pid_out < 0)
            AllStatus_S.pid_s.pid_out = 0;
        if (AllStatus_S.pid_s.pid_out > AllStatus_S.pid_s.pid_outMax)
            AllStatus_S.pid_s.pid_out = AllStatus_S.pid_s.pid_outMax;

        if ((CurTemp < (TarTemp - AllStatus_S.pid_s.diffTempOutMaxPWM)) && (TarTemp > 150)) // 温度过低，强制输出最大功率
            AllStatus_S.pid_s.pid_out = AllStatus_S.pid_s.pid_outMax;

        Drive_MosSwitch_SetDuty(AllStatus_S.pid_s.pid_out);
    }
}

void app_pidOutCmd(void)
{
    if (AllStatus_S.SolderingState > SOLDERING_STATE_OK)
    {
        AllStatus_S.pid_s.outCmd = 0;
        AllStatus_S.pid_s.pid_out = 0.0f;
        AllStatus_S.pid_s.pid_iItem = 0.0f;
        AllStatus_S.pid_s.pid_iItemCmd = 0.0f;
        TIM3->CCR2 = 0;
    }
    else
    {
        AllStatus_S.pid_s.outCmd = 1;
    }
}
