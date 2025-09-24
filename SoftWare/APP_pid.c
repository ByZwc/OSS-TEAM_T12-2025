#include "main.h"

#define PID_PCMD_DIFF_THRESHOLD 15.0f // 超过此温差才增加P系数

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

    if (diff > AllStatus_S.pid_s.pid_iItemQuitTemp)
    {
        AllStatus_S.pid_s.pid_iItem = 0.0f;
        AllStatus_S.pid_s.pid_iItemCmd = 1.0f;
    }
    else
    {
        AllStatus_S.pid_s.pid_iItemCmd = 1.0f;
    }

    app_pidOutCmd();
}

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
        if (AllStatus_S.pid_s.pid_iItem < (-AllStatus_S.pid_s.pid_integration_max))
            AllStatus_S.pid_s.pid_iItem = -AllStatus_S.pid_s.pid_integration_max;

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

        if (CurTemp < (TarTemp - AllStatus_S.pid_s.diffTempOutMaxPWM))
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
