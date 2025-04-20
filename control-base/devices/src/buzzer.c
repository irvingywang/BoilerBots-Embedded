#include "buzzer.h"

#define MAX_BUZZER_NUM (1)
Buzzzer_Instance_t g_buzzer;
PWM_Instance_t *g_buzzer_uart_ptr;
uint8_t g_buzzer_count = 0;

#ifdef STM32H723xx
#define BUZZER_TIMER_NUM (htim12)
#define BUZZER_TIMER_CHANNEL (TIM_CHANNEL_2)
#else
#define BUZZER_TIMER_NUM (htim4)
#define BUZZER_TIMER_CHANNEL (TIM_CHANNEL_3)
#endif

void Buzzer_Init()
{
    PWM_Config_t buzzer_config = {
        .htim = &BUZZER_TIMER_NUM,
        .channel = BUZZER_TIMER_CHANNEL,
        .dutyratio = 0,
        .period = 0.001,
    };
    g_buzzer_uart_ptr = PWM_Register(&buzzer_config);
}

void Buzzer_Set_Loudness(Buzzzer_Instance_t *buzzer, float loudness)
{
    buzzer->loudness = loudness;
    PWM_Set_Duty_Ratio(g_buzzer_uart_ptr, buzzer->loudness);
}

void Buzzer_Set_Note(Buzzzer_Instance_t *buzzer)
{
    PWM_Set_Frequency(g_buzzer_uart_ptr, buzzer->frequency);
    PWM_Set_Duty_Ratio(g_buzzer_uart_ptr, buzzer->loudness);
}

void Buzzer_Play_Melody(Melody_t melody)
{
    for (int i = 0; i < melody.note_num; i++)
    {
        g_buzzer.frequency = melody.notes[i][0];
        g_buzzer.loudness = melody.loudness;
        if (g_buzzer.frequency != 0) {
            Buzzer_Set_Note(&g_buzzer);
        }
        HAL_Delay((uint32_t)(melody.notes[i][1]));
        Buzzer_Set_Loudness(&g_buzzer, 0);
        HAL_Delay((uint32_t)(melody.notes[i][1] * 0.3));
    }
}