#include "system.h"
#include "utils.h"
#include "si24r1.h"
// 20250717 Wakkk

extern uint16_t adc_buffer[1];
float battery_voltage = 0.0;

void system_init(void)
{
    slog("System Init\r\n");
    LED1_ON();
    LED2_ON();

    slog("ADC Init\r\n");
    adc_enable();
    configure_adc_dma((uint32_t)adc_buffer, 1);
    bool ret = adc_start();
    if (ret){slog("ADC Started Success.\r\n");
    }else{slog("ADC Start Failed.\r\n");}

    slog("SI24R1 Init\r\n");
    if(si24r1_check_connection()){
        slog("SI24R1 Detected\r\n");
        si24r1_init(MODE_TX); // 默认进入发射模式
    }else{
        slog("SI24R1 Detect Failed\r\n");
    }


}

void system_loop(void)
{
    battery_voltage = get_battery_voltage();
    slog("Battery Voltage: %.2fV\r\n", battery_voltage);
    HAL_Delay(100);
}

