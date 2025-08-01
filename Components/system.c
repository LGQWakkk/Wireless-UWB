// 20250801 Wakkk
#include "system.h"
#include "utils.h"
#include "si24r1.h"
#include "mk8000.h"

mk8000_frame_t frame;
bool frame_ready = false;
extern uint16_t adc_buffer[1];
float battery_voltage = 0.0;

void system_init(void)
{
    slog("System Init\r\n");
    LED1_OFF();
    LED2_OFF();

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

    slog("MK8000 UWB Init\r\n");
    mk8000_uart_start(); // 开启中断接收
    mk8000_slave_init(0x0003); // 从机初始化

    system_2khz_timer_start(); // 启动系统定时器
}

void system_loop(void)
{
    battery_voltage = get_battery_voltage();
    // slog("Battery Voltage: %.2fV\r\n", battery_voltage);
    if(battery_voltage < 3.70f){
        LED2_ON();
    }else{
        LED2_OFF();
    }
    if(frame_ready){
        LED1_TOGGLE();
        frame_ready = false;
        // slog("frame ready time: %d\r\n", frame.timestamp);
        // slog("distance: %.2fm\r\n", frame.distance);
        // slog("rssi: %ddBm\r\n", frame.rssi);
        // slog("send addr: 0x%04X\r\n", frame.send_addr);
    }
    HAL_Delay(10);
}
