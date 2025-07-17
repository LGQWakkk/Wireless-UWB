#pragma once
// 20250713 Wakkk
// MK8000 UWB模块驱动
// 模块默认波特率为115200 8N1

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "usart.h"

#define MK8000_UART_HANDLE huart1

// 测距模式数据包格式
// [帧头0xF0] [有效数据长度0x05] [发送地址2Byte] [距离2Byte] [信号强度 1Byte] [帧尾0xAA]
#define FRAME_HEAD 0XF0
#define FRAME_LEN 0X05
#define FRAME_TAIL 0XAA
#define FRAME_TOTAL_LEN 0x08 //数据包总长度(缓冲区大小)

// 时间戳类型
typedef uint32_t time_ms_t;

// 测距数据格式定义
typedef struct{
    uint8_t valid;          //有效标志位
    time_ms_t timestamp;    //接收时刻系统时间戳 HAL TICK
    uint16_t send_addr;     //数据包发送地址
    uint16_t raw_distance;  //距离 单位cm 原始数据
    uint8_t raw_rssi;       //数据包信号强度 原始数据
    float distance;         //距离 单位m
    int rssi;               //数据包信号强度 单位dBm(-256~-1)
}mk8000_frame_t;

// MK8000工作模式
typedef enum{
    MODE_CONFIG,//AT指令配置模式
    MODE_MEASURE//正常测距模式
}mk8000_mode_t;

// 主机 or 从机
typedef enum{
    ROLE_MASTER,//基站
    ROLE_SLAVE//标签
}mk8000_role_t;

// 波特率
typedef enum{
    BAUDRATE_9600,
    BAUDRATE_19200,
    BAUDRATE_38400,
    BAUDRATE_57600,
    BAUDRATE_115200,
    BAUDRATE_230400,
}mk8000_baudrate_t;

// 发射功率等级
typedef enum{
    POWER_LEVEL_0,//-17.63dBm
    POWER_LEVEL_1,//-11.18dBm
    POWER_LEVEL_2,//-8.57dBm
    POWER_LEVEL_3,//-4.24dBm
    POWER_LEVEL_4 //0dBm
}mk8000_tx_power_t;

typedef enum{
    LOW_POWER_DISABLE,//关闭低功耗模式
    LOW_POWER_ENABLE  //开启低功耗模式
}mk8000_low_power_t;

// mk8000所有参数
typedef struct{
    mk8000_role_t role;             //主机 or 从机
    mk8000_tx_power_t tx_power;     //发射功率等级
    uint16_t pid;                   //网络ID /信道
    uint16_t period;                //测距周期(ms) 范围50-1000ms
    mk8000_baudrate_t baudrate;     //波特率
    mk8000_low_power_t low_power;   //低功耗模式
    uint16_t host_addr;             //主机地址
    uint16_t slave_addr[3];         //从机地址 0-2
}mk8000_param_t;

// Functions
void mk8000_uart_send(char *buffer, uint16_t len);
void mk8000_init(void);
bool mk8000_parse_frame(uint8_t *buffer, mk8000_frame_t *frame);
uint8_t mk8000_set_mode(mk8000_mode_t mode);
uint8_t mk8000_set_role(mk8000_role_t role);
uint8_t mk8000_set_baudrate(mk8000_baudrate_t baudrate);
uint8_t mk8000_set_tx_power(mk8000_tx_power_t tx_power);
uint8_t mk8000_set_pid(uint8_t pid);
uint8_t mk8000_set_low_power(mk8000_low_power_t mode);
uint8_t mk8000_set_period(uint16_t period);
uint8_t mk8000_set_host_addr(uint16_t addr);
uint8_t mk8000_set_slave_addr(uint8_t slave, uint16_t addr);
uint8_t mk8000_reset_param(void);       //恢复出厂设置
uint8_t mk8000_software_reset(void);    //软件复位
