#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "system.h"
#include "main.h"
#include "gpio.h"
#include "dma.h"

// LED CONTROL
#define LED1_OFF()   HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_ON()    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED2_OFF()   HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define LED2_ON()    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)

float get_battery_voltage(void);
void adc_enable(void);
bool adc_start(void);
void configure_adc_dma(uint32_t bufferaddr, uint32_t bufferlen);
void adc_dma_transfer_complete_callback(void);
void adc_dma_transfer_half_callback(void);
void adc_dma_transfer_error_callback(void);
void adc_overrun_callback(void);
extern uint16_t adc_buffer[1];

void system_2khz_timer_start(void);
uint32_t get_system_2khz_tick(void);
void system_2khz_timer_callback(void);
uint32_t millis(void);

void slog(const char *fmt, ...);
void slog_dma(const char *fmt, ...);
void debug_uart_tx(uint8_t *buffer, uint32_t len);
void debug_uart_tx_dma(uint32_t buffer_addr, uint32_t len);
void debug_uart_tx_dma_cplt_callback(void);
void debug_uart_tx_dma_enable(void);
void debug_uart_tx_dma_disable(void);
void debug_uart_configure_tx_dma(uint32_t buffer_addr, uint32_t len);
void debug_uart_rx_callback(void);

void mk8000_uart_callback(void);
void mk8000_uart_start(void);
void mk8000_uart_tx(uint8_t *buffer, uint32_t len);
