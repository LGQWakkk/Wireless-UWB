#include "utils.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
// ADC BATTERY
uint16_t adc_buffer[1];
#define BATTERY_ADC_SCALE (2.0f) // 测量电压转换为实际电池电压的系数
// 获取电池电压
float get_battery_voltage(void)
{
    return ((float)adc_buffer[0] * 3.3f / 4095.0f * BATTERY_ADC_SCALE);
}
// 使能ADC
void adc_enable(void)
{
    __IO uint32_t backup_setting_adc_dma_transfer = 0U;
    if (LL_ADC_IsEnabled(ADC1) == 0){ // 配置时要求ADC处于关闭状态
        LL_ADC_EnableInternalRegulator(ADC1);/* Enable ADC internal voltage regulator */
        LL_mDelay(1);
        /* Disable ADC DMA transfer request during calibration */
        backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
        LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);
        LL_ADC_StartCalibration(ADC1);/* Run ADC self calibration */
        while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);
        /* Restore ADC DMA transfer request after calibration */
        LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);
        LL_mDelay(1);
        LL_ADC_Enable(ADC1);
        while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0); // 等待变为Active
    }
}
// ADC must be enabled without conversion on going on group regular, without ADC disable command on going. 
bool adc_start(void)
{   /* Start ADC group regular conversion */
    if ((LL_ADC_IsEnabled(ADC1) == 1)               &&
        (LL_ADC_IsDisableOngoing(ADC1) == 0)        &&
        (LL_ADC_REG_IsConversionOngoing(ADC1) == 0)){
        LL_ADC_REG_StartConversion(ADC1);
        return true;
    }else{
        return false;
    }
}
// 配置ADC DMA传输方式
// 注意ADC的中断优先级要高于DMA的中断优先级
void configure_adc_dma(uint32_t bufferaddr, uint32_t bufferlen)
{
    LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_1, LL_DMAMUX_REQ_ADC1);
    // 设置DMA传输源地址和目标地址
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
                LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                bufferaddr, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, bufferlen);  // 设置DMA传输大小
    // LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);   // 使能DMA传输完成中断
    // LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);   // 使能DMA传输半满中断
    // LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);   // 使能DMA传输错误中断
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1); /* Enable the DMA transfer */
}
// ADC 相关事件回调函数
void adc_dma_transfer_complete_callback(void){}
void adc_dma_transfer_half_callback(void){}
void adc_dma_transfer_error_callback(void){}
void adc_overrun_callback(void){}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
volatile bool system_2khz_update = false;
volatile uint32_t system_2khz_tick = 0;
void system_2khz_timer_start(void)
{
    LL_TIM_ClearFlag_UPDATE(TIM1);
    LL_TIM_EnableIT_UPDATE(TIM1);
    LL_TIM_EnableCounter(TIM1);
}
uint32_t get_system_2khz_tick(void)
{
    return system_2khz_tick;
}
void system_2khz_timer_callback(void)
{
    system_2khz_update = true;
    system_2khz_tick ++;
}
uint32_t millis(void)
{
    return (system_2khz_tick / 2);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t log_buffer[128];
void slog(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf((char*)log_buffer, sizeof(log_buffer), fmt, args);
    va_end(args);
    debug_uart_tx(log_buffer, strlen((char*)log_buffer));
}
void slog_dma(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf((char*)log_buffer, sizeof(log_buffer), fmt, args);
    va_end(args);
    debug_uart_tx_dma((uint32_t)log_buffer, strlen((char*)log_buffer));
}
void debug_uart_tx(uint8_t *buffer, uint32_t len)
{
    uint32_t index = 0;
    uint8_t *p = buffer;
    for (index = 0; index < len; index++){
        while (!LL_USART_IsActiveFlag_TXE(USART1));
        LL_USART_TransmitData8(USART1, *p++);
        }
    while(!LL_USART_IsActiveFlag_TC(USART1));
}
void debug_uart_tx_dma(uint32_t buffer_addr, uint32_t len)
{
    debug_uart_configure_tx_dma(buffer_addr, len);
    debug_uart_tx_dma_enable();
}
void debug_uart_tx_dma_cplt_callback(void)
{
    // Nothing to do here
}
void debug_uart_tx_dma_enable(void)
{
    LL_USART_EnableDMAReq_TX(USART1);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}
void debug_uart_tx_dma_disable(void)
{
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}
void debug_uart_configure_tx_dma(uint32_t buffer_addr, uint32_t len)
{
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, (uint32_t)buffer_addr, LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, len);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);
}
void debug_uart_rx_callback(void)
{

}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
// UART2 MK8000 串口接收回调函数
void mk8000_uart_callback(void)
{
    uint8_t data = LL_USART_ReceiveData8(USART2);

}
// 启动UART2 MK8000 串口接收
void mk8000_uart_start(void)
{
    LL_USART_ClearFlag_ORE(USART2);
    LL_USART_EnableIT_RXNE(USART2);
    // LL_USART_EnableIT_ERROR(USART2);
}
// MK8000 串口轮询发送
void mk8000_uart_tx(uint8_t *buffer, uint32_t len)
{
    uint32_t index = 0;
    uint8_t *p = buffer;
    for (index = 0; index < len; index++){
        while (!LL_USART_IsActiveFlag_TXE(USART2));
        LL_USART_TransmitData8(USART2, *p++);
        }
    while(!LL_USART_IsActiveFlag_TC(USART2));
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
