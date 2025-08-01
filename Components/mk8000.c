#include "mk8000.h"
#include "utils.h"
// 20250713 Wakkk
#define CMD_BUFFER_SIZE 128
char cmd_buffer[CMD_BUFFER_SIZE];  // 发送指令缓冲区

// MK8000串口发送数据
void mk8000_uart_send(char *buffer, uint16_t len)
{
    mk8000_uart_tx((uint8_t *)buffer, len);
}

// 主机模式初始化
// 注意指令之间延时一定要足够 否则会配置失败
void mk8000_master_init(void)
{
    mk8000_set_mode(MODE_CONFIG); // 进入配置模式
    HAL_Delay(100);
    mk8000_set_role(ROLE_MASTER); // 设置为主机模式
    HAL_Delay(100);
    mk8000_software_reset(); // 设置软件复位以使得配置生效
    HAL_Delay(100);

    mk8000_set_mode(MODE_CONFIG); // 进入配置模式
    HAL_Delay(100);
    mk8000_set_tx_power(POWER_LEVEL_4); // 设置为最大发射功率
    HAL_Delay(100);
    mk8000_set_pid(0); // 设置网络PID为0
    HAL_Delay(100);
    mk8000_set_period(50); // 设置测距周期为50ms
    HAL_Delay(100);
    mk8000_set_host_addr(0x0000); // 设置本机(主机)地址为0x0000
    HAL_Delay(100);
    mk8000_set_slave_addr(0, 0x0001); // 设置从机1地址
    HAL_Delay(100);
    mk8000_set_slave_addr(1, 0x0002); // 设置从机2地址
    HAL_Delay(100);
    mk8000_set_slave_addr(2, 0x0003); // 设置从机3地址
    HAL_Delay(100);

    // 查询所有参数以确认
    mk8000_read_all_param();
    HAL_Delay(100);

    mk8000_software_reset(); // 设置软件复位以使得配置生效 之后会自动进入测量模式
    HAL_Delay(100);
}

// 从机模式初始化 传入参数为本从机地址
// 注意设置主从模式之后需要立即复位 复位之前设置其它参数无效
// ALL PARAM:
// AT+ROLE=0 (SLAVE)
// AT+PWR=4
// AT+PID=0
// AT+PERIOD=5
// AT+UART=115200
// AT+LPWR=0
// AT+MADDR=0001 本机地址
// AT+SADDR0=0000 主机地址
// AT+SADDR1=0002
// AT+SADDR2=0003
void mk8000_slave_init(uint16_t slave_addr)
{
    mk8000_set_mode(MODE_CONFIG); // 进入配置模式
    HAL_Delay(100);
    mk8000_set_role(ROLE_SLAVE); // 设置为从机模式
    HAL_Delay(100);
    mk8000_software_reset(); // 设置软件复位以使得配置生效
    HAL_Delay(100);

    mk8000_set_mode(MODE_CONFIG); // 进入配置模式
    HAL_Delay(100);
    mk8000_set_tx_power(POWER_LEVEL_4); // 设置为最大发射功率
    HAL_Delay(100);
    mk8000_set_pid(0); // 设置网络PID为0
    HAL_Delay(100);
    mk8000_set_period(50); // 设置测距周期为50ms
    HAL_Delay(100);
    mk8000_set_host_addr(slave_addr); // 设置本机(从机)地址
    HAL_Delay(100);
    mk8000_set_slave_addr(0, 0x0000); // 设置主机地址
    HAL_Delay(100);

    // 查询所有参数以确认
    mk8000_read_all_param();
    HAL_Delay(100);

    mk8000_software_reset(); // 设置软件复位以使得配置生效 之后会自动进入测量模式
    HAL_Delay(100);
}

// 解析帧数据 返回值表示是否成功解析
// buffer: 要解析的数据缓冲区
// frame: 解析后的数据帧输出
// 注意对frame的valid位进行检验
bool mk8000_parse_frame(uint8_t *buffer, mk8000_frame_t *frame)
{
    // check frame
    if(buffer[0]!= FRAME_HEAD || buffer[1]!= 0X05 || buffer[7]!= FRAME_TAIL){
        frame->valid = false; //数据包无效
        return false; //error
    }
    frame->valid = true;
    frame->timestamp = millis(); //记录数据包接收时刻系统时间戳
    frame->send_addr = (uint16_t)(((uint16_t)buffer[3]<<8)|((uint16_t)buffer[2]));
    frame->raw_distance = (uint16_t)(((uint16_t)buffer[5]<<8)|((uint16_t)buffer[4]));
    frame->raw_rssi = buffer[6];

    frame->distance = (float)frame->raw_distance/100.0f;//距离单位m
    frame->rssi = (int)frame->raw_rssi-256; //信号强度单位dBm(-256~-1)
    return true; //success
}

// 设置模式: 配置模式or测距模式
// 注意事项: 立即生效 掉电不保存 上电之后默认是测距模式
uint8_t mk8000_set_mode(mk8000_mode_t mode)
{
    switch (mode)
    {
        case MODE_CONFIG:
            snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+MODE=0\r\n");
            break;
        case MODE_MEASURE:
            snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+MODE=1\r\n");
            break;
        default:
        return 1; //error
    }
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

// 设置为主机还是从机
// 注意事项: 模块复位之后生效 支持掉电保存
// 主机模式(基站模式) 从机模式(标签模式)
uint8_t mk8000_set_role(mk8000_role_t role)
{
    switch (role)
    {
        case ROLE_MASTER:
            snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+ROLE=1\r\n");
            break;
        case ROLE_SLAVE:
            snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+ROLE=0\r\n");
            break;  
        default:
            return 1; //error
    }
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

// 设置模块串口波特率
// 注意事项: 模块复位之后生效 支持掉电保存
uint8_t mk8000_set_baudrate(mk8000_baudrate_t baudrate)
{
    switch (baudrate)
    {
        case BAUDRATE_9600:
            snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+UART=9600\r\n");
            break;
        case BAUDRATE_19200:
            snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+UART=19200\r\n");
            break;
        case BAUDRATE_38400:
            snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+UART=38400\r\n");
            break;
        case BAUDRATE_57600:
            snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+UART=57600\r\n");
            break;
        case BAUDRATE_115200:
            snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+UART=115200\r\n");
            break;
        case BAUDRATE_230400:
            snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+UART=230400\r\n");
            break;
        default:
            return 1; //error
    }
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

// 设置模块发射功率等级
// 注意事项: 立即生效 支持掉电保存
// "AT+PWR=<power>\r\n" <power>: 0-4
uint8_t mk8000_set_tx_power(mk8000_tx_power_t tx_power)
{
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+PWR=%d\r\n", tx_power);
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

// 设置网络ID
// 注意事项: 立即生效 支持掉电保存
// "AT+PID=<ID>\r\n" <ID>:0-255
uint8_t mk8000_set_pid(uint8_t pid)
{
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+PID=%d\r\n", pid);
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

// 设置低功耗模式
// "AT+LPWR=<mode>\r\n" <mode>: 0-1
uint8_t mk8000_set_low_power(mk8000_low_power_t mode)
{
    switch (mode)
    {
        case LOW_POWER_DISABLE:
            snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+LPWR=0\r\n");//关闭低功耗模式
            break;
        case LOW_POWER_ENABLE:
            snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+LPWR=1\r\n");//开启低功耗模式
            break;
        default:
            return 1; //error
    }
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

// 设置主机地址(模块为从机模式时 主机地址为此从机模块的本机地址)
// 16进制地址 0x0000-0xFFFF (经过实际测试好像不能设置为FFFF)
// "AT+MADDR=<addr>\r\n"
// 注意事项: 模块复位之后生效 支持掉电保存
uint8_t mk8000_set_host_addr(uint16_t addr)
{
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+MADDR=%04X\r\n", addr);
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

// 查询主机地址
uint8_t mk8000_read_host_addr(void)
{
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+MADDR=?\r\n");
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

// 设置从机地址(模块为从机模式时 从机0地址为需要挂载的主机模块地址)
// 设置为哪个从机: 范围0-2
// "AT+SADDR0=<addr>\r\n"
// "AT+SADDR1=<addr>\r\n"
// "AT+SADDR2=<addr>\r\n"
// 注意事项: 模块复位之后生效 支持掉电保存
uint8_t mk8000_set_slave_addr(uint8_t slave, uint16_t addr)
{
    if(slave > 2){
        return 1; //error
    }
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+SADDR%d=%04X\r\n", slave, addr);
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

// 查询从机地址
uint8_t mk8000_read_slave_addr(uint8_t slave)
{
    if(slave > 2){
        return 1; //error
    }
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+SADDR%d=?\r\n", slave);
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

// 设置测距周期(主机和从机的测距周期一致才能通信)
// period: 测距周期ms 范围50-1000ms 频率1Hz-20Hz 步进10ms
// "AT+PERIOD=<period>\r\n" <period>: 5-100
// 注意事项: 模块复位之后生效 支持掉电保存
uint8_t mk8000_set_period(uint16_t period)
{
    if(period < 50 || period > 1000){
        return 1; //invalid value
    }
    uint8_t _period = (period/10); //5~100
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+PERIOD=%d\r\n", _period);
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

// 恢复出厂设置
uint8_t mk8000_reset_param(void)
{
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+DEFT\r\n");
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

// 模块软件复位
uint8_t mk8000_software_reset(void)
{
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+RST\r\n");
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

// 查询所有参数
uint8_t mk8000_read_all_param(void)
{
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+ALL\r\n");
    mk8000_uart_send(cmd_buffer, strlen(cmd_buffer));
    return 0; //success
}

#define MK8000_UART_RX_BUFFER_SIZE (2*FRAME_TOTAL_LEN)
static uint8_t buf[MK8000_UART_RX_BUFFER_SIZE];
static uint8_t buf_p;

// MK8000 串口单字节解析函数
// return:
// 0: success
// 1: wait for next byte
// 2: len error
// 3: overflow error
// 4: parse error
uint8_t mk8000_parse_char(uint8_t data, mk8000_frame_t *_frame)
{
    if(data == FRAME_HEAD){
        buf_p = 0;
        buf[buf_p] = data;
        buf_p++;
        return 1; // wait for next byte
    }else if(data == FRAME_TAIL){
        if(buf_p == (FRAME_TOTAL_LEN-1)){ //success
            buf[buf_p] = data;
            buf_p = 0;//reset
            if(mk8000_parse_frame(buf, _frame)){
                return 0; // success
            }else{
                return 4; // parse error
            }
        }else{ // len error
            buf_p = 0;//reset
            return 2; // len error
        }
    }else{
        if(buf_p < (FRAME_TOTAL_LEN-1)){
            buf[buf_p] = data;
            buf_p++;
            return 1; // wait for next byte
        }else if(buf_p > (FRAME_TOTAL_LEN-1)){ // overflow
            buf_p = 0;//reset
            return 3; // overflow error
        }
    }
}
