#include "u8g2_stm32.h"

// 声明外部SPI句柄，这个句柄是在 main.c 中由CubeIDE自动生成的
extern SPI_HandleTypeDef hspi2;

// 定义全局u8g2对象
u8g2_t u8g2;

// 硬件适配回调函数
uint8_t u8g2_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            // 初始化GPIO，CubeMX已经做过了，这里可以留空
            break;
        case U8X8_MSG_DELAY_MILLI:
            // 毫秒延时
            HAL_Delay(arg_int);
            break;
        case U8X8_MSG_GPIO_DC:
            // 控制DC引脚
            HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
            break;
        case U8X8_MSG_GPIO_RESET:
            // 控制RES引脚
            HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, arg_int);
            break;
        // CS引脚我们是直接接地的，所以不需要控制
        // case U8X8_MSG_GPIO_CS:
        //     HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, arg_int);
        //     break;
        default:
            return 0; // 未处理的消息
    }
    return 1;
}

// SPI通信回调函数
uint8_t u8g2_byte_spi_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_BYTE_SEND:
            // 通过SPI发送数据
            HAL_SPI_Transmit(&hspi2, (uint8_t *)arg_ptr, arg_int, 100);
            break;
        case U8X8_MSG_BYTE_INIT:
            // SPI初始化，CubeMX已经做过了，这里可以留空
            break;
        case U8X8_MSG_BYTE_SET_DC:
            // 控制DC引脚
            HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
            break;
        default:
            return 0; // 未处理的消息
    }
    return 1;
}

// U8g2 初始化函数
void u8g2_Init(u8g2_t *u8g2) {
    // 根据你的OLED控制器选择合适的初始化函数
    // SH1107是常见的128x128 OLED控制器，如果不是，请查找你的OLED模块资料
    // u8g2_Setup_sh1107_i2c_128x128_f(u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_stm32); // I2C 示例
    // 替换为下面这行
    u8g2_Setup_sh1107_seeed_128x128_f(u8g2, U8G2_R0, u8g2_byte_spi_stm32, u8g2_gpio_and_delay_stm32); // 4线SPI

    // 初始化U8g2
    u8g2_InitDisplay(u8g2);
    // 唤醒显示器
    u8g2_SetPowerSave(u8g2, 0);
}