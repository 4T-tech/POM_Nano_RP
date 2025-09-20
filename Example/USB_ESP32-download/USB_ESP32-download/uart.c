#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "tusb.h"

/* ===修改RP2040与ESP的模式，1为通过rp2040的USB口给ESP下载固件，0为rp2040向esp32发送AT指令 === */
#define MODE_DOWNLOAD   1   // 1 = 下载模式，0 = AT模式
/* ======================================== */

/* ------------- UART 定义 ------------- */
#if MODE_DOWNLOAD
    #define UART_USE    uart1
    #define UART_TX_PIN 8
    #define UART_RX_PIN 9
#else
    #define UART_USE    uart0
    #define UART_TX_PIN 12
    #define UART_RX_PIN 13
#endif
#define BAUD_RATE   115200

/* ------------- ESP32 控制引脚 ------------- */
#define ESP_RST_PIN  23
#define ESP_BOOT_PIN 25

/* ------------- 中断转发 ------------- */
void on_uart_irq(void)
{
    while (uart_is_readable(UART_USE)) {
        uint8_t ch = uart_getc(UART_USE);
        tud_cdc_write_char(ch);
    }
    tud_cdc_write_flush();
}

/* ------------- CDC 接收 ------------- */
void tud_cdc_rx_cb(uint8_t itf)
{
    uint8_t buf[64];
    uint32_t len = tud_cdc_read(buf, sizeof(buf));
    if (len) {
        uart_write_blocking(UART_USE, buf, len);
    }
}

/* ------------- 主函数 ------------- */
int main(void)
{
    stdio_init_all();
    tusb_init();

    /* 配置 ESP32 控制引脚 */
    gpio_init(ESP_RST_PIN);
    gpio_init(ESP_BOOT_PIN);
    gpio_set_dir(ESP_RST_PIN, GPIO_OUT);
    gpio_set_dir(ESP_BOOT_PIN, GPIO_OUT);

    /* 根据宏控制 ESP32 进入对应模式  下载模式：EN↓ -> BOOT0↓ -> EN↑ AT模式：EN↓ -> BOOT0↑ -> EN↑*/
    gpio_put(ESP_RST_PIN, 0);
    gpio_put(ESP_BOOT_PIN, MODE_DOWNLOAD ? 0 : 1);
    sleep_ms(10);
    gpio_put(ESP_RST_PIN, 1);
    sleep_ms(100);

    /* 初始化 UART */
    uart_init(UART_USE, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(UART_USE, false);

    /* 配置中断 */
    int irq_num = (UART_USE == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(irq_num, on_uart_irq);
    irq_set_enabled(irq_num, true);
    uart_set_irq_enables(UART_USE, true, false);

    while (1) {
        tud_task();
    }
}