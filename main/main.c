#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#define I2C_MASTER_SCL_IO 22        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define LCD_ADDR 0x27
#define RS (1 << 0)
#define RW (1 << 1)
#define E  (1 << 2)
#define BL (1 << 3)

esp_err_t i2c_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
void delay_ms(int ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}
void write_data(uint8_t data)
{
    uint8_t error = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    error |= i2c_master_start(cmd);
    error |= i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    error |= i2c_master_write_byte(cmd, data | BL, true);
    error |= i2c_master_stop(cmd);
    error |= i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (error != ESP_OK)
    {
        printf("::::warning::::\n");
    }
}
void send_nibble(bool is_char, uint8_t nibble)
{
    uint8_t data = nibble << 4;
    if (is_char)
    {
        data |= RS;
    }
    else
    {
        data &= ~RS;
    }
    write_data(data | E);
    delay_ms(1);
    write_data(data & ~E);
}
void send_byte(bool is_char, uint8_t byte)
{
    send_nibble(is_char, byte >> 4);
    send_nibble(is_char, byte & 0xF);
}
void send_cmd(uint8_t cmd)
{
    send_byte(false, cmd);
}
void send_char(uint8_t ch)
{
    send_byte(true, ch);
}
void i2c_lcd_init()
{
    delay_ms(15);
    send_cmd(0x30);
    send_cmd(0x30);
    send_cmd(0x32);
    send_cmd(0x2C);
    send_cmd(0xC);
    send_cmd(0x6);
    send_cmd(0x1);
}
void print_string(char *ptr)
{
    while (*ptr != '\0')
    {
        send_char(*ptr);
        ptr++;
    }
}
void gotoxy(int x, int y)
{
    if (x >= 0 && x <= 15)
    {
        if (y == 0)
        {
            send_cmd(0x80 + x);
        }
        else if (y == 1)
        {
            send_cmd(0xC0 + x);
        }
    }
}
void clear()
{
    send_cmd(0x1);
}
void app_main(void)
{
    i2c_init();
    i2c_lcd_init();
    while (1)
    {
        gotoxy(0, 0);
        print_string("HOLA");
        gotoxy(0, 1);
        print_string("MUNDO");
        delay_ms(1000);
        
    }
}
