#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define LED 13
void app_main(void)
{
    
    gpio_reset_pin(LED);
    gpio_pad_select_gpio(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    srand(time(NULL));
    while(1)
    {
        gpio_set_level(LED, rand() % 2);
        vTaskDelay((rand() % 1000) / portTICK_PERIOD_MS);
    }

}