
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define ENCODER_A 25
#define ENCODER_B 26

void app_main(void)
{
    gpio_pad_select_gpio(ENCODER_A);
    gpio_set_direction(ENCODER_A, GPIO_MODE_INPUT);

    gpio_pad_select_gpio(ENCODER_B);
    gpio_set_direction(ENCODER_B, GPIO_MODE_INPUT);

    uint8_t sig_a = 0, sig_b = 0;
    uint8_t kp, ki, kd = 0;

    // encoder table
    // actual - last
    // 0 0 0 0 |  0 
    // 0 0 0 1 | -1
    // 0 0 1 0 |  1
    // 0 0 1 1 |  !
    // 0 1 0 0 |  1
    // 0 1 0 1 |  0
    // 0 1 1 0 |  !
    // 0 1 1 1 | -1
    // 1 0 0 0 | -1
    // 1 0 0 1 |  !
    // 1 0 1 0 |  0
    // 1 0 1 1 |  1
    // 1 1 0 0 |  !
    // 1 1 0 1 |  1
    // 1 1 1 0 | -1
    // 1 1 1 1 |  0

    while (1)
    {
        sig_a = gpio_get_level(ENCODER_A);
        sig_b = gpio_get_level(ENCODER_B);
        printf("Enc A: %d  -  Enc B: %d\n", sig_a, sig_b);
        vTaskDelay(50);
    }
}
