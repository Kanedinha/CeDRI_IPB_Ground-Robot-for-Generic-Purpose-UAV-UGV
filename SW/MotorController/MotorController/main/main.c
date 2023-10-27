
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define ENCODER_A GPIO_NUM_25
#define ENCODER_B GPIO_NUM_17
#define MOTOR_A1 GPIO_NUM_16
#define MOTOR_A2 GPIO_NUM_27 

const char *TAG = "CeDRI";

TaskHandle_t Motor_Direction_Handle = NULL;
TaskHandle_t Encoder_Read_Handle = NULL;

void Motor_Direction(void *args)
{
    uint8_t dir = 0;

    while (1)
    {
        gpio_set_level(MOTOR_A1, dir);
        gpio_set_level(MOTOR_A2, !dir);
        dir = !dir;
        vTaskDelay(1000 / portTICK_RATE_MS);

        gpio_set_level(MOTOR_A1, 0);
        gpio_set_level(MOTOR_A2, 0);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

void Encoder_Read(void *args)
{
    uint8_t sig_a = 0, sig_b = 0;

    while (1)
    {
        sig_a = gpio_get_level(ENCODER_A);
        sig_b = gpio_get_level(ENCODER_B);
        printf("Enc A: %d  -  Enc B: %d\n", sig_a, sig_b);
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

void app_main(void)
{
    esp_err_t err = 0;

    err = gpio_set_direction(ENCODER_A, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ENCODER_A, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(GPIO_INTR_ANYEDGE);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "error in I/O configuration");
    }

    err = gpio_set_direction(ENCODER_B, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ENCODER_B, GPIO_PULLDOWN_ONLY);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "error in I/O configuration");
    }

    err = gpio_set_direction(MOTOR_A1, GPIO_MODE_OUTPUT);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "error in I/O configuration");
    }

    err = gpio_set_direction(MOTOR_A2, GPIO_MODE_OUTPUT);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "error in I/O configuration");
    }

    // acording with datasheet we have 64 counts per revolution
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

    xTaskCreate(Motor_Direction, "MotorDirection", 4096, NULL, 10, &Motor_Direction_Handle);
    xTaskCreate(Encoder_Read, "Encoder_Read", 4096, NULL, 10, &Encoder_Read_Handle);

    while (1)
    {
     vTaskDelay(100 / portTICK_RATE_MS);
    }
}
