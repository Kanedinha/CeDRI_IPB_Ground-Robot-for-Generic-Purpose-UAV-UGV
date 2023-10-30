/*
    CeDRI Project - Grounded Robot with Two Dimensional Platform Self
                    Stabilizer for UAV-UGV coordenation.

    Author: Emerson Kazuyoshi Kaneda
    e-mail: emerson.k.kaneda@gmail.com
            a61103@alunos.ipb.pt

    acording with datasheet of the motor we have 64 counts per revolution
    ON MOTOR SHAFT, and we have a reduction of 70:1, thus we need 4480 pulse
    counts to have a full revolution on the final axis of the motor.

    encoder table

    Truth table for encoder state
     0 -> stop
     1 -> clockwise
    -1 -> counter clockwise
     ! -> error

    actual - last (sig_a | sig_b | sig_ant_a | sig_ant_b)
    0 0 0 0 |  0
    0 0 0 1 | -1
    0 0 1 0 |  1
    0 0 1 1 |  !
    0 1 0 0 |  1
    0 1 0 1 |  0
    0 1 1 0 |  !
    0 1 1 1 | -1
    1 0 0 0 | -1
    1 0 0 1 |  !
    1 0 1 0 |  0
    1 0 1 1 |  1
    1 1 0 0 |  !
    1 1 0 1 |  1
    1 1 1 0 | -1
    1 1 1 1 |  0

*/

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <rom/ets_sys.h>.
#include "esp_log.h"
#include "soc/rtc_wdt.h"

// #define DEBUG

#define ENCODER_A GPIO_NUM_25
#define ENCODER_B GPIO_NUM_17
#define MOTOR_A1 GPIO_NUM_16
#define MOTOR_A2 GPIO_NUM_27

#define ENCODER_STOP 0
#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 2
#define ENCODER_ERROR 3

#define ESP_INTR_FLAG_DEFAULT 0

const char *TAG = "CeDRI";

const uint8_t enc_table[16] = {ENCODER_STOP, COUNTER_CLOCKWISE, CLOCKWISE, ENCODER_ERROR,
                               CLOCKWISE, ENCODER_STOP, ENCODER_ERROR, COUNTER_CLOCKWISE,
                               COUNTER_CLOCKWISE, ENCODER_ERROR, ENCODER_STOP, CLOCKWISE,
                               ENCODER_ERROR, CLOCKWISE, COUNTER_CLOCKWISE, ENCODER_ERROR};

TaskHandle_t Motor_Direction_Handle = NULL;
TaskHandle_t Encoder_Read_Handle = NULL;

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR Encoder_Read_interrupt(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

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
    bool sig_a = 0, sig_b = 0, sig_ant_a = 0, sig_ant_b = 0;
    uint8_t dir_rot = 0;
    uint16_t angle = 0;
    uint32_t io_num;

    while (1)
    {

#ifdef DEBUG
        long long int Timer1 = esp_timer_get_time();
        ets_delay_us(15);
        long long int Timer2 = esp_timer_get_time();
        long long int diff = Timer2 - Timer1;
        ESP_LOGI(TAG, "Sample period: %lld μs", diff);
#endif

        sig_a = gpio_get_level(ENCODER_A);
        sig_b = gpio_get_level(ENCODER_B);

        if ((sig_a != sig_ant_a) || (sig_b != sig_ant_b))
        {
            dir_rot = (sig_a << 3) | (sig_b << 2) | (sig_ant_a << 1) | (sig_ant_b);
            ESP_LOGI(TAG, "Enc A: %d - Enc B: %d - wise: %d", sig_a, sig_b, enc_table[dir_rot]);
        }

        sig_ant_a = sig_a;
        sig_ant_b = sig_b;
    }
}

void app_main(void)
{
    // Disable Watchdog interrupt
    rtc_wdt_protect_off();
    rtc_wdt_disable();

    esp_err_t err = 0;

    err = gpio_set_direction(ENCODER_A, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ENCODER_A, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(ENCODER_A, GPIO_INTR_ANYEDGE);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "error in I/O configuration");
    }

    err = gpio_set_direction(ENCODER_B, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ENCODER_B, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(ENCODER_B, GPIO_INTR_ANYEDGE);

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

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(ENCODER_A, Encoder_Read_interrupt, (void *)ENCODER_A);
    gpio_isr_handler_add(ENCODER_B, Encoder_Read_interrupt, (void *)ENCODER_B);

    xTaskCreate(Motor_Direction, "MotorDirection", 4096, NULL, 10, &Motor_Direction_Handle);
    xTaskCreate(Encoder_Read, "Encoder_Read", 4096, NULL, 10, &Encoder_Read_Handle);

    while (1)
    {
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}
