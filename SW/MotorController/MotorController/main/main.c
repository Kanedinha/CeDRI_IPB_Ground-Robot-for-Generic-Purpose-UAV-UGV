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

#define DEBUG

#define ENCODER_A GPIO_NUM_25
#define ENCODER_B GPIO_NUM_26
#define MOTOR_A1 GPIO_NUM_16
#define MOTOR_A2 GPIO_NUM_27

#define ENCODER_STOP 0
#define CLOCKWISE 1
#define COUNTER_CLOCKWISE 2
#define ENCODER_ERROR 3

#define ESP_INTR_FLAG_DEFAULT 0

#ifdef DEBUG
long long int Timer1 = 0;
long long int Timer2 = 0;
long long int diff = 0;
#endif

const char *TAG = "CeDRI";

uint8_t sig_a = 0, sig_b = 0, sig_ant_a = 0, sig_ant_b = 0;
uint8_t dir_rot = 0;
uint16_t angle = 0;

const uint8_t enc_table[16] = {ENCODER_STOP, COUNTER_CLOCKWISE, CLOCKWISE, ENCODER_ERROR,
                               CLOCKWISE, ENCODER_STOP, ENCODER_ERROR, COUNTER_CLOCKWISE,
                               COUNTER_CLOCKWISE, ENCODER_ERROR, ENCODER_STOP, CLOCKWISE,
                               ENCODER_ERROR, CLOCKWISE, COUNTER_CLOCKWISE, ENCODER_ERROR};

TaskHandle_t Motor_Direction_Handle = NULL;
TaskHandle_t Encoder_Read_Handle = NULL;

// QueueHandle_t encoderDataQueue = NULL;

void IRAM_ATTR encoder_isr_handler(void *arg)
{
    sig_a = gpio_get_level(ENCODER_A);
    sig_b = gpio_get_level(ENCODER_B);
    if ((sig_a != sig_ant_a) || (sig_b != sig_ant_b))
    {
        dir_rot = (sig_a << 3) | (sig_b << 2) | (sig_ant_a << 1) | (sig_ant_b);
        sig_ant_a = sig_a;
        sig_ant_b = sig_b;
        // ESP_LOGI(TAG, "Enc A: %d - Enc B: %d - wise: %d", sig_a, sig_b, enc_table[dir_rot]);
    }
    else{
        
    }
}

void Motor_Direction(void *args)
{
    uint8_t dir = 0;

    while (1)
    {
        ESP_LOGI(TAG, "Enc A: %d - Enc B: %d - wise: %d", sig_a, sig_b, enc_table[dir_rot]);
        gpio_set_level(MOTOR_A1, dir);
        gpio_set_level(MOTOR_A2, !dir);
        dir = !dir;
        vTaskDelay(pdMS_TO_TICKS(10));

        gpio_set_level(MOTOR_A1, 0);
        gpio_set_level(MOTOR_A2, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Encoder_Read(void *args)
{

    while (1)
    {
        Timer1 = esp_timer_get_time();
        sig_a = gpio_get_level(ENCODER_A);
        sig_b = gpio_get_level(ENCODER_B);

        ets_delay_us(20);
        Timer2 = esp_timer_get_time();
        diff = Timer2 - Timer1;
        ESP_LOGI(TAG, "Sample period: %lld μs", diff);
    }
}

void app_main(void)
{
    // Disable Watchdog interrupt
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = (1ULL << ENCODER_A) | (1ULL << ENCODER_B);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // gpio_install_isr_service(0);
    // gpio_isr_handler_add(ENCODER_A, encoder_isr_handler, NULL);
    // gpio_isr_handler_add(ENCODER_B, encoder_isr_handler, NULL);

    xTaskCreate(Motor_Direction, "MotorDirection", 4096, NULL, 10, &Motor_Direction_Handle);
    xTaskCreate(Encoder_Read, "Encoder_Read", 4096, NULL, 20, &Encoder_Read_Handle);

    while (1)
    {
        // Leia encoder_count para determinar a posição do motor
        vTaskDelay(pdMS_TO_TICKS(100)); // Aguarde um curto período entre as leituras
    }
}
