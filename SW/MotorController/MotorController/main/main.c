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
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <esp32/rom/ets_sys.h>.
#include "esp_log.h"
#include "soc/rtc_wdt.h"
#include "esp_timer.h"

#define DEBUG

#define ENCODER_A GPIO_NUM_25
#define ENCODER_B GPIO_NUM_26
#define MOTOR_A1 GPIO_NUM_16
#define MOTOR_A2 GPIO_NUM_27

#define COUNTER_CLOCKWISE -1
#define CLOCKWISE 1
#define ENCODER_STOP 0
#define ENCODER_ERROR 0

#define ESP_INTR_FLAG_DEFAULT 0

#ifdef DEBUG
long long int Timer1 = 0;
long long int Timer2 = 0;
long long int diff = 0;
#endif

const char *TAG = "CeDRI";

uint8_t sig_a = 0, sig_b = 0, sig_ant_a = 0, sig_ant_b = 0;
uint8_t dir_rot = 0;
int16_t pulses = 0;
float angle = 0;

const float ang_per_pulse = 0.1607;

const int8_t enc_table[16] = {ENCODER_STOP, COUNTER_CLOCKWISE, CLOCKWISE, ENCODER_ERROR,
                              CLOCKWISE, ENCODER_STOP, ENCODER_ERROR, COUNTER_CLOCKWISE,
                              COUNTER_CLOCKWISE, ENCODER_ERROR, ENCODER_STOP, CLOCKWISE,
                              ENCODER_ERROR, CLOCKWISE, COUNTER_CLOCKWISE, ENCODER_ERROR};

// QueueHandle_t encoderDataQueue = NULL;
void timer_isr(void *arg)
{
    gpio_get_level(ENCODER_A);
    gpio_get_level(ENCODER_B);

    dir_rot = (sig_a << 3) | (sig_b << 2) | (sig_ant_a << 1) | (sig_ant_b);

    sig_ant_a = sig_a;
    sig_ant_b = sig_b;

    pulses += enc_table[dir_rot];

    ESP_LOGI(TAG, "Pulses: %d - dir: %d", pulses, dir_rot);
}

void app_main(void)
{
    // Disable Watchdog interrupt
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << ENCODER_A) | (1ULL << ENCODER_B);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << MOTOR_A1) | (1ULL << MOTOR_A2);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io_conf);

    sig_ant_a = gpio_get_level(ENCODER_A);
    sig_ant_b = gpio_get_level(ENCODER_B);

    esp_timer_create_args_t timer_config = {
        .callback = &timer_isr,
        .arg = NULL,
        .name = "timer"};

    esp_timer_handle_t timer;
    esp_timer_create(&timer_config, &timer);
    esp_timer_start_periodic(timer, 80); // 80 microssegundos

    while (1)
    {

        gpio_set_level(MOTOR_A1, 1);
    }
}
