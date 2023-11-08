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

#include <Arduino.h>
#include "esp_timer.h"
#include <Ticker.h>

#define ENCODER_A 25
#define ENCODER_B 26
#define MOTOR_A_PIN 14
#define MOTOR_B_PIN 27
#define MOTOR_A 0
#define MOTOR_B 1

#define COUNTER_CLOCKWISE -1
#define CLOCKWISE 1
#define ENCODER_STOP 0
#define ENCODER_ERROR 0

uint8_t dir_rot = 0;
int16_t pulses = 0;
bool sig_a = 0;
bool sig_b = 0;
bool sig_ant_a = 0;
bool sig_ant_b = 0;
float angle = 0;
const float ang_per_pulse = 1;

float Kp = 0.0001;
float Ki = 0.0001;
float Kd = 0.0001;

float previous_error = 0;
float integral = 0;
float derivative = 0;
float set_point = 500;

Ticker timer;

const int8_t enc_table[16] = {ENCODER_STOP, COUNTER_CLOCKWISE, CLOCKWISE, ENCODER_ERROR,
                              CLOCKWISE, ENCODER_STOP, ENCODER_ERROR, COUNTER_CLOCKWISE,
                              COUNTER_CLOCKWISE, ENCODER_ERROR, ENCODER_STOP, CLOCKWISE,
                              ENCODER_ERROR, CLOCKWISE, COUNTER_CLOCKWISE, ENCODER_ERROR};

void timer_isr()
{
  sig_a = digitalRead(ENCODER_A);
  sig_b = digitalRead(ENCODER_B);

  dir_rot = (sig_a << 3) | (sig_b << 2) | (sig_ant_a << 1) | (sig_ant_b);

  sig_ant_a = sig_a;
  sig_ant_b = sig_b;

  pulses += enc_table[dir_rot];
}

void setup()
{
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  // pinMode(MOTOR_A, OUTPUT);
  // pinMode(MOTOR_B, OUTPUT);
  ledcSetup(MOTOR_A, 5000, 8);
  ledcSetup(MOTOR_B, 5000, 8);
  ledcAttachPin(MOTOR_A, MOTOR_A);
  ledcAttachPin(MOTOR_B, MOTOR_B);

  Serial.begin(115200);

  timer.attach(0.00008, timer_isr);
}

void loop()
{
  // float ang = pulses;

  // float erro = set_point - ang;
  // integral += erro;
  // derivative = erro - previous_error;
  // int16_t output = 0;

  // if (Kp * erro + Ki * integral + Kd * derivative > 255)
  // {
  //   output = 255;
  // }
  // else if (Kp * erro + Ki * integral + Kd * derivative < -255)
  // {
  //   output = -255;
  // }
  // else
  // {
  //   output = Kp * erro + Ki * integral + Kd * derivative;
  // }
  // Serial.print(" pulses: ");
  // Serial.print(pulses);
  // Serial.print(" erro: ");
  // Serial.println(erro);

  // if (output > 0)
  // {

  //   ledcWrite(MOTOR_A, output);
  //   ledcWrite(MOTOR_B, 0);

  //   // sentido horario
  // }
  // else
  // {
  //   ledcWrite(MOTOR_A, 0);
  //   ledcWrite(MOTOR_B, abs(output));
  //   // sentido anti horario
  // }

  // previous_error = erro;
}
