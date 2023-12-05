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


pulses 5760 -> 360º

10 ms de timer pro PID

counter clock wise h_ccw = 36 (37 roda começa a rodar sempre)
clock wise h_cw = 51 (52 começa a rodar sempre)
*/

#include <Arduino.h>
#include "esp_timer.h"
#include <Ticker.h>

#define ENCODER_A 25    // fio branco enc
#define ENCODER_B 26    // fio amarelo enc
#define MOTOR_A1_PIN 16 // R PWM driver
#define MOTOR_A2_PIN 27 // L PWM driver
#define MOTOR_ASPEED_PIN 14
#define MOTOR_A 0
#define MOTOR_B 1

#define CCW -1 // counter clockwise
#define CW 1   // clockwise
#define STOP 0
#define EE 0 // encoder erro

uint8_t encoder_signal = 0;
int16_t pulses = 0;
bool sig_a = 0;
bool sig_b = 0;
bool sig_ant_a = 0;
bool sig_ant_b = 0;
float angle = 0;
const float ang_per_pulse = (2 * PI) / 5770;

const uint8_t h_cw = 15;
const uint8_t h_ccw = 13;

float Kff = 27;
float Kp = 5;
float Ki = 1;
float Kd = 6;

float previous_error = 0;
float integral = 0;
float derivative = 0;
float set_point = PI / 2;
float erro = 0;
float output = 0;
float real_output = 0;

uint8_t i = 0;

Ticker timer;
Ticker timer2;

const int8_t enc_table[16] = {STOP, CCW, CW, EE,
                              CW, STOP, EE, CCW,
                              CCW, EE, STOP, CW,
                              EE, CW, CCW, STOP};

void forward(uint8_t output)
{
  analogWrite(MOTOR_A1_PIN, 0);

  analogWrite(MOTOR_A2_PIN, output);
}

void backward(uint8_t output)
{
  analogWrite(MOTOR_A2_PIN, 0);
  analogWrite(MOTOR_A1_PIN, output);
}

void enc_timer_isr()
{
  sig_a = digitalRead(ENCODER_A);
  sig_b = digitalRead(ENCODER_B);

  encoder_signal = (sig_a << 3) | (sig_b << 2) | (sig_ant_a << 1) | (sig_ant_b);

  sig_ant_a = sig_a;
  sig_ant_b = sig_b;

  pulses += enc_table[encoder_signal];
}

void PID_timer_isr()
{
  angle = pulses * ang_per_pulse;
  erro = set_point - angle;
  integral += erro; // saturar depois
  derivative = erro - previous_error;
  output = Kp * erro + Ki * integral + Kd * derivative + Kff * set_point;
  // output = Kp * erro + Kd * derivative;

  if (integral > 255)
  {
    integral = 255;
  }
  if (integral < -255)
  {
    integral = -255;
  }

  // direção do motor
  if (output > 0)
  {
    real_output = output + h_ccw;
    // verificação de overflow
    if (real_output > 255)
    {
      real_output = 255;
    }
    backward(real_output);
  }
  else
  {
    real_output = abs(output) + h_cw;
    // verificação de overflow
    if (real_output > 255)
    {
      real_output = 255;
    }
    forward(real_output);
  }
}

void setup()
{
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_A2_PIN, OUTPUT);
  ledcSetup(0, 10000, 8);
  ledcSetup(1, 10000, 8);
  ledcAttachPin(MOTOR_A1_PIN, 0);
  ledcAttachPin(MOTOR_A2_PIN, 1);

  Serial.begin(115200);

  sig_a = digitalRead(ENCODER_A);
  sig_b = digitalRead(ENCODER_B);
  sig_ant_a = digitalRead(ENCODER_A);
  sig_ant_b = digitalRead(ENCODER_B);

  timer.attach(0.00008, enc_timer_isr);
  timer2.attach(0.01, PID_timer_isr);
  delay(1000);

  // backward(set_point * Kff);
}

void loop()
{

  if (Serial.available() > 0)
  {
    // Aguarda a chegada de dados pela porta serial
    char receivedChar = Serial.read();

    // Verifica se o caractere recebido é um número (48 a 57 na tabela ASCII)
    if (receivedChar >= '0' && receivedChar <= '9')
    {
      // Converte o caractere para um valor numérico e atualiza a variável correspondente
      switch (receivedChar)
      {
      case '1':
        Kp = Serial.parseInt();
        break;
      case '2':
        Ki = Serial.parseInt();
        break;
      case '3':
        Kd = Serial.parseInt();
        break;
      case '4':
        set_point = Serial.parseInt();
        break;
      default:
        break;
      }
    }

    Serial.print("Variáveis atualizadas: ");
    Serial.print("Kp: ");
    Serial.print(Kp);
    Serial.print(", Ki: ");
    Serial.print(Ki);
    Serial.print(", Kd: ");
    Serial.print(Kd);
    Serial.print(", set point: ");
    Serial.println(set_point);
  }
}
