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
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
// #include "main.h"
#include "MoveMean.h"
#include "IIR_filter.h"
// #include "ADXL345.cpp"
#include <ESP32Servo.h>

// motor defines
#define ENCODER_A 25    // fio branco enc
#define ENCODER_B 26    // fio amarelo enc
#define MOTOR_A1_PIN 16 // R PWM driver
#define MOTOR_A2_PIN 27 // L PWM driver
#define MOTOR_ASPEED_PIN 14
#define MOTOR_A 0
#define MOTOR_B 1
#define MOTOR_STEP 18
#define MOTOR_DIR 19

// Servo motor
uint8_t angleX = 0;
uint8_t angleY = 0;
uint16_t timeCount = 0;
bool stepState = HIGH;
bool motorDirection = 0;

// directions
#define CCW -1 // counter clockwise
#define CW 1   // clockwise
#define STOP 0
#define EE 0 // encoder erro

// ADXL Defines
#define DEVICE 0x53
#define SAMPLE_LENGHT 6

// encoder vars
#define MAX_ANGLE 180
#define MIN_ANGLE 0
uint8_t encoder_signal = 0;
int16_t pulses = 0;
bool sig_a = 0;
bool sig_b = 0;
bool sig_ant_a = 0;
bool sig_ant_b = 0;
float angle = 0;
const float ang_per_pulse = (2 * PI) / 5770;

// compensation h
const uint8_t h_cw = 15;
const uint8_t h_ccw = 13;

// PID FF Gains
float Kff = 27;
float Kp = 5;
float Ki = 1;
float Kd = 6;

// PID Vars
float previous_error = 0;
float integral = 0;
float derivative = 0;
float set_point = PI / 2;
float erro = 0;
float output = 0;
float real_output = 0;

// counter
#define INTERVALO_DE_AMOSTRAGEM 1000
unsigned long tempoAnterior = 0;
unsigned long contadorAmostras = 0;
uint8_t i = 0;

// timers
Ticker timer;
Ticker timer2;
Ticker timer3;

// ADXL345 object and vars
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
float a_x = 0;
float a_y = 0;
float a_z = 0;
float xFiltrado = 0;
float yFiltrado = 0;
float zFiltrado = 0;
float pitchFiltrado = 0;
float rollFiltrado = 0;
float roll = 0;
float pitch = 0;
float rollOffset = 0;
float pitchOffset = 0;

// encoder table
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

void step_isr()
{
  digitalWrite(MOTOR_DIR, motorDirection);
  digitalWrite(MOTOR_STEP, stepState);
  stepState = !stepState;
}

void setup()
{

  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_STEP, OUTPUT);
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
  timer3.attach(0.01, step_isr);

  delay(1000);

  if (!accel.begin())
  {
    Serial.println("O sensor ADXL345 não foi detectado");
    while (1)
      ;
  }
  accel.setRange(ADXL345_RANGE_16_G);
  // backward(set_point * Kff);
}

void loop()
{

  if (Serial.available() > 0)
  {
    // set_point = Serial.parseFloat();
    String dadosRecebidos = Serial.readStringUntil('\n');

    set_point = dadosRecebidos.substring(0, dadosRecebidos.indexOf(';')).toFloat();
    if (set_point > MAX_ANGLE)
    {
      set_point = MAX_ANGLE;
    }
    if (set_point < MIN_ANGLE)
    {
      set_point = MIN_ANGLE;
    }

    dadosRecebidos.remove(0, dadosRecebidos.indexOf(';') + 1);

    angleX = dadosRecebidos.substring(0, dadosRecebidos.indexOf(',')).toFloat();
    dadosRecebidos.remove(0, dadosRecebidos.indexOf(';') + 1);

    angleY = dadosRecebidos.toFloat();

    motorDirection = !motorDirection;

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

  sensors_event_t event;
  accel.getEvent(&event);

  // --------------------------------------------------------------------------------------- //
  // Roll and Pitch angles, accelerations

  a_x = event.acceleration.x;
  a_y = event.acceleration.y;
  a_z = event.acceleration.z;

  // roll = atan2(a_y, a_z) * 180.0 / PI;
  // roll = roll + rollOffset;
  // pitch = atan2(-a_x, sqrt(a_y * a_y + a_z * a_z)) * 180.0 / PI;
  // pitch = pitch + pitchOffset;
  float yaw = 0;

  // --------------------------------------------------------------------------------------- //
  // FIR

  // --------------------------------------------------------------------------------------- //
  // IIR

  // xFiltrado = filtroIIR(a_x);
  // yFiltrado = filtroIIR(a_y);
  // zFiltrado = filtroIIR(a_z);

  // xFiltrado = IIR(a_x, xFiltrado, amortecimento);
  // yFiltrado = IIR(a_y, yFiltrado, amortecimento);
  // zFiltrado = IIR(a_z, zFiltrado, amortecimento);

  // --------------------------------------------------------------------------------------- //
  // Move Mean

  xFiltrado = MoveMean(a_x, xFiltrado);
  yFiltrado = MoveMean(a_y, yFiltrado);
  zFiltrado = MoveMean(a_z, zFiltrado);

  // --------------------------------------------------------------------------------------- //
  // print values

  roll = atan2(yFiltrado, zFiltrado) * 180.0 / PI;
  roll = roll + rollOffset;
  pitch = atan2(-xFiltrado, sqrt(yFiltrado * yFiltrado + zFiltrado * zFiltrado)) * 180.0 / PI;
  pitch = pitch + pitchOffset;

  Serial.print(stepState);
  Serial.print(";");
  Serial.print(roll);
  Serial.print(";");
  Serial.println(pitch);

  // calcular taxa de amostragem
  // unsigned long tempoAtual = millis();
  //   if (tempoAtual - tempoAnterior >= INTERVALO_DE_AMOSTRAGEM) {
  //       float taxaAmostragem = contadorAmostras / (float)(tempoAtual - tempoAnterior) * 1000;

  //       Serial.print("Taxa de Amostragem: ");
  //       Serial.print(taxaAmostragem);
  //       Serial.println(" amostras por segundo");

  //       contadorAmostras = 0;
  //       tempoAnterior = tempoAtual;
  //   }
  //   contadorAmostras++;
}
