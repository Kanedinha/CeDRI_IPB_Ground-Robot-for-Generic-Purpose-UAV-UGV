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
#include <Adafruit_MPU6050.h>
#include "MoveMean.h"
#include "IIR_filter.h"
#include <ESP32Servo.h>

// motor defines
#define MOTOR_PITCH_STEP 13
#define MOTOR_PITCH_DIR 5
#define MOTOR_ROLL_STEP 14
#define MOTOR_ROLL_DIR 12

// Servo motor
uint16_t timeCount = 0;
bool PitchStepState = HIGH;
bool PitchMotorDirection = 0;
uint8_t PitchNumOfSteps = 0;
bool RollStepState = HIGH;
bool RollMotorDirection = 0;
uint8_t RollNumOfSteps = 0;

// ADXL Defines
#define DEVICE 0x53
#define SAMPLE_LENGHT 6

// encoder vars
#define MAX_ANGLE 30
#define MIN_ANGLE -30
const float ang_per_step = 1.8 / 32;

// PID FF Gains
float Kff = 27;
float Kp = 5;
float Ki = 1;
float Kd = 6;

// PID Vars
float previous_error = 0;
float integral = 0;
float derivative = 0;
float set_point_pitch = 0;
float set_point_roll = 0;
float PitchError = 0;
float RollError = 0;

// counter
#define INTERVALO_DE_AMOSTRAGEM 1000
unsigned long tempoAnterior = 0;
unsigned long contadorAmostras = 0;
uint8_t i = 0;

// timers
Ticker timer;
Ticker timer2;
Ticker timer3;

// MPU object and vars
Adafruit_MPU6050 mpu1;

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

void PID_timer_isr()
{
  // Pitch calc
  PitchError = set_point_pitch - pitch;

  if (PitchError > MAX_ANGLE)
  {
    PitchError = MAX_ANGLE;
  }
  else if (PitchError < MIN_ANGLE)
  {
    PitchError = MIN_ANGLE;
  }
  else if (abs(PitchError) < 3 * ang_per_step)
  {
    RollError = 0;
  }

  PitchNumOfSteps = uint8_t(abs(PitchError / ang_per_step));

  // Roll calc
  RollError = set_point_roll - roll;

  if (RollError > MAX_ANGLE)
  {
    RollError = MAX_ANGLE;
  }
  else if (RollError < MIN_ANGLE)
  {
    RollError = MIN_ANGLE;
  }
  else if (abs(RollError) < 3 * ang_per_step)
  {
    RollError = 0;
  }

  RollNumOfSteps = uint8_t(abs(RollError / ang_per_step));

  // Direction of each angle
  if (PitchError > 0)
  {
    PitchMotorDirection = 1;
  }
  else
  {
    PitchMotorDirection = 0;
  }

  if (RollError > 0)
  {
    RollMotorDirection = 0;
  }
  else
  {
    RollMotorDirection = 1;
  }
}

void step_isr()
{
  // Pitch Steps
  if (PitchNumOfSteps != 0)
  {
    digitalWrite(MOTOR_PITCH_DIR, PitchMotorDirection);
    digitalWrite(MOTOR_PITCH_STEP, PitchStepState);
    PitchStepState = !PitchStepState;
    if (PitchStepState)
    {
      PitchNumOfSteps--;
    }
  }

  // Roll Steps
  if (RollNumOfSteps != 0)
  {
    digitalWrite(MOTOR_ROLL_DIR, RollMotorDirection);
    digitalWrite(MOTOR_ROLL_STEP, RollStepState);
    RollStepState = !RollStepState;
    if (RollStepState)
    {
      RollNumOfSteps--;
    }
  }
}

void setup()
{

  Serial.begin(9600);

  pinMode(MOTOR_PITCH_DIR, OUTPUT);
  pinMode(MOTOR_PITCH_STEP, OUTPUT);
  pinMode(MOTOR_ROLL_DIR, OUTPUT);
  pinMode(MOTOR_ROLL_STEP, OUTPUT);

  timer.attach_ms(5, PID_timer_isr);
  timer2.attach_ms(0.002, step_isr);

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
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();
    int separatorIndex = inputString.indexOf(';');

    if (separatorIndex != -1) {
            String value1String = inputString.substring(0, separatorIndex);
            String value2String = inputString.substring(separatorIndex + 1);

            // Converte as strings para inteiros (ou floats, se necessário)
            set_point_pitch = value1String.toFloat();
            set_point_roll = value2String.toFloat();
    }

    if (set_point_pitch > MAX_ANGLE)
      set_point_pitch = MAX_ANGLE;
    else if (set_point_pitch < MIN_ANGLE)
      set_point_pitch = MIN_ANGLE;

    if (set_point_roll > MAX_ANGLE)
      set_point_roll = MAX_ANGLE;
    else if (set_point_roll < MIN_ANGLE)
      set_point_roll = MIN_ANGLE;
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

  Serial.print(pitch);
  Serial.print(";");
  Serial.print(roll);
  Serial.print(";");
  Serial.print(set_point_pitch);
  Serial.print(";");
  Serial.println(set_point_roll);

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
