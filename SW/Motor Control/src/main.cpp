/*
    CeDRI Project - TWO-DEGREE-OF-FREEDOM PLAFORM AS A TOOL FOR 
                    COOPERATION BETWEEN MOBILE ROBOTS AND UAVs.

    Author: Emerson Kazuyoshi Kaneda
    e-mail: emerson.k.kaneda@gmail.com
            a61103@alunos.ipb.pt

*/

#include <Arduino.h>
#include "esp_timer.h"
#include <Ticker.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include "MoveMean.h"
#include <HardwareSerial.h>

// motor defines
#define MOTOR_PITCH_STEP 13
#define MOTOR_PITCH_DIR 5
#define MOTOR_ROLL_STEP 14
#define MOTOR_ROLL_DIR 12

// Servo motor
bool PitchStepState = HIGH;
bool PitchMotorDirection = 0;
int16_t PitchNumOfSteps = 0;
bool RollStepState = HIGH;
bool RollMotorDirection = 0;
int16_t RollNumOfSteps = 0;
uint8_t deadSteps = 0;

// ADXL Defines
#define DEVICE 0x53
#define SAMPLE_LENGHT 6

// step vars
#define MAX_ANGLE 20
#define MIN_ANGLE -15
#define MICRO_STEP 32
#define GEAR_BOX_REDUCTION 5.18
float ang_per_step = 1.8 / MICRO_STEP / GEAR_BOX_REDUCTION;

// PID FF Gains
float Kff = 0;
float Kp = 0.1;
float Ki = 0;
float Kd = 0;

// PID Vars
float previous_error = 0;
int16_t PitchIntegralError = 0;
int16_t PitchDerivativeError = 0;
int16_t RollIntegralError = 0;
int16_t RollDerivativeError = 0;
float set_point_pitch = 0;
float set_point_roll = 0;
float PitchError = 0;
float RollError = 0;
int16_t PitchStepsError = 0;
int16_t RollStepsError = 0;
int16_t LastPitchStepsError = 0;
int16_t LastRollStepsError = 0;

// timers
Ticker timer1;
Ticker timer2;
esp_timer_handle_t periodic_timer;
long double timeNow = 0;
long double lastTime = 0;
long double rollStepTime = 0;
uint16_t rollStepPeriod = 5000;
long double pitchStepTime = 0;
uint16_t pitchStepPeriod = 5000;
uint16_t PID_Period = 10000; // microsseconds

// ADXL345 object and vars
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
sensors_event_t event;

float a_x = 0;
float a_y = 0;
float a_z = 0;
float xFiltrado = 0;
float yFiltrado = 0;
float zFiltrado = 0;
float pitchFiltrado = 0;
float rollFiltrado = 0;
float roll = 0;
float lastRoll = 0;
float pitch = 0;
float lastPitch = 0;
float rollOffset = 0;
float pitchOffset = 0;
float rollSpeed = 0;
float pitchSpeed = 0;

// UART Obj
// HardwareSerial UART_0(0);
float dataSend = 0;
String receivedCommand = "";
char receivedChar;

void SensorUpdate()
{
  accel.getEvent(&event);

  float yaw = 0;

  xFiltrado = MoveMean(event.acceleration.x, xFiltrado);
  yFiltrado = MoveMean(event.acceleration.y, yFiltrado);
  zFiltrado = MoveMean(event.acceleration.z, zFiltrado);

  roll = atan2(yFiltrado, zFiltrado) * 180.0 / PI;
  roll = roll + rollOffset;
  pitch = atan2(-xFiltrado, sqrt(yFiltrado * yFiltrado + zFiltrado * zFiltrado)) * 180.0 / PI;
  pitch = pitch + pitchOffset;


  // Angular Speed calc
  rollSpeed = (roll - lastRoll) / PID_Period;
  lastRoll = roll;

  pitchSpeed = (pitch - lastPitch) / PID_Period;
  lastPitch = pitch;
}

void rollStep()
{
  // Roll Steps
  timeNow = micros();
  if (timeNow - rollStepTime > rollStepPeriod)
  {
    rollStepTime = timeNow;

    if (RollNumOfSteps != 0)
    {
      digitalWrite(MOTOR_ROLL_DIR, RollMotorDirection);
      digitalWrite(MOTOR_ROLL_STEP, RollStepState);
      RollStepState = !RollStepState;
      if (RollStepState && RollNumOfSteps > 0)
      {
        RollNumOfSteps--;
      }
      else
      {
        RollNumOfSteps++;
      }
    }
  }
}

void pitchStep()
{
  // Pitch Steps
  timeNow = micros();
  if (timeNow - pitchStepTime > pitchStepPeriod)
  {
    pitchStepTime = timeNow;

    if (PitchNumOfSteps != 0)
    {
      digitalWrite(MOTOR_PITCH_DIR, PitchMotorDirection);
      digitalWrite(MOTOR_PITCH_STEP, PitchStepState);
      PitchStepState = !PitchStepState;
      if (PitchStepState && PitchNumOfSteps > 0)
      {
        PitchNumOfSteps--;
      }
      else
      {
        PitchNumOfSteps++;
      }
    }
  }
}

void PID()
{
  // ***************************************************************************** //
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
  else if (abs(PitchError) < deadSteps * ang_per_step)
  {
    PitchError = 0;
  }

  PitchStepsError = int16_t(PitchError / ang_per_step);
  PitchDerivativeError = PitchStepsError - LastPitchStepsError;
  PitchIntegralError += PitchStepsError;

  // limit of integrator
  if (PitchIntegralError > int16_t(MAX_ANGLE / ang_per_step))
  {
    PitchIntegralError = int16_t(MAX_ANGLE / ang_per_step);
  }
  else if (PitchIntegralError < int16_t(MIN_ANGLE / ang_per_step))
  {
    PitchIntegralError = int16_t(MIN_ANGLE / ang_per_step);
  }

  PitchNumOfSteps = (PitchStepsError * Kp) + (PitchIntegralError * Ki) + (PitchDerivativeError * Kd);

  // ***************************************************************************** //
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
  else if (abs(RollError) < deadSteps * ang_per_step)
  {
    RollError = 0;
  }

  RollStepsError = int16_t(RollError / ang_per_step);
  RollDerivativeError = RollStepsError - LastRollStepsError;
  RollIntegralError += RollStepsError;

  // limiting integrator
  if (RollIntegralError > int16_t(MAX_ANGLE / ang_per_step))
  {
    RollIntegralError = int16_t(MAX_ANGLE / ang_per_step);
  }
  else if (RollIntegralError < int16_t(MIN_ANGLE / ang_per_step))
  {
    RollIntegralError = int16_t(MIN_ANGLE / ang_per_step);
  }

  RollNumOfSteps = (RollStepsError * Kp) + (RollIntegralError * Ki) + (RollDerivativeError * Kd);

  // Direction of each angle
  // Pitch motor direction
  if (PitchNumOfSteps > 0)
  {
    PitchMotorDirection = 0;
  }
  else
  {
    PitchMotorDirection = 1;
  }
  // Roll motor direction
  if (RollNumOfSteps > 0)
  {
    RollMotorDirection = 0;
  }
  else
  {
    RollMotorDirection = 1;
  }

  // Step period
  if (RollNumOfSteps != 0)
  {
    rollStepPeriod = PID_Period / abs(RollNumOfSteps) / 2;
  }
  else
  {
    rollStepPeriod = 5000;
  }

  if (PitchNumOfSteps != 0)
  {
    pitchStepPeriod = PID_Period / abs(PitchNumOfSteps) / 2;
  }
  else
  {
    pitchStepPeriod = 5000;
  }
}

void DataSend()
{
  Serial.write(0xAB);

  dataSend = pitch * 1000;
  Serial.write(((int16_t)(dataSend)) >> 8);
  Serial.write((int16_t)(dataSend));

  dataSend = roll * 1000;
  Serial.write(((int16_t)(dataSend)) >> 8);
  Serial.write((int16_t)(dataSend));

  dataSend = set_point_pitch * 1000;
  Serial.write(((int16_t)(dataSend)) >> 8);
  Serial.write((int16_t)(dataSend));

  dataSend = set_point_roll * 1000;
  Serial.write(((int16_t)(dataSend)) >> 8);
  Serial.write((int16_t)(dataSend));
}

void processSerialCommand(String command)
{
  if (command.startsWith("SP;"))
  {
    // Comando de setpoint
    sscanf(command.c_str(), "SP;%f;%f", &set_point_pitch, &set_point_roll);
  }
  else if (command.startsWith("PID;"))
  {
    // Comando de ganhos PID
    sscanf(command.c_str(), "PID;%f;%f;%f", &Kp, &Ki, &Kd);
  }
}

void setup()
{
  Serial.begin(9600);

  pinMode(MOTOR_PITCH_DIR, OUTPUT);
  pinMode(MOTOR_PITCH_STEP, OUTPUT);
  pinMode(MOTOR_ROLL_DIR, OUTPUT);
  pinMode(MOTOR_ROLL_STEP, OUTPUT);

  timer1.attach_ms(0.002, rollStep);
  timer2.attach_ms(0.002, pitchStep);

  delay(1000);

  if (!accel.begin())
  {
    Serial.println("O sensor ADXL345 não foi detectado");
    while (1)
      ;
  }
  accel.setRange(ADXL345_RANGE_16_G);
}

void loop()
{

  // Leitura de dados da serial
  while (Serial.available() > 0)
  {
    receivedChar = Serial.read();
    if (receivedChar == '\n')
    {
      processSerialCommand(receivedCommand);
      receivedCommand = "";
    }
    else
    {
      receivedCommand += receivedChar;
    }
  }

  timeNow = micros();
  if (timeNow - lastTime > PID_Period)
  {
    lastTime = timeNow;
    SensorUpdate();
    PID();
    DataSend();
  }
}
