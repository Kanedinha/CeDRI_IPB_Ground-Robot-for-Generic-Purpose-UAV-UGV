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
// #include "ADXL345.cpp"

// motor defines
#define ENCODER_A 25    // fio branco enc
#define ENCODER_B 26    // fio amarelo enc
#define MOTOR_A1_PIN 16 // R PWM driver
#define MOTOR_A2_PIN 27 // L PWM driver
#define MOTOR_ASPEED_PIN 14
#define MOTOR_A 0
#define MOTOR_B 1

// directions
#define CCW -1 // counter clockwise
#define CW 1   // clockwise
#define STOP 0
#define EE 0 // encoder erro

// ADXL Defines
#define DEVICE 0x53
#define SAMPLE_LENGHT 6

// encoder vars
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

// ADXL345 object and vars
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
float pitchFiltrado = 0;
float rollFiltrado = 0;
float roll = 0;
float pitch = 0;
float rollOffset = 3;
float pitchOffset = 0;
#define ALPHA 0.8
#define N 31

// IIR
float amostraFiltrada = 0.0;
float decaimento = 0.6; // Fator de decaimento

// FIR
#define NUM_COEFICIENTES 93
float coeficientes[NUM_COEFICIENTES] = {
    -0.000082914856142391,
    0.000000000000000000,
    0.000156391860235382,
    0.000329010810197532,
    0.000417515837398014,
    0.000321246121310352,
    -0.000000000000000001,
    -0.000476415876259548,
    -0.000922445054680659,
    -0.001093387248099827,
    -0.000794344610961348,
    0.000000000000000001,
    0.001074602344078270,
    0.002004016174131908,
    0.002297658404639498,
    0.001620402015274704,
    -0.000000000000000002,
    -0.002083824721880051,
    -0.003802766194206129,
    -0.004275371600149631,
    -0.002962263572531809,
    0.000000000000000003,
    0.003695883573596093,
    0.006659177612149735,
    0.007403178437611751,
    0.005079756648464249,
    -0.000000000000000005,
    -0.006244102995226261,
    -0.011193777536036576,
    -0.012402988487857949,
    -0.008497966861335434,
    0.000000000000000006,
    0.010482926244677724,
    0.018898810405901956,
    0.021125202323808830,
    0.014657184176260709,
    -0.000000000000000007,
    -0.018823498383898803,
    -0.034988155777726480,
    -0.040728420000296604,
    -0.029837049948043554,
    0.000000000000000008,
    0.045858933606669018,
    0.099779569257477288,
    0.150579616327804394,
    0.186804439073354672,
    0.199928344940581787,
    0.186804439073354672,
    0.150579616327804394,
    0.099779569257477288,
    0.045858933606669018,
    0.000000000000000008,
    -0.029837049948043586,
    -0.040728420000296604,
    -0.034988155777726453,
    -0.018823498383898803,
    -0.000000000000000007,
    0.014657184176260709,
    0.021125202323808830,
    0.018898810405901956,
    0.010482926244677724,
    0.000000000000000006,
    -0.008497966861335434,
    -0.012402988487857949,
    -0.011193777536036576,
    -0.006244102995226256,
    -0.000000000000000005,
    0.005079756648464249,
    0.007403178437611751,
    0.006659177612149735,
    0.003695883573596093,
    0.000000000000000003,
    -0.002962263572531809,
    -0.004275371600149631,
    -0.003802766194206129,
    -0.002083824721880053,
    -0.000000000000000002,
    0.001620402015274704,
    0.002297658404639498,
    0.002004016174131908,
    0.001074602344078270,
    0.000000000000000001,
    -0.000794344610961349,
    -0.001093387248099827,
    -0.000922445054680659,
    -0.000476415876259548,
    -0.000000000000000001,
    0.000321246121310352,
    0.000417515837398014,
    0.000329010810197533,
    0.000156391860235382,
    0.000000000000000000,
    -0.000082914856142391,
};

float bufferRoll[NUM_COEFICIENTES] = {0};
float bufferPitch[NUM_COEFICIENTES] = {0};
int indice = 0;

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

  Serial.begin(256000);
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

  // update the PID controller values
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

  sensors_event_t event;
  accel.getEvent(&event);

  // --------------------------------------------------------------------------------------- //
  // Roll and Pitch angles

  roll = atan2(event.acceleration.y, event.acceleration.z) * 180.0 / PI;
  roll = roll + rollOffset;
  pitch = atan2(-event.acceleration.x, sqrt(event.acceleration.y * event.acceleration.y + event.acceleration.z * event.acceleration.z)) * 180.0 / PI;
  pitch = pitch + pitchOffset;
  float yaw = 0;

  // --------------------------------------------------------------------------------------- //
  // FIR
  bufferRoll[indice] = roll;
  bufferPitch[indice] = pitch;
  float rollFiltrado = 0;
  float pitchFiltrado = 0;
  for (int i = 0; i < NUM_COEFICIENTES; i++)
  {
    int indiceBuffer = (indice + i) % NUM_COEFICIENTES;
    rollFiltrado += coeficientes[i] * bufferRoll[indiceBuffer];
    pitchFiltrado += coeficientes[i] * bufferPitch[indiceBuffer];
  }
  indice = (indice + 1) % NUM_COEFICIENTES;

  // --------------------------------------------------------------------------------------- //
  // IIR
  // rollFiltrado = (1 - decaimento) * roll + decaimento * amostraFiltrada;
  // pitchFiltrado = (1 - decaimento) * pitch + decaimento * amostraFiltrada;
  // Serial.println(amostraFiltrada);

  // --------------------------------------------------------------------------------------- //
  // Move Mean
  // rollFiltrado = (rollFiltrado * N + roll) / (N + 1);
  // pitchFiltrado = (pitchFiltrado * N + pitch) / (N + 1);

  // --------------------------------------------------------------------------------------- //

  Serial.print(rollFiltrado);
  Serial.print(";");
  Serial.println(pitchFiltrado);

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
