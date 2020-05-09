#include<Wire.h>
#include<SPI.h>
#include<Servo.h>
#include<nRF24L01.h>
#include "printf.h"
#include<RF24.h>
#include "I2Cdev.h"
#include "freeram.h"
#include "mpu.h"

//NRF      ARDUINO
//1 GND    GND
//2 VCC    3.3V
//3 CE     49  out
//4 CSN    53  out
//5 SCK    52 out
//6 MOSI   51 out
//7 MISO   50 in

RF24 Radio(49, 53);

const uint64_t pipeIn = 0xA6A6B8B8E1LL;

struct radio {
  unsigned int Throttle;
  int Pitch;
  int Roll;
  int Yaw;
} RadioData;

Servo Motor1, Motor2, Motor3, Motor4;

float PID_P_ROLL, PID_I_ROLL, PID_D_ROLL, PID_P_PITCH, PID_I_PITCH, PID_D_PITCH, PID_P_YAW, PID_I_YAW, PID_D_YAW;
byte RadioAvail = 0;
float PID_ROLL, PID_PITCH, PID_YAW;
const float P_Gain = 0.9, I_Gain = 0.0, D_Gain = 15, P_Gain_YAW = 7.0, I_Gain_YAW = 0.0, D_Gain_YAW = 0.0, PID_MAX = 200;
float Roll_prev = 0, Radio_roll_prev = 0, Pitch_prev = 0, Radio_pitch_prev = 0, Yaw_prev = 0, Radio_yaw_prev = 0;

float Yaw = 0, Pitch = 0, Roll = 0;
double  Yaw_offset = 0, Roll_offset = 0, Pitch_offset = 0;

byte Motor1Pin = 3, Motor2Pin = 4, Motor3Pin = 5, Motor4Pin = 6;
int Motor1Value = 1000, Motor2Value = 1000, Motor3Value = 1000, Motor4Value = 1000;

int16_t ax, ay, az, gx, gy, gz;

void setup() {
  Serial.begin(115200);
  printf_begin();
  Wire.begin();
  Wire.setClock(400000);
  init_radio();
  Arm_Motor();
  Initialize_MPU9250();
  Calibrate_MPU_9250();
}

void loop() {
  get_radio_data();
  //  print_radio_data();
  calculate_sensor_data();
  //  print_sensor_data();
  PID_Calculations();
  //  print_PID_data();
  motor_corrections();
  print_motor_value();
  write_to_motor();
}

