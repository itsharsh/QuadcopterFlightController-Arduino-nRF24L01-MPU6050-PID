#include<Wire.h>
#include<SPI.h>
#include<Servo.h>
#include<nRF24L01.h>
#include "printf.h"
#include<RF24.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//NRF      ARDUINO
//1 GND    GND
//2 VCC    3.3V
//3 CE     49  out
//4 CSN    53  out
//5 SCK    52 out
//6 MOSI   51 out
//7 MISO   50 in

RF24 Radio(49, 53);

const uint64_t pipeIn = 0xA6C2B8B8E1LL;

struct radio {
  unsigned int Throttle;
  int Pitch;
  int Roll;
  int Yaw;
  float P;
  float D;
  float P_Yaw;
} RadioData;


struct quad_data {
  unsigned int M1;
  unsigned int M2;
  unsigned int M3;
  unsigned int M4;
  float pid_pitch;
  float pid_roll;
  float pid_yaw;
} quad_rdata;

Servo Motor1, Motor2, Motor3, Motor4;

boolean led_status=false;

float PID_P_ROLL, PID_I_ROLL, PID_D_ROLL, PID_P_PITCH, PID_I_PITCH, PID_D_PITCH, PID_P_YAW, PID_I_YAW, PID_D_YAW;
float PID_ROLL, PID_PITCH, PID_YAW;
float P_Gain = 1.5, I_Gain = 0.0, D_Gain = 35, P_Gain_YAW = 14.0, I_Gain_YAW = 0.0, D_Gain_YAW = 0.0, PID_MAX = 200;
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
  Initialize_MPU6050();
  Calibrate_MPU6050();
}

void loop() {
  get_radio_data();
  setAckData();
  //  print_radio_data();
  calculate_sensor_data();
  //  print_sensor_data();
  offset_corr();
  PID_Calculations();
  //  print_PID_data();
  motor_corrections();
  print_motor_value();
  write_to_motor();
}

