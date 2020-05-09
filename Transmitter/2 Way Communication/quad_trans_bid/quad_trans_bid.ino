#include<SPI.h>
#include "printf.h"
#include<nRF24L01.h>
#include<RF24.h>
#include"MegunoLink.h"

//NRF      ARDUINO
//1 GND    GND
//2 VCC    3.3V
//3 CE     9  out
//4 CSN    10  out
//5 SCK    13 out
//6 MOSI   11 out
//7 MISO   12 in
//8 IRQ    2  in

RF24 Radio(9, 10);
const uint64_t pipeOut = 0xA6C2B8B8E1LL;

byte throttlePin = A0, yawPin = A1, pitchPin = A3, rollPin = A2;
int throttle, Yaw, Pitch, Roll, ch = 0;
boolean set_P = false, set_D = false, set_PYaw = false;

struct radio {
  unsigned int throttle;
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

//7 Gnd

void setup()
{
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);

  init_radio();
}

void loop()
{
  get_remote_data();
  //  display_data();
  tuning();
  send_data();
}


