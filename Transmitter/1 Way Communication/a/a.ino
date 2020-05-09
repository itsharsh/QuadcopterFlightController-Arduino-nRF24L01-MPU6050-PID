#include<SPI.h>
#include "printf.h"
#include<nRF24L01.h>
#include<RF24.h>

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

const uint64_t pipeOut = 0xB8B8F0F0E1LL;

struct radio
{
  unsigned int throttle;
  unsigned int pitch;
  unsigned int roll;
  unsigned int yaw;
} RadioData;

byte throttlePin = A0, yawPin = A1, pitchPin = A2, rollPin = A3;
int throttle, yaw, pitch, roll;

void setup()
{
  Serial.begin(115200);
  Radio.begin();
  printf_begin();
  Radio.setChannel(50);
  Radio.setPALevel(RF24_PA_MAX);
  Radio.setDataRate(RF24_250KBPS);
  Radio.setAutoAck(false);
  Radio.openWritingPipe(pipeOut);
  Radio.printDetails();
}

void loop()
{
  get_joystick_data();

  yaw = map(yaw, 0, 1023, -180, 180);
  roll = map(roll, 0, 1023, -45, 45);
  pitch = map(pitch, 0, 1023, -45, 45);
  //  throttle = map(throttle, 0, 1023, 0, 900);

  /*if (yaw > -7 && yaw < 7)yaw = 0;
    if (roll > -5 && roll < 5)roll = 0;
    if (pitch > -5 && pitch < 5)pitch = 0;*/

  //  print_joystick_data();

  RadioData.throttle = throttle;
  /*RadioData.yaw = yaw;
    RadioData.roll = roll;
    RadioData.pitch = pitch;*/

  Radio.write(&RadioData, sizeof(RadioData));
}

void get_joystick_data()
{
  throttle = analogRead(throttlePin);
  yaw = analogRead(yawPin);
  roll = analogRead(rollPin);
  pitch = analogRead(pitchPin);
}

void print_joystick_data()
{
  Serial.print(throttle);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(roll);
  Serial.print("\t");
  Serial.println(yaw);
}


