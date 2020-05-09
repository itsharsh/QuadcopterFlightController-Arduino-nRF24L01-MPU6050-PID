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
const uint64_t pipeOut = 0xA6A6B8B8E1LL;

byte throttlePin = A0, yawPin = A1, pitchPin = A2, rollPin = A3;
int throttle, yaw, pitch, roll;

struct radio {
  unsigned int throttle;
  int pitch;
  int roll;
  int yaw;
} RadioData;

void setup() {
  Serial.begin(115200);
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  Radio.begin();
  printf_begin();
  Radio.setChannel(85);
  Radio.setPALevel(RF24_PA_MAX);
  Radio.setDataRate(RF24_2MBPS);
  Radio.setAutoAck(false);
  Radio.openWritingPipe(pipeOut);
  Radio.printDetails();
//  RadioData.P = 1.6;
}

void loop() {
  throttle = analogRead(throttlePin);
  yaw = analogRead(yawPin);
  roll = analogRead(rollPin);
  pitch = analogRead(pitchPin);

  if (Serial.available()) {
//    RadioData.P = Serial.parseFloat();
  }

  RadioData.throttle = throttle;
  RadioData.yaw = map(yaw, 0, 1023, -180, 180);
  RadioData.roll = map(roll, 0, 1023, -180, 180);
  RadioData.pitch = map(pitch, 0, 1023, -180, 180);

  Serial.print(throttle); Serial.print("\t");
  Serial.print(yaw); Serial.print("\t");
  Serial.print(roll); Serial.print("\t");
  Serial.print(pitch); Serial.println("\t");
//  Serial.println(RadioData.P);

  Radio.write(&RadioData, sizeof(RadioData));
}
