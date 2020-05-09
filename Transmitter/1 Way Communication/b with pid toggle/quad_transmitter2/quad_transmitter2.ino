#include<SPI.h>
#include<LiquidCrystal.h>
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
LiquidCrystal Lcd(3, 4, 5, 6, 7, 8);

const uint64_t pipeOut = 0xB8B8F0F0E1LL;

struct radio {
  unsigned int throttle;
  unsigned int pitch;
  unsigned int roll;
  unsigned int yaw;
  float P_Gain;
  float I_Gain;
  byte D_Gain;

} RadioData;

byte throttlePin = A0, yawPin = A1, pitchPin = A2, rollPin = A3, toogle = 0;
int throttle, yaw, pitch, roll, i = 0;
boolean flag = 0, flagup = 0, flagdown = 0, up = 0, down = 0;

void setup() {
  Serial.begin(115200);
  Radio.begin();
  Lcd.begin(16, 2);
  printf_begin();
  Radio.setChannel(50);
  Radio.setPALevel(RF24_PA_MAX);
  Radio.setDataRate(RF24_250KBPS);
  Radio.setAutoAck(false);
  Radio.openWritingPipe(pipeOut);
  Radio.printDetails();

  RadioData.P_Gain = 0;
  RadioData.I_Gain = 0;
  RadioData.D_Gain = 0;
}
uint32_t timer;
void loop() {
  timer = micros();
  set_PID();
  Serial.print(toogle); Serial.print("\t");
  Serial.print(up); Serial.print("\t");
  Serial.println(down);
  throttle = analogRead(throttlePin);
  yaw = analogRead(yawPin);
  roll = analogRead(rollPin);
  pitch = analogRead(pitchPin);
  /*Serial.print(roll);Serial.print("\t");
    Serial.println(pitch);*/

  //  yaw = map(yaw, 0, 1023, -180, 180);
  //  roll = map(roll, 0, 1023, -45, 45);
  //  pitch = map(pitch, 0, 1023, -45, 45);
  //  throttle = map(throttle, 0, 1023, 0, 900);

  Display();
  RadioData.throttle = throttle;
  RadioData.yaw = yaw;
  RadioData.roll = roll;
  RadioData.pitch = pitch;
  /*Lcd.clear();
    Lcd.setCursor(0, 0); Lcd.print(throttle);
    Lcd.setCursor(8, 0); Lcd.print(yaw);
    Lcd.setCursor(0, 1); Lcd.print(pitch);
    Lcd.setCursor(8, 1); Lcd.print(roll);*/

  Radio.write(&RadioData, sizeof(RadioData));
  Serial.println((double)(micros() - timer));
}

void set_PID()
{
  if (roll > 600 && flag == 1) {
    toogle = i % 3;
    i++;
    flag = 0;
  }
  else if (roll < 600) {
    flag = 1;
  }
  /*Serial.println(flagup);
    Serial.println(flagdown);
    Serial.println(pitch);*/
  if (pitch > 600)
  {
    if (flagup == 1)
      up = 1;
    else
      up = 0;
    flagup = 0;
  }
  else if (pitch < 600)
  {
    up = 0;
    flagup = 1;
  }

  if (yaw > 600  )
  {
    if (flagdown == 1)
      down = 1;
    else
      down = 0;
    flagdown = 0;
  }
  else if (yaw < 600 )
  {
    flagdown = 1;
    down = 0;
  }
  switch (toogle) {

    case 0:
      PGain();
      break;

    case 1:
      IGain();
      break;

    case 2:
      DGain();
      break;

  }
}

void Display() {
  if (toogle == 0) {
    Lcd.setCursor(0, 0);
    if (millis() % 1000 < 500) {
      Lcd.print(RadioData.P_Gain);
    }
    else {
      Lcd.clear();
    }
    Lcd.setCursor(5, 0);
    Lcd.print(RadioData.I_Gain);
    Lcd.setCursor(10, 0);
    Lcd.print(RadioData.D_Gain);
  }
  else if (toogle == 1) {
    Lcd.setCursor(0, 0);
    Lcd.print(RadioData.P_Gain);
    if (millis() % 1000 < 500) {
      Lcd.setCursor(5, 0);
      Lcd.print(RadioData.I_Gain);
    }
    else {
      Lcd.clear();
    }
    Lcd.setCursor(10, 0);
    Lcd.print(RadioData.D_Gain);
  }
  else if (toogle == 2) {
    Lcd.setCursor(0, 0);
    Lcd.print(RadioData.P_Gain);
    Lcd.setCursor(5, 0);
    Lcd.print(RadioData.I_Gain);
    if (millis() % 1000 < 500) {
      Lcd.setCursor(10, 0);
      Lcd.print(RadioData.D_Gain);
    }
    else {
      Lcd.clear();
    }
  }
}

void PGain() {
  if (up) {
    RadioData.P_Gain += 0.1;
  }
  if (down) {
    RadioData.P_Gain -= 0.1;
  }
  if (RadioData.P_Gain < 0) {
    RadioData.P_Gain = 0;
  }
}

void IGain() {
  if (up) {
    RadioData.I_Gain += 0.01;
  }
  if (down) {
    RadioData.I_Gain -= 0.01;
  }
  if (RadioData.I_Gain < 0) {
    RadioData.I_Gain = 0;
  }
}

void DGain() {
  if (up) {
    RadioData.D_Gain += 1;
  }
  if (down) {
    RadioData.D_Gain -= 1;
  }
  if (RadioData.D_Gain < 0) {
    RadioData.D_Gain = 0;
  }
}
