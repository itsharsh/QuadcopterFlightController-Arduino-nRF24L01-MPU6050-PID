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

byte throttlePin = A0, yawPin = A1, pitchPin = A3, rollPin = A2;
int throttle, Yaw, Pitch, Roll, ch = 0;
boolean bool_display = true, set_P = false, set_D = false, set_PYaw = false;

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

void setup() {
  Serial.begin(115200);
  Radio.begin();
  printf_begin();
  Radio.setChannel(85);
  Radio.setPALevel(RF24_PA_MAX);
  Radio.setDataRate(RF24_2MBPS);
  Radio.setAutoAck(true);
  Radio.enableAckPayload();
  Radio.enableDynamicPayloads();
  Radio.openWritingPipe(pipeOut);
  Radio.setRetries(5, 5);
  Radio.printDetails();
  //  RadioData.P=1.6;
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
}

void loop() {

  throttle = analogRead(throttlePin);
  Yaw = analogRead(yawPin);
  Roll = analogRead(rollPin);
  Pitch = analogRead(pitchPin);

  if (Serial.available()) {
    ch = Serial.parseInt();
    switch (ch) {
      case 0:
        bool_display = true;
        break;
      case 1:
        set_P = true;
        bool_display = false;
        break;
      case 2:
        set_D = true;
        bool_display = false;
        break;
      case 3:
        set_PYaw = true;
        bool_display = false;
        break;
      default:
        Serial.println("Wrong Input");
    }
  }

  RadioData.throttle = throttle;
  RadioData.Yaw = map(Yaw, 0, 1023, -180, 180);
  RadioData.Roll = map(Roll, 0, 1023, 180, -180);
  RadioData.Pitch = map(Pitch, 0, 1023, 180, -180);
  /*
    Serial.print(throttle); Serial.print("\t");
    Serial.print(Yaw); Serial.print("\t");
    Serial.print(Roll); Serial.print("\t");
    Serial.print(Pitch); Serial.print("\t");
    Serial.println(RadioData.P);
  */
  setP();
  setD();
  setP_Yaw();
  if (Radio.write(&RadioData, sizeof(RadioData))) {
    if (Radio.isAckPayloadAvailable()) {
      Radio.read(&quad_rdata, sizeof(quad_rdata));
      display_data();
    }
    else {
      Serial.println("Connection Lost");
    }
  }
}

void display_data() {
  if (bool_display) {
    Serial.print(quad_rdata.M1); Serial.print("\t");
    Serial.print(quad_rdata.M2); Serial.print("\t");
    Serial.print(quad_rdata.M3); Serial.print("\t");
    Serial.print(quad_rdata.M4); Serial.print("\t");
    Serial.print(quad_rdata.pid_pitch); Serial.print("\t");
    Serial.print(quad_rdata.pid_roll); Serial.print("\t");
    Serial.println(quad_rdata.pid_yaw);
  }
}

void setP() {
  if (set_P) {
    if (Serial.available()) {
      RadioData.P = Serial.parseFloat();
      Serial.println(RadioData.P);
      set_P = false;
    }
  }
}

void setD() {
  if (set_D) {
    if (Serial.available()) {
      RadioData.D = Serial.parseFloat();
      Serial.println(RadioData.D);
      set_D = false;
    }
  }
}

void setP_Yaw() {
  if (set_PYaw) {
    if (Serial.available()) {
      RadioData.P_Yaw = Serial.parseFloat();
      Serial.println(RadioData.P_Yaw);
      set_PYaw = false;
    }
  }
}

