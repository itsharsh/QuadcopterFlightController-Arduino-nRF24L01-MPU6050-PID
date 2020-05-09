#include<SPI.h>
#include<nRF24L01.h>
#include<RF24.h>
#include "MPU9250.h"
#include "printf.h"
#include<Servo.h>
#include "Kalman.h"

//NRF      ARDUINO
//1 GND    GND
//2 VCC    3.3V
//3 CE     49  out
//4 CSN    53  out
//5 SCK    52 out
//6 MOSI   51 out
//7 MISO   50 in

RF24 Radio(49, 53);

Servo Motor1, Motor2, Motor3, Motor4;

Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

#define SPI_CLOCK 8000000

#define SS_PIN   48
#define INT_PIN  3

MPU9250 IMUSensor(SPI_CLOCK, SS_PIN);


const uint64_t pipeIn = 0xB8B8F0F0E1LL;
uint32_t timer;

double gyroXangle, gyroYangle, gyroZangle;
double kalAngleX, kalAngleY, kalAngleZ;

float Accel[3], Gyro[3], Magnet[3];
float Roll, Pitch, Yaw, Yawoffset;;
double dt;
float roll, pitch, yaw;

float PID_P_ROLL, PID_I_ROLL, PID_D_ROLL, PID_P_PITCH, PID_I_PITCH, PID_D_PITCH, PID_P_YAW, PID_I_YAW, PID_D_YAW;
float PID_ROLL, PID_PITCH, PID_YAW;
float Roll_prev = 0, Radio_roll_prev = 0, Pitch_prev = 0, Radio_pitch_prev = 0, Yaw_prev = 0, Radio_yaw_prev = 0;
const float P_Gain = 1.0, I_Gain = 0.0, D_Gain = 0.0, P_Gain_YAW = 3, I_Gain_YAW = 0.02, D_Gain_YAW = 0.0, PID_MAX = 200;

byte Motor1Pin = 4, Motor2Pin = 5, Motor3Pin = 6, Motor4Pin = 7;
int Motor1Value = 0, Motor2Value = 0, Motor3Value = 0, Motor4Value = 0;

struct radio
{
  unsigned int Throttle;
  int Pitch;
  int Roll;
  int Yaw;
} RadioData;

void setup()
{
  Serial.begin(115200);
  Radio.begin();
  IMUSensor.init(true);
  printf_begin();

  uint8_t wai = IMUSensor.whoami();
  /*if (wai == 0x71) {
    Serial.println("Successful connection");
    }
    else {
    Serial.print("Failed connection: ");
    Serial.println(wai, HEX);
    }*/

  uint8_t wai_AK8963 = IMUSensor.AK8963_whoami();
  /*if (wai_AK8963 == 0x48){
    Serial.println("Successful connection to mag");
    }
    else{
    Serial.print("Failed connection to mag: ");
    Serial.println(wai_AK8963, HEX);
    }*/
  delay(500);

  IMUSensor.calib_acc();
  IMUSensor.calib_mag();
  Calibrate();
  /*
    Serial.print("Yawoffset= ");
    Serial.println(Yawoffset);*/

  Radio.setChannel(50);
  Radio.setPALevel(RF24_PA_MAX);
  Radio.setDataRate(RF24_250KBPS);
  Radio.setAutoAck(false);
  Radio.openReadingPipe(0, pipeIn);
  Radio.printDetails();

  get_sensor_data();
  Convert_Raw_to_Angle();

  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  timer = micros();

  Radio.startListening();
  Arm_Motor();
}

void loop()
{

  get_radio_data();

  get_sensor_data();

  dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  Convert_Raw_to_Angle();
  //  print_raw_data();

  Kalman_Filter();
  print_kalman_data();

  //  Yaw = Yaw - offsetyaw;
  /*Serial.print(Roll); Serial.print("\t");
    Serial.print(Pitch); Serial.print("\t");
    Serial.print(Yaw); Serial.println();*/

  PID_Calculations();
  //  print_PID_value();

  motor_corrections();
  //  print_motor_value();
  write_to_motor();
}

void get_sensor_data()
{
  IMUSensor.read_all();

  Gyro[0] = IMUSensor.gyro_data[0];
  Gyro[1] = IMUSensor.gyro_data[1];
  Gyro[2] = IMUSensor.gyro_data[2];
  Accel[0] = IMUSensor.accel_data[0];
  Accel[1] = IMUSensor.accel_data[1];
  Accel[2] = IMUSensor.accel_data[2];
  Magnet[0] = IMUSensor.mag_data[0];
  Magnet[1] = IMUSensor.mag_data[1];
  Magnet[2] = IMUSensor.mag_data[2];
}

void get_radio_data()
{
  if (Radio.available())
  {
    Radio.read(&RadioData, sizeof(RadioData));
    analogWrite(12, 175);
    //    print_radio_data();
  }
  else
  {
    analogWrite(12, 0);
    RadioData.Throttle = 0;
  }
}

void print_radio_data()
{
  Serial.println(RadioData.Throttle);
  Serial.print("\tYaw=" + RadioData.Yaw);
  Serial.print("\tPitch=" + RadioData.Pitch);
  Serial.println("\tRoll=" + RadioData.Roll);
}

void write_to_motor()
{
  Motor1.writeMicroseconds(Motor1Value);
  Motor2.writeMicroseconds(Motor2Value);
  Motor3.writeMicroseconds(Motor3Value);
  Motor4.writeMicroseconds(Motor4Value);
}

void Arm_Motor()
{
  Motor1.attach(Motor1Pin);
  Motor2.attach(Motor2Pin);
  Motor3.attach(Motor3Pin);
  Motor4.attach(Motor4Pin);
  Motor1.writeMicroseconds(1000);
  Motor2.writeMicroseconds(1000);
  Motor3.writeMicroseconds(1000);
  Motor4.writeMicroseconds(1000);
  delay(1000);
}

void motor_corrections()
{
  //  PID_YAW = 0;
  //  Serial.println(PID_YAW);

  Motor1Value = RadioData.Throttle + PID_PITCH - PID_ROLL - PID_YAW + 750;
  Motor2Value = RadioData.Throttle + PID_PITCH + PID_ROLL + PID_YAW + 750;
  Motor3Value = RadioData.Throttle - PID_PITCH + PID_ROLL - PID_YAW + 750;
  Motor4Value = RadioData.Throttle - PID_PITCH - PID_ROLL + PID_YAW + 750;

  /*if (Motor1Value < 1100) {
    Motor1Value = 1100;
    }
    if (Motor2Value < 1100) {
    Motor2Value = 1100;
    }
    if (Motor3Value < 1100) {
    Motor3Value = 1100;
    }
    if (Motor4Value < 1100) {
    Motor4Value = 1100;
    }*/

}

void print_motor_value()
{
  Serial.print("1   -  ");
  Serial.print(Motor1Value);
  Serial.print("\t2   -  ");
  Serial.print(Motor2Value);
  Serial.print("\t3   -  ");
  Serial.print(Motor3Value);
  Serial.print("\t4   -  ");
  Serial.println(Motor4Value);
}

void Convert_Raw_to_Angle()
{

#ifdef RESTRICT_PITCH
  roll  = atan2(Accel[1], Accel[2]) * RAD_TO_DEG;
  pitch = atan(-Accel[0] / sqrt(Accel[1] * Accel[1] + Accel[2] * Accel[2])) * RAD_TO_DEG;
#else
  roll  = atan(Accel[1] / sqrt(Accel[0] * Accel[0] + Accel[2] * Accel[2])) * RAD_TO_DEG;
  pitch = atan2(-Accel[0], Accel[2]) * RAD_TO_DEG;
#endif
}

void print_raw_data()
{
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.println(yaw);
}

void Kalman_Filter()
{
  double gyroXrate = Gyro[0] / 16.4;
  double gyroYrate = Gyro[1] / 16.4;

#ifdef RESTRICT_PITCH
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
  {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
    gyroXangle = roll;
  }
  else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate;
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90))
  {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
    gyroYangle = pitch;
  }
  else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate;

  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
#endif

  gyroXangle += gyroXrate * dt;
  gyroYangle += gyroYrate * dt;

  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;

  Roll = kalAngleY;
  Pitch = (-1) * kalAngleX;
  float yawrad = atan2( (-Magnet[1] * cos(Roll * DEG_TO_RAD) + Magnet[2] * sin(Roll * DEG_TO_RAD) ) , (Magnet[0] * cos(Pitch * DEG_TO_RAD) + Magnet[1] * sin(Pitch * DEG_TO_RAD) * sin(Roll * DEG_TO_RAD) + Magnet[2] * sin(Pitch * DEG_TO_RAD) * cos(Roll * DEG_TO_RAD)) );
  Yaw = yawrad * RAD_TO_DEG;
  //  Yaw = Yaw * (0.9) + 0.1 * (Gyro[2] * dt);
  //  Serial.println(Yaw);

  double gyroZrate = Gyro[3] / 16.4;
  if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90))
  {
    kalmanZ.setAngle(yaw);
    kalAngleZ = yaw;
    gyroZangle = yaw;
  }
  else
    kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);

  gyroZangle += gyroZrate * dt;
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;
  Yaw = kalAngleZ;
}

void print_kalman_data()
{
  Serial.print(Pitch);
  Serial.print("\t");
  Serial.print(Roll );
  Serial.print("\t");
  Serial.println(Yaw);
}

void PID_Calculations()
{
  Yaw = Yaw - Yawoffset;

  if (RadioData.Roll > -5 && RadioData.Roll < 5)
  {
    RadioData.Roll = 0;
  }
  PID_P_ROLL = (Roll - RadioData.Roll) * P_Gain;
  PID_I_ROLL = PID_I_ROLL + (Roll - RadioData.Roll) * I_Gain;
  PID_D_ROLL = (Roll - RadioData.Roll - Roll_prev - Radio_roll_prev) * D_Gain;

  if (RadioData.Pitch > -5 && RadioData.Pitch < 5)
  {
    RadioData.Pitch = 0;
  }
  PID_P_PITCH = (Pitch - RadioData.Pitch) * P_Gain;
  PID_I_PITCH = PID_I_PITCH + (Pitch - RadioData.Pitch) * I_Gain;
  PID_D_PITCH = (Pitch - RadioData.Pitch - Pitch_prev - Radio_pitch_prev) * D_Gain;

  if (RadioData.Yaw > -5 && RadioData.Yaw < 5)
  {
    RadioData.Yaw = 0;
  }
  PID_P_YAW = (Yaw - RadioData.Yaw) * P_Gain_YAW;
  PID_I_YAW = PID_I_YAW + (Yaw - RadioData.Yaw) * I_Gain_YAW;
  PID_D_YAW = (Yaw - RadioData.Yaw - Yaw_prev - Radio_yaw_prev) * D_Gain_YAW;

  PID_YAW = PID_P_YAW + PID_I_YAW + PID_D_YAW;
  PID_ROLL = PID_I_ROLL + PID_P_ROLL + PID_D_ROLL;
  PID_PITCH = PID_P_PITCH + PID_I_PITCH + PID_D_PITCH;

  Roll_prev = Roll;
  Radio_roll_prev = RadioData.Roll;

  Pitch_prev = Pitch;
  Radio_pitch_prev = RadioData.Pitch;

  Yaw_prev = Yaw;
  Radio_yaw_prev = RadioData.Yaw;

  if (PID_ROLL > PID_MAX)
  {
    PID_ROLL = PID_MAX;
  }
  else if (PID_ROLL < (PID_MAX * (-1)))
  {
    PID_ROLL = PID_MAX * (-1);
  }

  if (PID_PITCH > PID_MAX)
  {
    PID_PITCH = PID_MAX;
  }
  else if (PID_PITCH < (PID_MAX * (-1)))
  {
    PID_PITCH = PID_MAX * (-1);
  }

  if (PID_YAW > PID_MAX)
  {
    PID_YAW = PID_MAX;
  }
  else if (PID_YAW < (PID_MAX * (-1)))
  {
    PID_YAW = PID_MAX * (-1);
  }
}

void print_PID_value()
{
  Serial.print("Yaw= ");
  Serial.print(PID_YAW);
  Serial.print("\tPitch= ");
  Serial.print(PID_PITCH);
  Serial.print("\tRoll= ");
  Serial.println(PID_ROLL);
}

void Calibrate()
{
  double Yawsum = 0;
  int i;
  for (i = 0; i < 1000; i++)
  {
    get_sensor_data();

    dt = (micros() - timer) / 1000000;
    timer = micros();

    Convert_Raw_to_Angle();

    Kalman_Filter();

    Yawsum += Yaw;
  }
  Yawoffset = Yawsum / 1000;
}

