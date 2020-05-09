void motor_corrections() {
  //    Serial.println(PID_YAW);
  Motor1Value = RadioData.Throttle - PID_PITCH + PID_ROLL - PID_YAW + 750;
  Motor2Value = RadioData.Throttle - PID_PITCH - PID_ROLL + PID_YAW + 750;
  Motor3Value = RadioData.Throttle + PID_PITCH - PID_ROLL - PID_YAW + 750;
  Motor4Value = RadioData.Throttle + PID_PITCH + PID_ROLL + PID_YAW + 750;

  if (Motor1Value < 1000) {
    Motor1Value = 1000;
  }
  if (Motor2Value < 1000) {
    Motor2Value = 1000;
  }
  if (Motor3Value < 1000) {
    Motor3Value = 1000;
  }
  if (Motor4Value < 1000) {
    Motor4Value = 1000;
  }
}

void Arm_Motor() {
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

void write_to_motor() {
  Motor1.writeMicroseconds(Motor1Value);
  Motor2.writeMicroseconds(Motor2Value);
  Motor3.writeMicroseconds(Motor3Value);
  Motor4.writeMicroseconds(Motor4Value);
}

void print_motor_value() {
  Serial.print("1   -  ");
  Serial.print(Motor1Value);
  Serial.print("\t2   -  ");
  Serial.print(Motor2Value);
  Serial.print("\t3   -  ");
  Serial.print(Motor3Value);
  Serial.print("\t4   -  ");
  Serial.println(Motor4Value);
}
