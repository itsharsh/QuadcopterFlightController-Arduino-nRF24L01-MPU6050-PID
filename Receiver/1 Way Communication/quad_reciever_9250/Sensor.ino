void Initialize_MPU9250() {
  Fastwire::setup(400, 0);
  mympu_open(200);
}

void calculate_sensor_data() {
  mympu_update();
  Roll = mympu.ypr[1];
  Pitch = mympu.ypr[2];
  Pitch = (-1) * Pitch;
  if (Pitch < 0) {
    Pitch = Pitch + 180;
  }
  else if (Pitch > 0) {
    Pitch = Pitch - 180;
  }
  Pitch=(-1)*Pitch;
  Roll=(-1)*Roll;
  Yaw = mympu.gyro[0];
}

void print_sensor_data() {
  Serial.print(Yaw); Serial.print("\t");
  Serial.print(Roll); Serial.print("\t");
  Serial.println(Pitch);
}

void Calibrate_MPU_9250() {
  double timer = millis();
  while ((millis() - timer) < 18000) {
    calculate_sensor_data();
    if (millis() % 500 == 0)digitalWrite(22, !digitalRead(22));
  }
  timer = millis();
  int i = 0;
  while ((millis() - timer) < 1000) {
    if (millis() % 500 == 0)digitalWrite(22, !digitalRead(22));
    calculate_sensor_data();
    Yaw_offset += Yaw;
    Pitch_offset += Pitch;
    Roll_offset += Roll;
    i++;
  }
  Yaw_offset = Yaw_offset / i;
  Pitch_offset = Pitch_offset / i;
  Roll_offset = Roll_offset / i;
  digitalWrite(22, LOW);
}

