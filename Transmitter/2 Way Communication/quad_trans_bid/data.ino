void get_remote_data()
{
  throttle = analogRead(throttlePin);
  Yaw = analogRead(yawPin);
  Roll = analogRead(rollPin);
  Pitch = analogRead(pitchPin);

  RadioData.throttle = throttle;
  RadioData.Yaw = map(Yaw, 0, 1023, -15, 15);
  RadioData.Roll = map(Roll, 0, 1023, -19, 21);
  RadioData.Pitch = map(Pitch, 0, 1023, -19, 21);

  /*Serial.print(throttle); Serial.print("\t");
    Serial.print(Yaw); Serial.print("\t");
    Serial.print(Roll); Serial.print("\t");
    Serial.println(Pitch);*/
  RadioData.Pitch += 1;
}

void timeplot()
{
  TimePlot M1("M1");
  TimePlot M2("M2");
  TimePlot M3("M3");
  TimePlot M4("M4");

  M1.SendData("Motor 1", quad_rdata.M1);
  M2.SendData("Motor 2", quad_rdata.M2);
  M3.SendData("Motor 3", quad_rdata.M3);
  M4.SendData("Motor 4", quad_rdata.M4);
}

void display_data()
{
  timeplot();
  //  Serial.print(throttle); Serial.print("\t");
  //  Serial.print(Yaw); Serial.print("\t");
  //  Serial.print(Roll); Serial.print("\t");
  //  Serial.print(Pitch); Serial.print("\t");

  //  Serial.print(quad_rdata.M1); Serial.print("\t");
  //  Serial.print(quad_rdata.M2); Serial.print("\t");
  //  Serial.print(quad_rdata.M3); Serial.print("\t");
  //  Serial.print(quad_rdata.M4); Serial.print("\t");

  Serial.print(RadioData.P); Serial.print("\t");
  Serial.print(RadioData.D); Serial.print("\t");
  Serial.print(RadioData.P_Yaw); Serial.print("\t");

  Serial.print(quad_rdata.pid_pitch); Serial.print("\t");
  Serial.print(quad_rdata.pid_roll); Serial.print("\t");
  Serial.print(quad_rdata.pid_yaw); Serial.print("\t");

  Serial.print("\n");
}
