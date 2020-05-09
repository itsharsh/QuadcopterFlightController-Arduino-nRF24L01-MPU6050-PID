void init_radio() {
  Radio.begin();
  Radio.setChannel(103);
  Radio.setPALevel(RF24_PA_MAX);
  Radio.setDataRate(RF24_2MBPS);
  Radio.setAutoAck(true);
  Radio.enableAckPayload();
  Radio.enableDynamicPayloads();
  Radio.openReadingPipe(0, pipeIn);
  Radio.printDetails();
  Radio.setRetries(0, 0);

  Radio.startListening();

  pinMode(10, OUTPUT);
  analogWrite(10, 0);
}

void get_radio_data() {
  if (Radio.available()) {
    Radio.read(&RadioData, sizeof(RadioData));
    Radio.writeAckPayload(0, &quad_rdata, sizeof(quad_rdata));
    analogWrite(12, 175);
    P_Gain = RadioData.P;
    D_Gain = RadioData.D;
    P_Gain_YAW = RadioData.P_Yaw;
  }
  else {
    RadioData.Throttle = 0;
    analogWrite(12, 0);
  }
}

void print_radio_data() {
  Serial.print(RadioData.Throttle); Serial.print("\t");
  Serial.print(RadioData.Yaw);      Serial.print("\t");
  Serial.print(RadioData.Pitch);    Serial.print("\t");
  Serial.print(RadioData.Roll);     Serial.print("\t");
  Serial.print(RadioData.P);        Serial.print("\t");
  Serial.print(RadioData.D);        Serial.print("\t");
  Serial.println(RadioData.P_Yaw);
}

void setAckData() {
  quad_rdata.M1 = Motor1Value;
  quad_rdata.M2 = Motor2Value;
  quad_rdata.M3 = Motor3Value;
  quad_rdata.M4 = Motor4Value;
  quad_rdata.pid_pitch = PID_PITCH;
  quad_rdata.pid_roll = PID_ROLL;
  quad_rdata.pid_yaw = PID_YAW;
}
