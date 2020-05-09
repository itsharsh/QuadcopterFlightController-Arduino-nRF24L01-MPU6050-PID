void init_radio() {
  Radio.begin();
  Radio.setChannel(85);
  Radio.setPALevel(RF24_PA_MAX);
  Radio.setDataRate(RF24_2MBPS);
  Radio.setAutoAck(false);
  Radio.openReadingPipe(0, pipeIn);
  Radio.printDetails();
  Radio.startListening();
}

void get_radio_data() {
  if (Radio.available()) {
    Radio.read(&RadioData, sizeof(RadioData));
    RadioAvail=1;
    analogWrite(12, 175);
    RadioData.Yaw=0;
    RadioData.Pitch=0;
    RadioData.Roll=0;
  }
  else{
    RadioAvail=0;
    RadioData.Throttle=0;
    analogWrite(12, 0);
  }
}

void print_radio_data() {
  Serial.print(RadioData.Throttle); Serial.print("\t");
  Serial.print(RadioData.Yaw); Serial.print("\t");
  Serial.print(RadioData.Pitch); Serial.print("\t");
  Serial.println(RadioData.Roll);
}
