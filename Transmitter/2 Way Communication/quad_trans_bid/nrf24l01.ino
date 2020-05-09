void init_radio()
{
  Radio.begin();
  printf_begin();
  Radio.setChannel(103);
  Radio.setPALevel(RF24_PA_MAX);
  Radio.setDataRate(RF24_2MBPS);
  Radio.setAutoAck(true);
  Radio.enableAckPayload();
  Radio.enableDynamicPayloads();
  Radio.openWritingPipe(pipeOut);
  Radio.setRetries(20, 10);
  Radio.printDetails();
  RadioData.P = 0.5;
  RadioData.D = 5;
  RadioData.P_Yaw = 9;
}

void send_data()
{
  if (Radio.write(&RadioData, sizeof(RadioData)))
  {
    if (Radio.isAckPayloadAvailable())
    {
      Radio.read(&quad_rdata, sizeof(quad_rdata));
      display_data();
    }
    else
    {
      Serial.println("Connection Lost");
    }
  }
}

