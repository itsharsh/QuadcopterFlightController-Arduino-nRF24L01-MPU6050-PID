void tuning()
{
  if (Serial.available())
  {
    ch = Serial.parseInt();
    switch (ch)
    {
      case 1:
        set_P = true;
        break;
      case 2:
        set_D = true;
        break;
      case 3:
        set_PYaw = true;
        break;
      default:
        Serial.println("Wrong Input");
    }
    ch = 0;
  }
  setP();
  setD();
  setP_Yaw();
}

void setP()
{
  if (set_P)
  {
    if (Serial.available())
    {
      RadioData.P = Serial.parseFloat();
      Serial.println(RadioData.P);
      set_P = false;
    }
  }
}

void setD()
{
  if (set_D)
  {
    if (Serial.available())
    {
      RadioData.D = Serial.parseFloat();
      Serial.println(RadioData.D);
      set_D = false;
    }
  }
}

void setP_Yaw()
{
  if (set_PYaw)
  {
    if (Serial.available())
    {
      RadioData.P_Yaw = Serial.parseFloat();
      Serial.println(RadioData.P_Yaw);
      set_PYaw = false;
    }
  }
}
