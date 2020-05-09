void PID_Calculations() {

  /*Serial.print(Pitch); Serial.print("\t");
    Serial.print(Roll); Serial.print("\t");
    Serial.println(Yaw);*/

  if (RadioData.Roll > -5 && RadioData.Roll < 5)
  {
//    RadioData.Roll = 0;
  }
  PID_P_ROLL = (Roll - RadioData.Roll) * P_Gain;
  PID_I_ROLL = PID_I_ROLL + (Roll - RadioData.Roll) * I_Gain;
  PID_D_ROLL = (Roll - RadioData.Roll - Roll_prev + Radio_roll_prev) * D_Gain;    //error=sensor-receiver

  if (RadioData.Pitch > -5 && RadioData.Pitch < 5)
  {
//    RadioData.Pitch = 0;
  }
  PID_P_PITCH = (Pitch - RadioData.Pitch) * P_Gain;
  PID_I_PITCH = PID_I_PITCH + (Pitch - RadioData.Pitch) * I_Gain;
  PID_D_PITCH = (Pitch - RadioData.Pitch - Pitch_prev + Radio_pitch_prev) * D_Gain;

  if (RadioData.Yaw > -5 && RadioData.Yaw < 5)
  {
//    RadioData.Yaw = 0;
  }
  PID_P_YAW = (Yaw - RadioData.Yaw) * P_Gain_YAW;
  PID_I_YAW = PID_I_YAW + (Yaw - RadioData.Yaw) * I_Gain_YAW;
  PID_D_YAW = (Yaw - RadioData.Yaw - Yaw_prev + Radio_yaw_prev) * D_Gain_YAW;

  PID_YAW = PID_P_YAW + PID_I_YAW - PID_D_YAW;
  PID_ROLL = PID_I_ROLL + PID_P_ROLL - PID_D_ROLL;
  PID_PITCH = PID_P_PITCH + PID_I_PITCH - PID_D_PITCH;

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

void print_PID_data() {
  Serial.print(PID_PITCH); Serial.print("\t");
  Serial.print(PID_ROLL); Serial.print("\t");
  Serial.println(PID_YAW);
}

