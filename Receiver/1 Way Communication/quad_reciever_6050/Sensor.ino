MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];


Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

void Initialize_MPU6050() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);    //220
  mpu.setYGyroOffset(76);     //76
  mpu.setZGyroOffset(-85);     //36
  mpu.setZAccelOffset(1788);    //1600

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    //mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  //mpu.calibrateGyro();
  pinMode(LED_PIN, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(22, OUTPUT);
  digitalWrite(23, LOW);
  digitalWrite(22, LOW);
}

void calculate_sensor_data() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();

  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Yaw = ypr[0] * 180 / M_PI;
    Roll = ypr[1] * 180 / M_PI;
    Pitch = ypr[2] * 180 / M_PI;
#endif
  }
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Yaw = (float)gz * 250 / 32768;
  
}

void print_sensor_data() {
  Serial.print(Yaw); Serial.print("\t");
  Serial.print(Roll); Serial.print("\t");
  Serial.println(Pitch);
}

void Calibrate_MPU6050() {
  double timer = millis();
  while ((millis() - timer) < 21000) {
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

