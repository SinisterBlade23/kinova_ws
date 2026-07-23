#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

BNO08x myIMU;
#define BNO08X_ADDR 0x4B
#define FSR_PIN A0
#define REPORT_INTERVAL_MS 10

unsigned long lastSendTime = 0;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Wire.begin(D2, D1);
  Wire.setClock(100000);

  if (!myIMU.begin(BNO08X_ADDR, Wire, -1, -1)) { Serial.println("BNO086 not found!"); while (1); }
  if (!myIMU.enableRotationVector()) { Serial.println("Could not enable Rotation Vector!"); while (1); }
}

void loop()
{
  unsigned long now = millis();
  if (now - lastSendTime < REPORT_INTERVAL_MS) { yield(); return; }

  int fsr = analogRead(FSR_PIN);

  if (myIMU.getSensorEvent())
  {
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)
    {
      float qi = myIMU.getQuatI();
      float qj = myIMU.getQuatJ();
      float qk = myIMU.getQuatK();
      float qr = myIMU.getQuatReal();

      Serial.print(qi, 6); Serial.print(",");
      Serial.print(qj, 6); Serial.print(",");
      Serial.print(qk, 6); Serial.print(",");
      Serial.print(qr, 6); Serial.print(",");
      Serial.println(fsr);

      lastSendTime = now;
    }
  }
}