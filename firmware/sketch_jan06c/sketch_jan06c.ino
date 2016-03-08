#include <Wire.h>
#include <L3G4200D.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>
#include "Protocol.h"

L3G4200D gyro;
/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(1);

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(2);

enum RegMap{
  //READ_ONLY_REGS
  TIME_MS_HI = 0,
  TIME_MS_LO = 1,
  GYRO_X = 31,
  GYRO_Y = 32,
  GYRO_Z = 33,
  ACCEL_X = 11,
  ACCEL_Y = 12,
  ACCEL_Z = 13,
  MAG_X = 21,
  MAG_Y = 22,
  MAG_Z = 23 
};

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  gyro.writeReg(L3G4200D_CTRL_REG1, 0x7F);
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_2_G);

  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
}

class SerialOutput : public Output {
  public:
    virtual void put(uint8_t byte){
      Serial.write(byte);
    }
};

class RegHandler : public Handler {
  public:
     virtual void onReadReg(uint8_t address) {}

    virtual void onWriteReg(uint8_t address, uint16_t value) {}
};

SerialOutput output;
RegHandler handler;
Protocol protocol(output, handler);
sensors_event_t event; 

void loop() {
  gyro.read();
  protocol.writeRegister(GYRO_X, gyro.g.x);
  protocol.writeRegister(GYRO_Y, gyro.g.y);
  protocol.writeRegister(GYRO_Z, gyro.g.z);

  mag.getEvent(&event);
  protocol.writeRegister(MAG_X, mag._magData.x);
  protocol.writeRegister(MAG_Y, mag._magData.y);
  protocol.writeRegister(MAG_Z, mag._magData.z);

  accel.getEvent(&event);
  protocol.writeRegister(ACCEL_X, accel.getX());
  protocol.writeRegister(ACCEL_Y, accel.getY());
  protocol.writeRegister(ACCEL_Z, accel.getZ());

  uint32_t currentTimeMillis = micros();
  protocol.writeRegister(TIME_MS_HI, (currentTimeMillis >> 16) & 0xFFFF);
  protocol.writeRegister(TIME_MS_LO, currentTimeMillis & 0xFFFF);  
}
