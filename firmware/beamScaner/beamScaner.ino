#include <Servo.h>
#include "Protocol.h"


class SerialOutput : public Output {
  public:
    virtual void put(uint8_t byte) {
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
Servo servo;

void setup() {
  // put your setup code here, to run once:
  servo.attach(9);
  Serial.begin(57600);
  pinMode (A8, INPUT);
  servo.writeMicroseconds(500);

  servo_write_angle(0);
  delay(1000);
}

enum RegMap {
  IRDAR_ADC = 41,
  IRDAR_ANGLE = 42,
  IRDAR_DONE = 43
};

void servo_write_angle(int angle){
  servo.write(map(angle, 0, 180, 10, 165));
}

void report_distance(int angle) {
  float adc = analogRead(A8);
  for ( int i = 0; i < 100; i++) {
    adc = adc * 0.9 + 0.1 * analogRead(A8);
  }

  protocol.writeRegister(IRDAR_ANGLE, angle);
  protocol.writeRegister(IRDAR_ADC, adc);
}

void loop() {
  // put your main code here, to run repeatedly:   
  for (int i = 0;i < 180; i++) {
    servo_write_angle(i);
    delay(50);
    report_distance(i);
  }
   protocol.writeRegister(IRDAR_DONE, 1);
//
  for (int i = 179;i >= 0; i--) {
    servo_write_angle(i);
    delay(50);
    report_distance(i);
  }
  protocol.writeRegister(IRDAR_DONE, 1);     
}
