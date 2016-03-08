#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include "Protocol.h"

Servo servo;

int trigPin = 25;
int echoPin = 24;

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

void setup() {
  servo.attach(9);
  //pinMode(sensors[i].trigPin, OUTPUT);
  //pinMode(sensors[i].echoPin, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial3.begin(9600);
}

 int16_t measureDistance(){
    //int16_t duration, distance; // Duration used to calculate distance
  
    //digitalWrite(trigPin, LOW);
    //delayMicroseconds(10);
  
    //digitalWrite(trigPin, HIGH);
    //delayMicroseconds(6);
  
    //digitalWrite(trigPin, LOW);
    //duration = pulseIn(echoPin, HIGH);
  
    //distance = (duration / 100) * 1.657;
    //return distance;
    Serial3.flush();     // clear receive buffer of serial port
    Serial3.write(0X55); // trig US-100 begin to measure the distance
    bool hasData = false;
    for(int i = 0; i < 100; i++){
      if(Serial3.available() >= 2){
        hasData = true;
        break;
      }else{
        delay(10);
      }
    }
    if(hasData){                    //when receive 2 bytes {
      unsigned int HighLen = Serial3.read();                   //High byte of distance
      unsigned int LowLen  = Serial3.read();                   //Low byte of distance
      unsigned int Len_mm  = HighLen*256 + LowLen;             //Calculate the distance
    
      if((Len_mm > 1) && (Len_mm < 10000)){        //normal distance should between 1mm and 10000mm (1mm, 10m)
            return Len_mm;
      }else{
        return 0;
      }
    }
}  

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
  MAG_Z = 23,
  SODAR_DIST = 41,
  SODAR_ANGLE = 42
};

void report_distance(int angle){
  int16_t distance = measureDistance();
  for( int i = 0; i < 1000; i++){
    distance = distance * 0.5 + 0.5 * measureDistance(); 
  }
  
  protocol.writeRegister(SODAR_DIST, distance);
  protocol.writeRegister(SODAR_ANGLE, angle); 
}

void loop() {
  // put your main code here, to run repeatedly: 
  for(int i = 0; i < 160; i+=2){        
    servo.write(i);
    report_distance(i);   
  }

  for(int i = 160; i > 0; i-=2){
    servo.write(i);
    report_distance(i);     
  }
}
