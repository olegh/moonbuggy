
#include <AFMotor.h>
#include <TimerOne.h>
#include "Protocol.h"
#include "Regs.h"

AF_DCMotor leftHeadMotor(4);
AF_DCMotor leftTailMotor(2);
AF_DCMotor rightHeadMotor(3);
AF_DCMotor rightTailMotor(1);

#define TIMER_PERIOD_US 500
#define REGULATOR_PERIOD_US 50000



struct SPid
{
  float dState;                  // Last position input
  float iState;                  // Integrator state
  float iMax, iMin;      
  // Maximum and minimum allowable integrator state
  float    iGain,        // integral gain
           pGain,        // proportional gain
           dGain;         // derivative gain

  float UpdatePID(float error, float pos){
    float pTerm = 0, dTerm = 0, iTerm = 0;
   
    pTerm = pGain * error;    // calculate the proportional term
    iState += error;          // calculate the integral state with appropriate limiting
   
    if (iState > iMax) 
        iState = iMax;     
    else if (iState < iMin) 
        iState = iMin;
        
    iTerm = iGain * iState;    // calculate the integral term
    dTerm = dGain * (pos - dState);
    dState = pos;
    
    return (pTerm + iTerm - dTerm);
  }             
};

struct MotorController{
  MotorController(int8_t pin):
    prevState_(HIGH)
    ,pin_(pin)
    ,counter_(0)
    ,step_(0)
    ,lastRPS_(0) 
    ,targetRPS_(0)
    ,pid_()   
    {
      pid_.dState = 0;
      pid_.iState = 0;
      pid_.iMax = 10000;
      pid_.iMin = 0;
      pid_.iGain = 0.5;
      pid_.pGain = 1;
      pid_.dGain = 0.2;      
    }

  void reset(){
     step_ = 0; 
     lastRPS_ = 0;
  }
  
  void onTimer(){
    bool currentState = digitalRead(pin_);
    if(prevState_ == HIGH && currentState == LOW ){
      incCounter();
      if(lastSeenLOW_ == 0){
        lastSeenLOW_ = micros();
      }else{
        uint32_t period = micros() - lastSeenLOW_;
        if(lastRPS_ == 0){
          lastRPS_ = (1000000 / period);
        }else{
          lastRPS_ = lastRPS_ * 0.5 + 0.5 * (1000000 / period);
        }
        lastSeenLOW_ = micros();
      }
    }
    prevState_ = currentState;
    
    ++step_;    
    if(step_ >= (REGULATOR_PERIOD_US / TIMER_PERIOD_US)){      
      regulate();
      reset();      
    }
  } 

  int16_t getRPS(){
    if(targetRPS_ >= 0){
      return lastRPS_;
    }else{
      return -lastRPS_;
    }
  }

  void setTarget(int16_t target){
    targetRPS_ = target;
  }

  uint16_t getControl(){
    return control_;
  }

  int16_t getCounter(){
    return counter_;
  }
  
  private:
    void regulate(){ 
      //lastRPS_ = counter_;
      int16_t u = pid_.UpdatePID(abs(targetRPS_) - lastRPS_, lastRPS_);
      if( u < 0 ){
        u = 0;
      }
      control_ = map( u, 0, 100, 0, 254); 
      if( control_ > 254 ) control_ = 254;     
    }

    void incCounter(){
      if(targetRPS_ > 0) ++counter_;
      else --counter_;
    }
  
    bool prevState_;
    int8_t pin_;
    volatile int16_t counter_;
    uint16_t step_; 
    volatile float lastRPS_;
    volatile float control_;
    volatile int16_t targetRPS_;
    uint32_t lastSeenLOW_ = 0;
    SPid pid_;    

   public:
   float getIState(){
     return pid_.iState;     
   }
};

MotorController motorLeftFront(A2);
MotorController motorLeftRear((A3));
MotorController motorRightFront((A1));
MotorController motorRightRear((A0));

void timerISR(){
  digitalWrite(A4, LOW);
  motorLeftFront.onTimer();
  motorLeftRear.onTimer();
  motorRightFront.onTimer();
  motorRightRear.onTimer();
  digitalWrite(A4, HIGH);  
}

uint32_t lastCmdReceivedTime;
uint32_t lastRPSReport;

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, OUTPUT);
  
  Timer1.initialize(TIMER_PERIOD_US);
  Timer1.attachInterrupt( timerISR );

  Serial.begin(57600);  

  delay(5);
  
  lastCmdReceivedTime = millis();
  lastRPSReport = millis();
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

    virtual void onWriteReg(uint8_t address, uint16_t value) {
      int16_t signedVal = (int16_t)value;
      lastCmdReceivedTime = millis();
      switch(address){
        case LEFT_FRONT_ROTOR_RPS:
          setMotor(motorLeftFront, leftHeadMotor, signedVal);
          break;

         case LEFT_REAR_ROTOR_RPS:
          setMotor(motorLeftRear, leftTailMotor, signedVal);
          break;  
          
        case RIGHT_FRONT_ROTOR_RPS:
          setMotor(motorRightFront, rightHeadMotor, signedVal);
          break;    
          
        case RIGHT_REAR_ROTOR_RPS:
          setMotor(motorRightRear, rightTailMotor, signedVal);
          break;

        default:     
         break;
      }
    }

  private:
   void setMotor(MotorController& controller,
                 AF_DCMotor& motor,                 
                 int16_t value){    
     if(0 == value){
       controller.setTarget(0);
       motor.run(BRAKE);
     }else if(value > 0){
      controller.setTarget(value);
      motor.run(FORWARD);       
     }else{
      controller.setTarget(value);
      motor.run(BACKWARD);      
     }
   }
};

SerialOutput output;
RegHandler handler;
Protocol protocol(output, handler);

void loop() {
  if(abs(millis() - lastRPSReport) > 50){
     lastRPSReport = millis();
     protocol.writeRegister(LEFT_FRONT_ROTOR_ROUNDS, motorLeftFront.getCounter());
     protocol.writeRegister(LEFT_REAR_ROTOR_ROUNDS, motorLeftRear.getCounter());
     protocol.writeRegister(RIGHT_FRONT_ROTOR_ROUNDS, motorRightFront.getCounter());
     protocol.writeRegister(RIGHT_REAR_ROTOR_ROUNDS, motorRightRear.getCounter());
  }
  
  if(abs(millis() - lastCmdReceivedTime) < 200){
    rightHeadMotor.setSpeed(motorRightFront.getControl());  
    rightTailMotor.setSpeed(motorRightRear.getControl());  
    leftHeadMotor.setSpeed(motorLeftFront.getControl());  
    leftTailMotor.setSpeed(motorLeftRear.getControl());
  }else{
    rightHeadMotor.setSpeed(0);
    rightHeadMotor.run(BRAKE);  
    rightTailMotor.setSpeed(0);  
    rightTailMotor.run(BRAKE);
    leftHeadMotor.setSpeed(0);  
    leftHeadMotor.run(BRAKE);
    leftTailMotor.setSpeed(0);    
    leftTailMotor.run(BRAKE);
  }

  while(Serial.available() > 0){
    protocol.onByte(Serial.read());
  }
}
