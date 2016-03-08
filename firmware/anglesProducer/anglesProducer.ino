#include<Wire.h>
#include <math.h> 

#include "CircularBuffer.h"
#include "Protocol.h"

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADXL345_ADDRESS                 (0x53)    // Assumes ALT address pin low
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_REG_DEVID               (0x00)    // Device ID
    #define ADXL345_REG_THRESH_TAP          (0x1D)    // Tap threshold
    #define ADXL345_REG_OFSX                (0x1E)    // X-axis offset
    #define ADXL345_REG_OFSY                (0x1F)    // Y-axis offset
    #define ADXL345_REG_OFSZ                (0x20)    // Z-axis offset
    #define ADXL345_REG_DUR                 (0x21)    // Tap duration
    #define ADXL345_REG_LATENT              (0x22)    // Tap latency
    #define ADXL345_REG_WINDOW              (0x23)    // Tap window
    #define ADXL345_REG_THRESH_ACT          (0x24)    // Activity threshold
    #define ADXL345_REG_THRESH_INACT        (0x25)    // Inactivity threshold
    #define ADXL345_REG_TIME_INACT          (0x26)    // Inactivity time
    #define ADXL345_REG_ACT_INACT_CTL       (0x27)    // Axis enable control for activity and inactivity detection
    #define ADXL345_REG_THRESH_FF           (0x28)    // Free-fall threshold
    #define ADXL345_REG_TIME_FF             (0x29)    // Free-fall time
    #define ADXL345_REG_TAP_AXES            (0x2A)    // Axis control for single/double tap
    #define ADXL345_REG_ACT_TAP_STATUS      (0x2B)    // Source for single/double tap
    #define ADXL345_REG_BW_RATE             (0x2C)    // Data rate and power mode control
    #define ADXL345_REG_POWER_CTL           (0x2D)    // Power-saving features control
    #define ADXL345_REG_INT_ENABLE          (0x2E)    // Interrupt enable control
    #define ADXL345_REG_INT_MAP             (0x2F)    // Interrupt mapping control
    #define ADXL345_REG_INT_SOURCE          (0x30)    // Source of interrupts
    #define ADXL345_REG_DATA_FORMAT         (0x31)    // Data format control
    #define ADXL345_REG_DATAX0              (0x32)    // X-axis data 0
    #define ADXL345_REG_DATAX1              (0x33)    // X-axis data 1
    #define ADXL345_REG_DATAY0              (0x34)    // Y-axis data 0
    #define ADXL345_REG_DATAY1              (0x35)    // Y-axis data 1
    #define ADXL345_REG_DATAZ0              (0x36)    // Z-axis data 0
    #define ADXL345_REG_DATAZ1              (0x37)    // Z-axis data 1
    #define ADXL345_REG_FIFO_CTL            (0x38)    // FIFO control
    #define ADXL345_REG_FIFO_STATUS         (0x39)    // FIFO status
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_MG2G_MULTIPLIER (0.004)  // 4mg per lsb
/*=========================================================================*/

/* Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth */
typedef enum
{
  ADXL345_DATARATE_3200_HZ    = 0b1111, // 1600Hz Bandwidth   140<B5>A IDD
  ADXL345_DATARATE_1600_HZ    = 0b1110, //  800Hz Bandwidth    90<B5>A IDD
  ADXL345_DATARATE_800_HZ     = 0b1101, //  400Hz Bandwidth   140<B5>A IDD
  ADXL345_DATARATE_400_HZ     = 0b1100, //  200Hz Bandwidth   140<B5>A IDD
  ADXL345_DATARATE_200_HZ     = 0b1011, //  100Hz Bandwidth   140<B5>A IDD
  ADXL345_DATARATE_100_HZ     = 0b1010, //   50Hz Bandwidth   140<B5>A IDD
  ADXL345_DATARATE_50_HZ      = 0b1001, //   25Hz Bandwidth    90<B5>A IDD
  ADXL345_DATARATE_25_HZ      = 0b1000, // 12.5Hz Bandwidth    60<B5>A IDD
  ADXL345_DATARATE_12_5_HZ    = 0b0111, // 6.25Hz Bandwidth    50<B5>A IDD
  ADXL345_DATARATE_6_25HZ     = 0b0110, // 3.13Hz Bandwidth    45<B5>A IDD
  ADXL345_DATARATE_3_13_HZ    = 0b0101, // 1.56Hz Bandwidth    40<B5>A IDD
  ADXL345_DATARATE_1_56_HZ    = 0b0100, // 0.78Hz Bandwidth    34<B5>A IDD
  ADXL345_DATARATE_0_78_HZ    = 0b0011, // 0.39Hz Bandwidth    23<B5>A IDD
  ADXL345_DATARATE_0_39_HZ    = 0b0010, // 0.20Hz Bandwidth    23<B5>A IDD
  ADXL345_DATARATE_0_20_HZ    = 0b0001, // 0.10Hz Bandwidth    23<B5>A IDD
  ADXL345_DATARATE_0_10_HZ    = 0b0000  // 0.05Hz Bandwidth    23<B5>A IDD (default value)
} dataRate_t;

/* Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range */
typedef enum
{
  ADXL345_RANGE_16_G          = 0b11,   // +/- 16g
  ADXL345_RANGE_8_G           = 0b10,   // +/- 8g
  ADXL345_RANGE_4_G           = 0b01,   // +/- 4g
  ADXL345_RANGE_2_G           = 0b00    // +/- 2g (default value)
} range_t;

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define HMC5883_ADDRESS_MAG            (0x3C >> 1)         // 0011110x
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      HMC5883_REGISTER_MAG_CRA_REG_M             = 0x00,
      HMC5883_REGISTER_MAG_CRB_REG_M             = 0x01,
      HMC5883_REGISTER_MAG_MR_REG_M              = 0x02,
      HMC5883_REGISTER_MAG_OUT_X_H_M             = 0x03,
      HMC5883_REGISTER_MAG_OUT_X_L_M             = 0x04,
      HMC5883_REGISTER_MAG_OUT_Z_H_M             = 0x05,
      HMC5883_REGISTER_MAG_OUT_Z_L_M             = 0x06,
      HMC5883_REGISTER_MAG_OUT_Y_H_M             = 0x07,
      HMC5883_REGISTER_MAG_OUT_Y_L_M             = 0x08,
      HMC5883_REGISTER_MAG_SR_REG_Mg             = 0x09,
      HMC5883_REGISTER_MAG_IRA_REG_M             = 0x0A,
      HMC5883_REGISTER_MAG_IRB_REG_M             = 0x0B,
      HMC5883_REGISTER_MAG_IRC_REG_M             = 0x0C,
      HMC5883_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
      HMC5883_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
    } hmc5883MagRegisters_t;
/*=========================================================================*/

/*=========================================================================
    MAGNETOMETER GAIN SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      HMC5883_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
      HMC5883_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
      HMC5883_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
      HMC5883_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
      HMC5883_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
      HMC5883_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
      HMC5883_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
    } hmc5883MagGain;   
/*=========================================================================*/



#define ADXL345_INT0_PIN 7

enum{
  ADXL345_FIFO_SIZE = 30
};

uint8_t ADXL345_buf[sizeof(ElemType) * (ADXL345_FIFO_SIZE)];
CircularBuffer ADXL345_circularBuffer;
uint16_t gAccelDrops = 0;

enum RegMap{
  //READ_ONLY_REGS  
  ROLL = 10,
  PITCH = 11,
  YAW = 12
};

void initAccel(){
  //Setup accel
  //place ADXL345 to standby mode
  writeSingleRegisterI2C(ADXL345_ADDRESS, ADXL345_REG_POWER_CTL, 0);
  
  //disable ADXL345 interrupts
  writeSingleRegisterI2C(ADXL345_ADDRESS, ADXL345_REG_INT_ENABLE, 0);

  //set ADXL345 data rate to 200Hz (100Hz) bandwidth, normal power mode
  writeSingleRegisterI2C(ADXL345_ADDRESS, ADXL345_REG_BW_RATE, ADXL345_DATARATE_200_HZ);
  
  //setup ADXL345 format (10bit-res, 2g range)
  writeSingleRegisterI2C(ADXL345_ADDRESS, ADXL345_REG_DATA_FORMAT, 0b00000000);

  //configuring FIFO stream mode, watermark to 1 samples in FIFO (10ms)
  writeSingleRegisterI2C(ADXL345_ADDRESS, ADXL345_REG_FIFO_CTL, 
      /* Stream mode*/ 0b100 << 5 | /*Number of samples to trigger interrupt*/ ADXL345_FIFO_SIZE);       
 
  //map ADXL345 watermark to INT1, other INT2
  writeSingleRegisterI2C(ADXL345_ADDRESS, ADXL345_REG_INT_MAP, 0b11111101);   
  
  //enable ADXL345 measurement, disable sleep
  writeSingleRegisterI2C(ADXL345_ADDRESS, ADXL345_REG_POWER_CTL, 0b00001000);

  //setup fifo watermark signal
  pinMode(ADXL345_INT0_PIN, INPUT);
  
  //enable ADXL345 interrupts
  writeSingleRegisterI2C(ADXL345_ADDRESS, ADXL345_REG_INT_ENABLE, 0b10);
}

void initMagnetic(){  
  writeSingleRegisterI2C(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_CRA_REG_M, 0b01111000);

  writeSingleRegisterI2C(HMC5883_ADDRESS_MAG,HMC5883_REGISTER_MAG_CRB_REG_M, HMC5883_MAGGAIN_1_3); 

  writeSingleRegisterI2C(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_MR_REG_M, 0x00);
}

void setup() {
  cbInit(ADXL345_circularBuffer, &ADXL345_buf[0], sizeof(ADXL345_buf) / sizeof(ADXL345_buf[0]));
  
  // put your setup code here, to run once:
  Serial.begin(56700);
  Wire.begin(); 

  initAccel();
  initMagnetic();
}

void ADXL345_receiveData(){
  uint8_t drops = readToRingBuffer(ADXL345_ADDRESS, ADXL345_REG_DATAX0, ADXL345_FIFO_SIZE, ADXL345_circularBuffer);
  if(drops > 0){
     Serial.print("drops:"); Serial.println(drops);
  }
}

void HMC5883_receiveData(ElemType* xyz){
  Wire.beginTransmission((byte)HMC5883_ADDRESS_MAG);
  Wire.write(HMC5883_REGISTER_MAG_OUT_X_H_M);
  Wire.endTransmission();
  Wire.requestFrom((byte)HMC5883_ADDRESS_MAG, (byte)6);
  while (Wire.available() < 6);
  uint8_t xhi = Wire.read();
  uint8_t xlo = Wire.read();
  uint8_t zhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t ylo = Wire.read();

  xyz->x = (int16_t)(xlo | ((int16_t)xhi << 8));
  xyz->y = (int16_t)(ylo | ((int16_t)yhi << 8));
  xyz->z = (int16_t)(zlo | ((int16_t)zhi << 8));  
  Wire.endTransmission();
}

void writeSingleRegisterI2C(uint8_t device, uint8_t reg, uint8_t value){
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readSingleRegisterI2C(uint8_t device, uint8_t reg){
   Wire.beginTransmission((uint8_t)device);
   Wire.write(reg); 
   Wire.endTransmission();  
   Wire.requestFrom((uint8_t)device, (uint8_t)1);
   while (Wire.available() < 1);
   uint8_t value = Wire.read();
   Wire.endTransmission();

   return value;
}

uint8_t readToRingBuffer(uint8_t device, uint8_t reg, uint8_t samples, CircularBuffer& cb ){
   uint16_t drops = 0;
   
   for(int i = 0; i < samples; ++i){     
     uint16_t x = readDoubleByte(device, reg);
     uint16_t y = readDoubleByte(device, reg + 2);
     uint16_t z = readDoubleByte(device, reg + 4);
     //Serial.println((int16_t)x); //Serial.print(" "); Serial.print(y); Serial.print(" "); Serial.println(z);
    //delayMicroseconds(5);
     if(cbGetSpace(cb) > 0){
       ElemType* next = (ElemType*)getWritePtr(cb);
       next->x = x;
       next->y = y;
       next->z = z;
       confirmWrite(cb, sizeof(ElemType));
     }else{
       drops++;
     }        
  }

  return drops;
}

uint16_t readDoubleByte(uint8_t device, uint8_t reg){
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(device, (uint8_t)2);
  while (Wire.available() < 2 );
  uint16_t value = (uint16_t)(Wire.read() | (Wire.read() << 8));
  Wire.endTransmission();

  return value;
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

bool accelFifoWatermark(){
  return digitalRead(ADXL345_INT0_PIN);
}

double xAccel = 0, yAccel = 0, zAccel = 0;
double roll = 0, pitch = 0, yaw = 0;

double rollFiltered = 0, pitchFiltered = 0, yawFiltered = 0;

#define TO_DEG(x) ((x) * 180 / 3.14159265359)

void loop() {
  if(accelFifoWatermark()){    
    ADXL345_receiveData();    
  }
      
  if(cbGetLength(ADXL345_circularBuffer) > 0){    
    while(cbGetLength(ADXL345_circularBuffer) > 0){
      ElemType* accelNext = (ElemType*)cbGetReadPtr(ADXL345_circularBuffer);      
    
      xAccel = xAccel * 0.7 + 0.3 * accelNext->x;
      yAccel = yAccel * 0.7 + 0.3 * accelNext->y;
      zAccel = zAccel * 0.7 + 0.3 * accelNext->z;
          
      confirmRead(ADXL345_circularBuffer, sizeof(ElemType));
    }
    
    ElemType magnetic;
    HMC5883_receiveData(&magnetic);

    roll = atan2(yAccel,zAccel);
    pitch = atan2(-xAccel, sqrt(yAccel*yAccel + zAccel*zAccel));      
    
    double xMag = (double)magnetic.x / 1100.0 * 100;
    double yMag = (double)magnetic.y / 1100.0 * 100;
    double zMag = (double)magnetic.z / 980.0 * 100;

    xMag = xMag * cos(pitch) + yMag * sin(roll) * sin(pitch) + zMag * cos(roll) * sin(pitch);
    yMag = yMag * cos(roll) - zMag * sin(roll);

    yaw = atan2( yMag, xMag);

    rollFiltered = 0.9 * rollFiltered + 0.1 * roll;
    pitchFiltered = 0.9 * pitchFiltered + 0.1 * pitch;
    yawFiltered = 0.9 * yawFiltered + 0.1 * yaw;
       
    protocol.writeRegister( ROLL, rollFiltered * 180 / 3.14159265359);
    protocol.writeRegister( PITCH, pitchFiltered * 180 / 3.14159265359 );
    protocol.writeRegister( YAW, yawFiltered * 180 / 3.14159265359);
  }  
}
