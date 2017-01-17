#include <Wire.h>
#include <String.h>
#define DEBUG true


//Registry adresses for accelerometer, SAD+read 00110011, SAD+write 00110010.

#define ACC_SAD 0b00011001

#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define CTRL_REG6_A 0x25

#define REFERENCE_A 0x26
#define STATUS_REG_A 0x27

#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D

#define FIFO_CTRL_REG_A 0x2E//default FIFO disabled
#define FIFO_SRC_REG_A 0x30

//used for interupts, not liekly to be used
#define INT1_CFG_A 0x31
#define INT1_SOURCE_A 0x32
#define INT1_THS_A 0x33
#define INT2_CFG_A 0x34
#define INT2_SOURCE_A 0x35
#define INT2_THS_A 0x36
#define INT2_DURATION_A 0x37
#define CLICK_CFG_A 0x38
#define CLICK_SRC_A 0x39
#define CLICK_THS_A 0x3A
#define TIME_LIMIT_A 0x3B
#define TIME_LATENCY_A 0x3C
#define TIME_WINDOW_A 0x3D

//Registry adresses for magnometer, SAD+read 00111101, SAD+write 00111100.

#define MAG_SAD 0b00011110

#define CRA_REG_M 0x00
#define CRB_REG_M 0x01
#define MR_REG_M 0x02
#define OUT_X_H_M 0x03
#define OUT_X_L_M 0x04
#define OUT_Z_H_M 0x05
#define OUT_Z_L_M 0x06
#define OUT_Y_H_M 0x07
#define OUT_Y_L_M 0x08
#define SR_REG_Mg 0x09
#define IRA_REG_M 0x0A
#define IRB_REG_M 0x0B
#define IRC_REG_M 0x0C

#define TEMP_OUT_H_M 0x31
#define TEMP_OUT_L_M 0x32

//Register adresses for gyroscope
#define GYRO_SAD 0b01101010

#define WHO_AM_I 0x0F

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define REFERANCE 0x25

#define OUT_TEMP 0x26

#define STATUS_REG 0x27

#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

#define FIFO_CTRL_REG//default FIFO disabled?
#define FIFO_SRC_REG

#define INT1_CFG
#define INT1_SRC
#define INT1_TSH_XH
#define INT1_TSH_XL
#define INT1_TSH_YH
#define INT1_TSH_YL
#define INT1_TSH_ZH
#define INT1_TSH_ZL
#define INT1_DURATION

#define ACC_POL_RATE
#define MAG_POL_RATE
#define GYRO_POL_RATE


#define accSetup 0b01000111//100Hz, normal, all axis on
#define magSetup 0b10011000//temp on, sensor rate 75hz
#define gyroSetup 0b01111111//ODR 190hz, power on, all axis on, cut off 25


//arrays to hold the data
int accData[3];
int magData[3];
int gyroData[3];
void setup()
{


  Wire.begin();

  write(MAG_SAD, CRA_REG_M, magSetup);
  write(ACC_SAD, CTRL_REG1_A, accSetup);
  write(GYRO_SAD, CTRL_REG1, gyroSetup);
  #if DEBUG
    Serial.begin(9600);
  #endif
}


void loop()
{ 
  //get the data and reformat into int array
  { 
    String tempData;
    read(ACC_SAD, OUT_X_L_A, 6, tempData);
    accData[0]=((((int) tempData[1])<<8)|((int) tempData[0]));
    accData[1]=((((int) tempData[3])<<8)|((int) tempData[2]));
    accData[2]=((((int) tempData[5])<<8)|((int) tempData[4]));

    read(MAG_SAD, OUT_X_H_M, 6, tempData);
    magData[0]=((((int) tempData[0])<<8)|((int) tempData[1]));
    magData[1]=((((int) tempData[2])<<8)|((int) tempData[3]));
    magData[2]=((((int) tempData[4])<<8)|((int) tempData[5]));

    read(GYRO_SAD, OUT_X_L, 6, tempData);
    gyroData[0]=((((int) tempData[1])<<8)|((int) tempData[0]));
    gyroData[1]=((((int) tempData[3])<<8)|((int) tempData[2]));
    gyroData[2]=((((int) tempData[5])<<8)|((int) tempData[4]));
  }
  #if DEBUG

    Serial.print("ACC X:");
    Serial.print(accData[0]); 
    Serial.print(" ACC Y:"); 
    Serial.print(accData[1]); 
    Serial.print(" ACC Z:");
    Serial.print(accData[2]);
    Serial.println();

    Serial.print("MAG X:");
    Serial.print(magData[0]); 
    Serial.print(" MAG Y:"); 
    Serial.print(magData[1]); 
    Serial.print(" MAG Z:");
    Serial.print(magData[2]);
    Serial.println();

    Serial.print("GYRO X:");
    Serial.print(gyroData[0]); 
    Serial.print(" GYRO Y:"); 
    Serial.print(gyroData[1]); 
    Serial.print(" GYRO Z:");
    Serial.print(gyroData[2]);
    Serial.println();
  #endif
  
}

void write(char SAD, char address, char data)
{
  Wire.beginTransmission(SAD);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

void read(char SAD, char address, int numOfData, String &data)
{
  Wire.beginTransmission(SAD);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(address, numOfData);
  for(int i=0; i < numOfData; i++)
    data[i]=Wire.read();
}




