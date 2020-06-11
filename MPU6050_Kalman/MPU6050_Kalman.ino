#include <Wire.h>

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

float previous_estimate_Gx = 0.0;
float previous_estimate_error_Gx = 4;


float current_measure_Gx;
float current_measure_error_Gx = 10;

float current_estimate_Gx;

float Kalman_Gain_Gx;

void setup() {
  Serial.begin(9600);
  Wire.begin(); //
  setupMPU();
  Serial.println("Setup Completed");
  }


void loop() {
  recordAccelRegisters();
  recordGyroRegisters();

  Kalman_Gain_Gx = previous_estimate_error_Gx / (previous_estimate_error_Gx + current_measure_error_Gx);

  current_estimate_Gx = previous_estimate_Gx + (Kalman_Gain_Gx * (rotX - previous_estimate_Gx));

  previous_estimate_error_Gx = (1 - Kalman_Gain_Gx) * previous_estimate_error_Gx;
  previous_estimate_error_Gx = previous_estimate_error_Gx + 0.5;
  Serial.print(current_estimate_Gx);
  Serial.print('\t');
  Serial.println(rotX);
  //printData();
  delay(100);
  previous_estimate_Gx = current_estimate_Gx;
}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = (gyroX / 131.0) + 1.57;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

void printData() {
  //Serial.print("Gyro (deg)");
  //Serial.print(" X=");
  Serial.println(rotX);
 // Serial.print(" Y=");
 // Serial.println(rotY);
 // Serial.print(" Z=");
 // Serial.println(rotZ);
 // Serial.print(" Accel (g)");
 // Serial.print(" X=");
  //Serial.print(gForceX);
 // Serial.print(" Y=");
 // Serial.print(gForceY);
 // Serial.print(" Z=");
 // Serial.println(gForceZ);
}
