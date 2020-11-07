// User Settings
const char* ssid = "********-******";
const char* password = "************";
const int user_id = 1; 

#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Dps310.h>

// for MPU6050
#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b

// for dps310 
Dps310 Dps310PressureSensor = Dps310();
const float detectSittingThreshold = 100100.00;

// API url
const char* URL = "http://load-balancer-1-469612606.ap-northeast-1.elb.amazonaws.com/api/concentration_values";

// For Timer
int startTime;
int Time;

// POST interval (ms)
const int postInterval = 10*1000;

// Use for measuring concentration
const float powerMaxThreshold = 2.00;
float offset= 0;
float power = 0;
int count = 0;

// i2c_write
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// i2c_read
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU6050_ADDR, 1/*length*/); 
  byte data =  Wire.read();
  return data;
}

void calcPower() {
  int16_t raw_acc_x, raw_acc_y, raw_acc_z ;
  float acc_x, acc_y, acc_z;
  
  //Get 6bytes data from MPU6050_ADDR
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  
  //Read data and bit shiting
  raw_acc_x = Wire.read() << 8 | Wire.read();
  raw_acc_y = Wire.read() << 8 | Wire.read();
  raw_acc_z = Wire.read() << 8 | Wire.read();
  
  //Change unit to G
  acc_x = ((float)raw_acc_x) / 16384.0;
  acc_y = ((float)raw_acc_y) / 16384.0;
  acc_z = ((float)raw_acc_z) / 16384.0;

  power += ((acc_x * acc_x + acc_y * acc_y + acc_z * acc_z) - offset);
  count++;

//  //For Debug
//  Serial.print(" accelX : ");
//  Serial.print(acc_x);
//  Serial.print(" accelY : ");
//  Serial.print(acc_y);
//  Serial.print(" accelZ : ");
//  Serial.println(acc_z);
}

void setupMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

//  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
//    Serial.println("\nWHO_AM_I error.");
//    while (true) ;
//  }

  // Set parameters
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro

  // define offset
  calcPower();
  offset = power - 1.0;
  power = 0;

//  // For Debug
//  Serial.print(" offset: ");
//  Serial.println(offset);
}

void setupDps310() {
  Dps310PressureSensor.begin(Wire);
}

void setupWiFi() {
//  // For Debug
//  Serial.printf("Connecting to %s", ssid);
//  Serial.println();
  
  while (WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect(true, true);
      WiFi.begin(ssid, password);
      
//      // For Debug
//      Serial.print(".");
      
      delay(3000);
  }

//  // For Debug
//  Serial.println(" CONNECTED");
}

void WiFiReconnection() {
  while (WiFi.status() != WL_CONNECTED) {
      
    WiFi.disconnect(true, true);
    WiFi.begin(ssid, password);

//    // For Debug
//    Serial.print(".");
    
    delay(3000);  
  }
  
//  //For Debug
//  Serial.println(" RECONNECTED");
}

void doPOST(int concentration_value,int is_sitting) {
  HTTPClient http;
  http.begin(URL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String requestBody = "user_id=" + String(user_id)+ "&concentration_value=" + String(concentration_value) + "&is_sitting=" + String(is_sitting);

  // For Debug
  int httpCode = http.POST(requestBody);
  String body = http.getString();

  http.end();

//  // For Debug
//  Serial.print(" requestBody: ");
//  Serial.println(requestBody);
//  Serial.print(" Response: ");
//  Serial.println(httpCode);
//  Serial.println(body);
}

float getPressure() {
  float pressure;
  uint8_t oversampling = 7;
  int16_t ret;

  ret = Dps310PressureSensor.measurePressureOnce(pressure, oversampling);

  if (ret != 0) pressure = -1.0;
  
//  // For Debug
//  Serial.print(" Pressure: ");
//  Serial.print(pressure);
//  Serial.println(" Pascal");

  return pressure;
}

int calcConcentrationValue() {
  float maximumPower = powerMaxThreshold * count;
  float base = (maximumPower - 1.0 * count) / 10.0;
  float difference = maximumPower - power;

  int concentration_value = int(difference / base) + 1;
  
  if (concentration_value < 1) concentration_value = 1;
  else if (concentration_value > 10) concentration_value = 10;

//  //For Debug
//  Serial.print(" power: ");
//  Serial.print(power);
//  Serial.print(" count: ");
//  Serial.println(count);

  power = 0;
  count = 0;

//  //For Debug
//  Serial.print(" maximumPower: ");
//  Serial.print(maximumPower);
//  Serial.print(" base: ");
//  Serial.print(base);
//  Serial.print(" difference: ");
//  Serial.print(difference);
//  Serial.print(" concentration_value: ");
//  Serial.println(concentration_value);

  return concentration_value;
}

int detectSitting() {
  int is_sitting;
  float pressure = getPressure();

  // If not detected sitting or something wrong with sensor, is_sitting = 0.
  if(pressure >= detectSittingThreshold) is_sitting = 1;
  else is_sitting = 0;

//  //For Debug
//  Serial.print(" is_sitting: ");
//  Serial.println(is_sitting);

  return is_sitting;
}

void doDetection() {
  int concentration_value;
  int is_sitting;

  is_sitting = detectSitting();

//  // For Debug (To see how calcConcentrationValue() works)
//  is_sitting = 1;

  // Only if is_sitting = true, send concentration_value > 0.
  if (is_sitting) concentration_value = calcConcentrationValue();
  else concentration_value = 0;

  if ((WiFi.status() != WL_CONNECTED)) WiFiReconnection(); 
  doPOST(concentration_value, is_sitting);
}

void setupTimer() {
  startTime = millis();
}

void Timer() {
  Time = millis();
  if((Time - startTime) > postInterval ) {
    doDetection();
    startTime = millis();
  }
}

void setup() {
//  // For Debug
//  Serial.begin(9600);
//  while (!Serial);
  
  Wire.begin();
  setupWiFi();
  setupMPU6050();
  setupDps310(); 
  setupTimer();

//  // For Debug
//  Serial.println("Setup complete!");
}

void loop() {
  calcPower();
  Timer();
}
