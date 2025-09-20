#include <Wire.h>

#define LSM6DSO_ADDR       0x6B  // SDO 引脚接地时的地址 (0x6B 如果接VCC)
#define LSM6DSO_CTRL1_XL   0x10U // 加速度计控制寄存器
#define LSM6DSO_CTRL2_G    0x11U // 陀螺仪控制寄存器

MbedI2C myWire(2,3);

struct GyroData {
  int16_t gyroX, gyroY, gyroZ;
  int16_t accelX, accelY, accelZ;
};

void setup() {
  Serial.begin(115200);
  while (!Serial);  
  
 
  myWire.begin(); 
  
  // 初始化陀螺仪传感器
  Serial.println("Initializing IMU...");
  initializeLSM6DSO();
  Serial.println("IMU ready!");
}

void loop() {
  GyroData data = readSensorData();
  

  Serial.print("Gyro X:"); Serial.print(data.gyroX);
  Serial.print(" Y:"); Serial.print(data.gyroY);
  Serial.print(" Z:"); Serial.print(data.gyroZ);
  
  Serial.print("  Accel X:"); Serial.print(data.accelX);
  Serial.print(" Y:"); Serial.print(data.accelY);
  Serial.print(" Z:"); Serial.println(data.accelZ);
  
  delay(10000);  // 控制输出频率
}

void initializeLSM6DSO() {
  // 唤醒加速度计 - 设置 104Hz 输出率，±4g 量程
  writeI2CRegister(LSM6DSO_ADDR, LSM6DSO_CTRL1_XL, 0x4C);
  
  // 配置陀螺仪 - 设置 104Hz 输出率，1000dps 量程
  writeI2CRegister(LSM6DSO_ADDR, LSM6DSO_CTRL2_G, 0x54);
  
  
}

GyroData readSensorData() {
  GyroData data;
  
  // 从加速度计读取 6 字节数据 (X,Y,Z 各 2 字节)
  myWire.beginTransmission(LSM6DSO_ADDR);
  myWire.write(0x28);  // 加速度计数据起始寄存器
  myWire.endTransmission(false); // 保持连接以读取
  myWire.requestFrom(LSM6DSO_ADDR, 6);
  while (myWire.available() < 6);
  
  data.accelX = myWire.read() | myWire.read() << 8;
  data.accelY = myWire.read() | myWire.read() << 8;
  data.accelZ = myWire.read() | myWire.read() << 8;
  
  // 从陀螺仪读取 6 字节数据 (X,Y,Z 各 2 字节)
  myWire.beginTransmission(LSM6DSO_ADDR);
  myWire.write(0x22);
  myWire.endTransmission(false);
  
  myWire.requestFrom(LSM6DSO_ADDR, 6);
  while (myWire.available() < 6);
  
  data.gyroX = myWire.read() | myWire.read() << 8;
  data.gyroY = myWire.read() | myWire.read() << 8;
  data.gyroZ = myWire.read() | myWire.read() << 8;
  
  return data;
}

void writeI2CRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value) {
  myWire.beginTransmission(devAddr);
  myWire.write(regAddr);
  myWire.write(value);
  myWire.endTransmission();
}

uint8_t readI2CRegister(uint8_t devAddr, uint8_t regAddr) {
  myWire.beginTransmission(devAddr);
  myWire.write(regAddr);
  myWire.endTransmission(false);
  myWire.requestFrom(devAddr, 1);
  return myWire.read();
}
