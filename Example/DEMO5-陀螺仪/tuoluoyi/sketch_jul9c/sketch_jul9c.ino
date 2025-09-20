#include <Wire.h>


#define LSM6DSO_ADDR       0x6B   // I²C 地址 (SDO接地)
#define LSM6DSO_CTRL1_XL   0x10U  // 加速度计控制寄存器
#define LSM6DSO_CTRL2_G    0x11U  // 陀螺仪控制寄存器

MbedI2C myWire(2,3);

struct SensorData {
  int16_t gyroX, gyroY, gyroZ;
  int16_t accelX, accelY, accelZ;
};


const float ROTATION_THRESHOLD = 100.0; // 转动检测阈值 (dps)
enum RotationState { NONE, LEFT, RIGHT };
RotationState lastRotationState = NONE;
unsigned long lastRotationTime = 0;
const unsigned long ROTATION_DEBOUNCE = 300; // 防抖时间(ms)

void setup() {
  Serial.begin(115200);
  while (!Serial); // 等待串口连接
  
  
  myWire.begin(); 
  Serial.println("系统已就绪，等待旋转检测...");
  
  initializeLSM6DSO();
  
  Serial.println("系统已就绪，等待旋转检测...");
}

void loop() {
  SensorData data = readSensorData();
  
  // 检测旋转方向
  RotationState currentState = detectRotation(data.gyroZ);
  
  // 显示方向变化
  if (currentState != NONE) {
    if (currentState != lastRotationState || 
        (millis() - lastRotationTime) > ROTATION_DEBOUNCE) {
      displayRotation(currentState);
      lastRotationState = currentState;
      lastRotationTime = millis();
    }
  }
  
  
  delay(50); // 50ms采样间隔
}

// 检测旋转方向
RotationState detectRotation(int16_t gyroZ) {
  // 单位转换：原始值 -> 度/秒 (dps)
  // 1000dps量程下灵敏度 = 35 mdps/LSB
  float dps = gyroZ * 0.035f;
  
  // 判断方向
  if (dps > ROTATION_THRESHOLD) {
    return LEFT;
  } else if (dps < -ROTATION_THRESHOLD) {
    return RIGHT;
  }
  return NONE;
}

// 显示旋转方向
void displayRotation(RotationState state) {
  switch(state) {
    case LEFT:
      Serial.println("检测到：向左转动");
      break;
    case RIGHT:
      Serial.println("检测到：向右转动");
      break;
    default:
      break;
  }
}

// 初始化传感器
void initializeLSM6DSO() {
  // 配置加速度计: 104Hz, ±4g
  writeI2CRegister(LSM6DSO_ADDR, LSM6DSO_CTRL1_XL, 0x4C);
  
  // 配置陀螺仪: 104Hz, 1000dps (关注Z轴旋转)
  writeI2CRegister(LSM6DSO_ADDR, LSM6DSO_CTRL2_G, 0x54);
  
  
  uint8_t id = readI2CRegister(LSM6DSO_ADDR, 0x0F);
  Serial.print("传感器ID: 0x"); Serial.println(id, HEX);
}

// 读取传感器数据
SensorData readSensorData() {
  SensorData data;
  
  // 读取加速度数据
  readRegisterMulti(LSM6DSO_ADDR, 0x28, (uint8_t*)&data.accelX, 6);
  
  // 读取陀螺仪数据 
  readRegisterMulti(LSM6DSO_ADDR, 0x22, (uint8_t*)&data.gyroX, 6);
  
  return data;
}


void readRegisterMulti(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len) {
  myWire.beginTransmission(devAddr);
  myWire.write(regAddr);
  myWire.endTransmission(false); // 保持连接
  
  myWire.requestFrom(devAddr, len);
  
  for (uint8_t i = 0; i < len; i++) {
    data[i] = myWire.read();
  }
}

// 单寄存器写入
void writeI2CRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value) {
  myWire.beginTransmission(devAddr);
  myWire.write(regAddr);
  myWire.write(value);
  myWire.endTransmission();
}

// 单寄存器读取
uint8_t readI2CRegister(uint8_t devAddr, uint8_t regAddr) {
  myWire.beginTransmission(devAddr);
  myWire.write(regAddr);
  myWire.endTransmission(false);
  myWire.requestFrom(devAddr, 1);
  return myWire.read();
}


void printSensorData(SensorData data) {
  // 加速度计数据处理 (±4g量程)
  float ax = data.accelX * 0.000122f; 
  float ay = data.accelY * 0.000122f;
  float az = data.accelZ * 0.000122f;
  
  // 陀螺仪数据处理 (1000dps量程)
  float gx = data.gyroX * 0.035f;
  float gy = data.gyroY * 0.035f;
  float gz = data.gyroZ * 0.035f;
  
  Serial.print("Acc: X:");
  Serial.print(ax, 3); Serial.print("g ");
  Serial.print("Y:"); Serial.print(ay, 3); Serial.print("g ");
  Serial.print("Z:"); Serial.print(az, 3); Serial.print("g\t");
  
  Serial.print("Gyro: X:");
  Serial.print(gx, 0); Serial.print("° ");
  Serial.print("Y:"); Serial.print(gy, 0); Serial.print("° ");
  Serial.print("Z:"); Serial.print(gz, 0); Serial.print("°");
  
  Serial.println();
}