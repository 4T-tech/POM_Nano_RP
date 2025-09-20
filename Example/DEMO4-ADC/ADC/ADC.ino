#include <Arduino.h>
#include "hardware/adc.h"

// 配置参数
const uint INPUT_PIN = 26;         // ADC输入引脚(GPIO26)
const uint SAMPLE_RATE = 50000;    // 采样率(Hz)
const uint BAUD_RATE = 115200;     // 串口波特率

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial);  // 等待串口连接
  
  Serial.println("Pico ADC Demo");
  
  adc_init(); // 初始化ADC硬件
  
  adc_gpio_init(INPUT_PIN); // 配置ADC引脚

  adc_select_input(0);  // 选择通道0(GPIO26)
  
  adc_set_clkdiv(48000000.0f / SAMPLE_RATE - 1);// 配置时钟分频因子 (48MHz / SAMPLE_RATE - 1)
  
  Serial.println("ADC initialized");
  Serial.println("Press: 1=Single read, 2=FIFO read, 3=Temp sensor");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case '1': // 单次读取模式
        singleRead();
        break;
        
      case '2': // FIFO连续读取模式
        fifoRead();
        break;
        
      case '3': // 读取温度传感器
        readTemperature();
        break;
    }
  }
}

// 单次采样模式
void singleRead() {
  uint16_t value = adc_read();
  float voltage = value * 3.3f / 4096;  // 转换为电压(3.3V参考电压)
  
  Serial.print("Single Read: ");
  Serial.print(value);
  Serial.print(" (");
  Serial.print(voltage, 3);
  Serial.println("V)");
}

// 连续采样模式(FIFO)
void fifoRead() {
  Serial.println("Starting FIFO mode. Press any key to stop");
  
  adc_fifo_setup(true, true, 1, false, false);

  adc_run(true);
  
  uint sample_count = 0;
  unsigned long start_time = millis();
  
  while (!Serial.available()) {
    if (!adc_fifo_is_empty()) {
      uint16_t value = adc_fifo_get();
      sample_count++;
      
      // 每100个样本打印一次
      if (sample_count % 100 == 0) {
        float voltage = value * 3.3f / 4096;
        Serial.print("Sample ");
        Serial.print(sample_count);
        Serial.print(": ");
        Serial.print(value);
        Serial.print(" (");
        Serial.print(voltage, 3);
        Serial.println("V)");
      }
    }
  }
  
  adc_run(false);
  adc_fifo_drain();
  
  // 计算实际采样率
  float duration = (millis() - start_time) / 1000.0f;
  float actual_rate = sample_count / duration;
  
  Serial.print("Stopped. Samples: ");
  Serial.print(sample_count);
  Serial.print(" | Actual rate: ");
  Serial.print(actual_rate);
  Serial.println(" Hz");
  Serial.read();  // 清除缓冲区
}

// 读取内部温度传感器
void readTemperature() {

  adc_set_temp_sensor_enabled(true);
  adc_select_input(4);  // 温度传感器通道
  
  // 取多次读数平均
  const int SAMPLES = 10;
  uint32_t total = 0;
  
  for (int i = 0; i < SAMPLES; i++) {
    total += adc_read();
    delay(10);
  }
  
  // 计算电压值
  float voltage = (total / (float)SAMPLES) * 3.3f / 4096;
  
  // 计算温度(使用头文件中的公式)
  float tempC = 27.0f - (voltage - 0.706f) / 0.001721f;
  
  Serial.print("Temp Sensor: ");
  Serial.print(tempC, 1);
  Serial.println("°C");
  
  // 禁用温度传感器节省功耗
  adc_set_temp_sensor_enabled(false);
}