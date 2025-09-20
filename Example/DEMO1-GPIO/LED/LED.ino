#define LED 23  //定义LED灯的管脚为pin23

void setup() {
  pinMode(LED, OUTPUT); //设置pin23为输出
}

void loop() {
   digitalWrite(LED, LOW);   //将pin23置低电平                  
}

