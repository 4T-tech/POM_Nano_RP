

void pwm(){
digitalWrite(19,HIGH);
delay(10);
digitalWrite(19,LOW);
delay(10);
}

void setup() {
  pinMode(19, OUTPUT); //设置pin23为输出
}


void loop() {
   pwm();   //将pin23置低电平                  
}