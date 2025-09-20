
int counter=100000;//定义时间
int a=0;//控制LED灯模式
int t=0;


void tim(){
 
  if(t<counter){
    t++;  
  }else {
    t=0;
    a =!a; //时间结束，模式切换
  }

}

void setup() {
 
  pinMode(23, OUTPUT);//管脚23设为输出

}


void loop() {
  tim();
  if (a<1){
   digitalWrite(23, HIGH); //管脚23高电平
  }else{
   digitalWrite(23, LOW);  //管脚23低电平
  }
    
}