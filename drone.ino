#include<Wire.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050

int angle = 0;
int input_Angle = 0;
int angle_pev = 0;
int left = 3;
int right = 4;
int analogAngle = A0;
int pwmleft , pwmright;
int input_Index = 0;

float resultAngle= 0.0;

char input_char[3];
  
void setup() {
  MPUSetUp();
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);
  digitalWrite(left,LOW);
  digitalWrite(right,LOW);
  pwmleft  = pwmright = 5;
}

void loop() {
  // put your main code here, to run repeatedly:
  int angleInput = map(analogRead(analogAngle), 0, 1023, -9, 9);
  
  printOrinentation();  //mpu를 이용하여 각도 계산 
  
  serial_Input();       //serial을 이용해 원하는 각도 입력
  
  PWM(pwmleft,pwmright); 
}

void serial_Input(){
  if(Serial.available()){
    while(Serial.available()){
      input_char[input_Index] = Serial.read();   //char한글자씩 입력 받기에 char형 배열로 받아온다.
      input_Index++;
    }
    input_Angle = atoi(input_char);               //char를 int형으로 변환
  }
  
  for(int i = 0; i <= input_Index; i++){
    input_char[i] = NULL;
  }
  input_Index = 0;
}

//MPU셋팅
void MPUSetUp(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);  
}

//좌 우 모터 pwm 제어
void PWM(int leftdelay,int rightdelay){
  Serial.print(leftdelay);
  Serial.print("/");
  Serial.println(rightdelay);
  if(angle > input_Angle){
    pwmleft -= 1;
    pwmright += 1; 
  }else if(angle < input_Angle){
    pwmleft += 1;
    pwmright -= 1;
  }

  if(pwmleft < 1) pwmleft = 1;
  if(pwmleft > 9) pwmleft = 9;
  if(pwmright < 1)  pwmright = 1;
  if(pwmright > 9)  pwmright = 9;
  
  motorCycle(left, pwmleft);
  motorCycle(right, pwmright);
  
  Serial.print("left : ");
  Serial.println(leftdelay);
  Serial.print("right : ");
  Serial.println(rightdelay);
  angle_pev = angle;
}

// 모터제어
void motorCycle(int angles,int delayTime){
  digitalWrite(angles, HIGH);
  delay(delayTime);
  digitalWrite(angles, LOW);
  delay(50 - delayTime);
}

