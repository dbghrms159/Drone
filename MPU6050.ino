#define dt 0.02
#define RAD 180 /3.14139

int index = 7;
int filter[7];
int cpy[7];

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float angles[6];
float target_angle = 0.0;
float Kp = 1.0, Ki= 0.0 , Kd = 0.5;
float inte, prev;
float pidDT ;
float t_new = 0.0, t_prev = millis();

void  printOrinentation() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  angles[0] = (float)GyX/131.072*dt;
  angles[1] = (float)GyY/131.072*dt;
  //angles[2] = (float)GyZ/131.072*dt;
  angles[3] = (float)atan(AcY/sqrt(pow(AcX,2)+pow(AcZ,2)))*RAD;
  angles[4] = (float)atan(AcX/sqrt(pow(AcY,2)+pow(AcZ,2)))*RAD;
  //angles[5] = (float)GyX/131.072*dt;
 
  angle = (0.96*(angle+angles[0]))+(0.04*angles[3]);
  for(int a = index-1; a > 0; a--){
    cpy[a] = filter[a] = filter[a-1];
  }
  filter[0] = cpy[0] = angle;
  posSort(cpy);
  angle = filter[3]= cpy[3];
  
  t_new = millis();
  pidDT = (t_new - t_prev)/1000.0;
  t_prev = t_new;
  
  PID(target_angle,(float)angle,prev,inte,resultAngle);
  Serial.println(resultAngle);
  
}

void PID(float& set, float input,float& prev, float& i, float& out){
  float error;
  float dInput;
  float p, d;

  error = set - input;
  dInput = input - prev;
  prev = input;

  p = Kp * error;
  i += Ki * error * pidDT;
  d = Kd * (dInput / pidDT);
  out = p + i + d;
}

void posSort(int sorts[]){
  for(int i= 0;i<index-1;i++)
    for(int j = 1; j< index-i; j++)
      if(sorts[j-1]>sorts[j]){
        int cpy= sorts[j-1];
        sorts[j-1] = sorts[j];
        sorts[j] = cpy;
      }  
}
