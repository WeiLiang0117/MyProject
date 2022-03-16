#include <Wire.h>
#include <I2Cdev.h>
#include <JJ_MPU6050_DMP_6Axis.h>  // 與DMP工作庫的修改版本（見註釋內）
#define DEBUG 0
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define ZERO_SPEED 65535 //零速
#define MAX_ACCEL 7 //最大ACCEL
#define MAX_THROTTLE 600 //最大油門530
#define MAX_STEERING 136 //最大轉向
#define MAX_TARGET_ANGLE 12 //最大目標角度12
#define I2C_SPEED 400000L //I2C速度
#define Gyro_Gain 0.03048//陀螺增益
#define Gyro_Scaled(x) x*Gyro_Gain //返回每秒度的縮放陀螺儀的原始數據
#define RAD2GRAD 57.2957795//57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489//0.01745329251994329576923690768489

//LQR K值設定
#define LQR_K1  1.0596  // PI*180/ 1.8       //輪子角度比例參數KP的增益值   弧度to角度to步進值
#define LQR_K2  34.1132 // PI*180            //車體角度比例參數KP的增益值   弧度to角度
#define LQR_K3  1.1094  // PI*180/ 1.8       //輪子角度微分參數KD的增益值   弧度to角度to步進值
#define LQR_K4  3.2127  // PI*180            //車體角度微分參數KD的增益值   弧度to角度
#define LQR_K5  0.5099  // PI*180 / 1.8      //輪子角度積分參數KI的增益值   弧度to角度to步進值

#define ITERM_MAX_ERROR 40   // ITERM飽和常數
#define ITERM_MAX 5000
// MPU控制
bool dmpReady = false;  // 設置為true，如果DMP初始化成功
uint8_t mpuIntStatus;   // 擁有從MPU實際中斷狀態字節
uint8_t devStatus;      // 每個設備操作後返回的狀態（0=成功！0=錯誤）
uint16_t packetSize;    // 預計DMP數據包大小（我們18字節）
uint16_t fifoCount;     // 目前在FIFO中的所有字節數
uint8_t fifoBuffer[18]; // FIFO存儲緩衝器
Quaternion q;
uint8_t loop_counter;       // 要生成媒體40Hz的循環
uint8_t slow_loop_counter;  // 慢環2HZ
long timer_old;//計時器老
long timer_value;//定時器的值
int debug_counter;//調試計數器
float dt;
float alltime;
int lkf;
// 類的默認I2C地址為0X68
MPU6050 mpu;
float angle_adjusted;//角度調整
float angle_adjusted_Old;//角度調整

//LQR K值單位轉換成角度 matlab為單位RAD arduino 的phi單位為角度 theta_dot單位為步進值(全步進1:1.8)

float K1 = LQR_K1 * PI/180 / 1.8;        //theta      步進值to角度
float K2 = LQR_K2 * PI/180;              //phi        弧度to角度
float K3 = LQR_K3 * PI/180 / 1.8;        //theta_dot  步進值to角度
float K4 = LQR_K4 * PI/180;              //phi_dot    弧度to角度
float K5 = LQR_K5 * PI/180 / 1.8;        //theta_int  步進值to角度

bool newControlParameters = false;//布爾新的控制參數
bool modifing_control_parameters=false;//布爾新的控制參數

float phi;
float phi_dot;
float theta;
float theta_int;
float theta_dot;
float theta_error;
float theta_errorsum;
float PID_errorSum;
float PI_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
int throttle;
float kkll;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;
float e;
float ec;
float alldt;


int fuzzyeoNB=-21;    //phi規則-21~21角度
int fuzzyeoNM=-14;
int fuzzyeoNS=-7;
int fuzzyeoZO=0;
int fuzzyeoPS=7;
int fuzzyeoPM=14;
int fuzzyeoPB=21;

int fuzzyepNB=-801;    //phi_dot規則-14~14弧度
int fuzzyepNM=-534;
int fuzzyepNS=-267;
int fuzzyepZO=0;
int fuzzyepPS=267;
int fuzzyepPM=534;
int fuzzyepPB=801;


int fuzzyewNB=-399;     //theta規則-0.5~0.5 位移轉換成角度再轉乘步進值 0.5 /W2M / 1.8 = 398
int fuzzyewNM=-266;
int fuzzyewNS=-133;
int fuzzyewZO=0;
int fuzzyewPS=133;
int fuzzyewPM=266;
int fuzzyewPB=399;

float eNBdegree;
float eNMdegree;
float eNSdegree;
float eZOdegree;
float ePSdegree;
float ePMdegree;
float ePBdegree;
float ecNBdegree;
float ecZOdegree;
float ecPBdegree;

float NBKpt=-3.14/1.8;   //-0.8 theta  K1的上下限值 0.2  0.6333
float NMKpt=-2.094/1.8;
float NSKpt=-1.047/1.8;
float ZOKpt=0;
float PSKpt=1.047/1.8;
float PMKpt=2.094/1.8;
float PBKpt=3.14/1.8;  //3

float NBKpo=-14.88;   //-0.7 psi K2的上下限值   0.2333
float NMKpo=-9.923;
float NSKpo=-4.963;
float ZOKpo=0;
float PSKpo=4.963;
float PMKpo=9.923;
float PBKpo=14.88;   //0.7

float NBKdt=-0.3906/1.8;    //-0.2  theta_dot K3的上下限值 35 0.1
float NMKdt=-0.2604/1.8;
float NSKdt=-0.1302/1.8;
float ZOKdt=0;
float PSKdt=0.1302/1.8;
float PMKdt=0.2604/1.8;
float PBKdt=0.3906/1.8;   // 0.4

float NBKdo=-1.308;    //-0.5 psi_dot K4的上下限值  0.2
float NMKdo=-0.8716;
float NSKdo=-0.4358;
float ZOKdo=0;
float PSKdo=0.4358;
float PMKdo=0.8716;
float PBKdo=1.308;   //0.7


float efuzzy;
float K1efuzzy;
float K2efuzzy;
float K3efuzzy;
float K4efuzzy;
float K5efuzzy;


int Sec_Count = 0;
int Status = 0,Status_Pre = 1;
int Flag_Up = 0,Flag_Down = 0;
int count=0;
int mobile=0;
int start=0;

int16_t motor1;
int16_t motor2;
int16_t speed_m[2];           // 電機的實際轉速
uint8_t dir_m[2];             // 步進電機的實際方向
int16_t actual_robot_speed;          // 整個機器人的速度（從踏步機速測）
int16_t actual_robot_speed_Old;          // 整個機器人的速度（從踏步機速測）
float estimated_speed_filtered;//估計速度過濾
uint16_t counter_m[2];        // 計數器週期
uint16_t period_m[2][8];      // 八子時期
uint8_t period_m_index[2];    //索引子時期
int freeRam () { 
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
//DMP功能
//這個函數定義在傳感器融合加速的重量
//默認值是0x80的
//官方InvenSense公司名稱是inv_key_0_96（？）
void dmpSetSensorFusionAccelGain(uint8_t gain) {
  // INV_KEY_0_96
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(gain);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
}

// 快速計算，從四元解obtein披角度
float dmpGetPhi() {
  mpu.getFIFOBytes(fifoBuffer, 16); 
  mpu.dmpGetQuaternion(&q, fifoBuffer); 
  mpu.resetFIFO();  
  //返回( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI); //roll
  return (atan2(2*(q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)* RAD2GRAD+6); //校正後 回傳phi (使用roll尋找)
}

// DT是毫秒
float fuzzyrule(float value,int fuzzyeNB,int fuzzyeNM,int fuzzyeNS,int fuzzyeZO,int fuzzyePS,int fuzzyePM,int fuzzyePB,float NB,float NM,float NS,float ZO,float PS,float PM,float PB) {
  e=value;
  if(e<=fuzzyeNB)
    eNBdegree=1;
  else if(fuzzyeNB<=e && e<=fuzzyeNM)
    eNBdegree=(e-fuzzyeNM)/(fuzzyeNB-fuzzyeNM);
  else if(e>=fuzzyeNM)
    eNBdegree=0;
    
  if(e>=fuzzyeNS || e<=fuzzyeNB)
    eNMdegree=0;
  else if(fuzzyeNB<=e && e<=fuzzyeNM)
    eNMdegree=(e-fuzzyeNB)/(fuzzyeNM-fuzzyeNB);
  else if(fuzzyeNM<=e && e<=fuzzyeNS)
    eNMdegree=(e-fuzzyeNS)/(fuzzyeNM-fuzzyeNS);
    
  if(e>=fuzzyeZO || e<=fuzzyeNM)
    eNSdegree=0;
  else if(fuzzyeNM<=e && e<=fuzzyeNS)
    eNSdegree=(e-fuzzyeNM)/(fuzzyeNS-fuzzyeNM);
  else if(fuzzyeNS<=e && e<=fuzzyeZO)
    eNSdegree=(e-fuzzyeZO)/(fuzzyeNS-fuzzyeZO);
    
  if(e>=fuzzyePS || e<=fuzzyeNS)
    eZOdegree=0;
  else if(fuzzyeNS<=e && e<=fuzzyeZO)
    eZOdegree=(e-fuzzyeNS)/(fuzzyeZO-fuzzyeNS);
  else if(fuzzyeZO<=e && e<=fuzzyePS)
    eZOdegree=(e-fuzzyePS)/(fuzzyeZO-fuzzyePS);
    
  if(e>=fuzzyePM || e<=fuzzyeZO)
    ePSdegree=0;
  else if(fuzzyeZO<=e && e<=fuzzyePS)
    ePSdegree=(e-fuzzyeZO)/(fuzzyePS-fuzzyeZO);
  else if(fuzzyePS<=e && e<=fuzzyePM)
    ePSdegree=(e-fuzzyePM)/(fuzzyePS-fuzzyePM);
    
  if(e>=fuzzyePB || e<=fuzzyePS)
    ePMdegree=0;
  else if(fuzzyePS<=e && e<=fuzzyePM)
    ePMdegree=(e-fuzzyePS)/(fuzzyePM-fuzzyePS);
  else if(fuzzyePM<=e && e<=fuzzyePB)
    ePMdegree=(e-fuzzyePB)/(fuzzyePM-fuzzyePB);

  if(e>=fuzzyePB)
    ePBdegree=1;
  else if(fuzzyePM<=e && e<=fuzzyePB)
    ePBdegree=(e-fuzzyePM)/(fuzzyePB-fuzzyePM);
  else if(e<=fuzzyePM)
    ePBdegree=0;

  efuzzy=(eNBdegree*NB+eNMdegree*NM+eNSdegree*NS+eZOdegree*ZO+ePBdegree*PB+ePMdegree*PM+ePSdegree*PS)  /  (eNBdegree+eNMdegree+eNSdegree+eZOdegree+ePBdegree+ePMdegree+ePSdegree);
  return(efuzzy);
}

float stabilityLQRControl(float DT, float input, float setPoint,  float k1, float k2, float k3,float k4,float k5) {
  float error;
  float output;
  error = setPoint-input;  
  PI_errorSum += constrain(error,-ITERM_MAX_ERROR,ITERM_MAX_ERROR);  //constrain將數字限制在一個範圍內。(-40~error~40)
  PI_errorSum = constrain(PI_errorSum,-ITERM_MAX,ITERM_MAX);        //(-5000~PID_errorSum~5000)
  theta_int = PI_errorSum*dt*0.001;
  // Kd分兩部分實施
  // 最大的一個只使用輸入（傳感器）部分而不是 SetPoint input-input(t-2)
  // 第二個使用設定點使其更具侵略性 setPoint-setPoint(t-1)  actual_robot_speed
  output = -k2*phi  - k4*phi_dot - k1*theta - k3*theta_dot + k5 * theta_int;       // + 錯誤 - PID_error_Old2
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  return(output);
}


// 速度LQR的PID控制。 DT單位為毫秒 輸入為theta_dot 所以需積分2次取得 theta_int
float speedLQRControl(float DT, float input, float setPoint,  float K1, float K3,float K5 ) {
  float error;
  float output;
  

  error = setPoint-input;
  theta_error += constrain(error,-ITERM_MAX_ERROR,ITERM_MAX_ERROR); //第一次積分
  theta_error = constrain(theta_error,-ITERM_MAX,ITERM_MAX);

  theta = theta_error*DT*0.001;

  //          速度誤差         速度過去誤差和         角度積分誤差
  //output = K3*error + K1*PI_errorSum*DT*0.001 + K5*theta_errorsum*DT*0.001;
  return(output);
}

//在16MHz的200ns=>4條指令
void delay_200ns() {
  __asm__ __volatile__ (
  "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop"); 
}
ISR(TIMER1_COMPA_vect) {
  counter_m[0]++;
  counter_m[1]++;
  if (counter_m[0] >= period_m[0][period_m_index[0]]) {
    counter_m[0] = 0;
    if (period_m[0][0]==ZERO_SPEED)
      return;
    if (dir_m[0])
      SET(PORTB,4);  //DIR電機1
    else
      CLR(PORTB,4);
    // 我們需要等待，以免200ns的產生步進脈衝...
    period_m_index[0] = (period_m_index[0]+1)&0x07; // 週期M指數從0到7
    //delay_200ns();
    SET(PORTD,7); // 步進電機1
    delayMicroseconds(1);
    CLR(PORTD,7);
  }
  if (counter_m[1] >= period_m[1][period_m_index[1]]) {
    counter_m[1] = 0;
    if (period_m[1][0]==ZERO_SPEED)
      return;
    if (dir_m[1])
      SET(PORTC,7);   // DIR電機2
    else
      CLR(PORTC,7);
    period_m_index[1] = (period_m_index[1]+1)&0x07;
    //delay_200ns();
    SET(PORTD,6); // 步進電機1
    delayMicroseconds(1);
    CLR(PORTD,6);
  }
}
void calculateSubperiods(uint8_t motor) {
  int subperiod;
  int absSpeed;
  uint8_t j;

  if (speed_m[motor] == 0) {
    for (j=0;j<8;j++)
      period_m[motor][j] = ZERO_SPEED;
    return;
  }
  if (speed_m[motor] > 0 ) { // 正速度
    dir_m[motor] = 1;
    absSpeed = speed_m[motor];
  }
  else {                     // 負速度
    dir_m[motor] = 0;
    absSpeed = -speed_m[motor];
  }

  for (j=0;j<8;j++)
    period_m[motor][j] = 1000/absSpeed;
  // 計算亞期。如果模塊<0.25=>子期間= 0，如果模塊<0.5=>子週期= 1。如果模塊<0.75子期間=2其他子期間=3
  subperiod = ((1000 % absSpeed)*8)/absSpeed;   // 優化代碼來計運算元期間（整數運算）
  if (subperiod>0)
    period_m[motor][1]++;
  if (subperiod>1)
    period_m[motor][5]++;
  if (subperiod>2)
    period_m[motor][3]++;
  if (subperiod>3)
    period_m[motor][7]++;
  if (subperiod>4)
    period_m[motor][0]++;
  if (subperiod>5)
    period_m[motor][4]++;
  if (subperiod>6)
    period_m[motor][2]++; 
}
void setMotorSpeed(uint8_t motor, int16_t tspeed) {
  // 我們限制最大加速度
  if ((speed_m[motor] - tspeed)>MAX_ACCEL)
    speed_m[motor] -= MAX_ACCEL;
  else if ((speed_m[motor] - tspeed)<-MAX_ACCEL)
    speed_m[motor] += MAX_ACCEL;
  else
    speed_m[motor] = tspeed;  
  calculateSubperiods(motor);  //我們採用四個子週期來提高解析度
  // 為了節省能量，當它沒有運行...
  if ((speed_m[0]==0)&&(speed_m[1]==0))
    digitalWrite(4,HIGH);   //禁用電機
  else
    digitalWrite(4,LOW);   // 使用電機
}
void setup() {
  //Init_Hx711();        //初始化HX711模組連接的IO設置
  Serial1.begin(9600);
  //Get_Maopi();    //獲取毛皮
  //delay(3000);
  //Get_Maopi();    //獲取毛皮
  //Serial.print("Start!\n");
  // 步進引腳 
  pinMode(4,OUTPUT);  // ENABLE MOTORS
  pinMode(12,OUTPUT);  // STEP MOTOR 1 PORTD,7//6
  pinMode(13,OUTPUT);  // DIR MOTOR 1 8
  pinMode(6,OUTPUT); // STEP MOTOR 2 PORTD,6 12
  pinMode(8,OUTPUT); // DIR MOTOR 2 13
  digitalWrite(4,HIGH);   // Disbale motors
  Serial1.begin(9600);
  // 加入I2C總線
  Wire.begin();
  // 4000Khz fast mode
  TWSR = 0;
  TWBR = ((16000000L/I2C_SPEED)-16)/2;
  TWCR = 1<<TWEN;
  //mpu.initialize();
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
  mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpu.setSleepEnabled(false);

  delay(1000);
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    // 打開DMP，現在，它已經準備好
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
  }
  // 陀螺儀的校準
  //機器人必須在初始化時是穩定的
  delay(1500);   // 一段時間來解決的事情從沒有運動演算法的偏置需要一些時間才能生效並重置陀螺儀偏置。1500
  timer_old = millis();
  //我們將覆蓋計時器使用步進電機
  //步進電機初始化
  //計時器CTC模式
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);
  // 輸出模式=00（斷開）
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 
  // 設置計時器預分頻器
  //通常我們使用的8分頻器，產生16MHz的CPU的一個2MHz的計時器
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);
  OCR1A = 80;   // 25Khz
  TCNT1 = 0;
  delay(1000);
  //調整感測器融合增益
  dmpSetSensorFusionAccelGain(0x40);//0x20/////////////////////////////////////////////////////////調整感測器
  delay(1000);
  TIMSK1 |= (1<<OCIE1A);  // 啟用計時器中斷
  digitalWrite(4,LOW);    // 使步進驅動器
  
  // 小電機的振動，表明機器人已準備就緒
  for (uint8_t k=0;k<3;k++) {
    setMotorSpeed(0,3);
    setMotorSpeed(1,-3);
    delay(150);
    setMotorSpeed(0,-3);
    setMotorSpeed(1,3);
    delay(150);
  }  
  mpu.resetFIFO();
  timer_old = millis();

}
// 主迴圈//////////////////////////////////////////////////////////////////////////
void loop() {	   
  kongzhi();
  timer_value = millis();
  // 新的DMP定位解決方案？
  fifoCount = mpu.getFIFOCount();
  if (fifoCount>=18) {
    if (fifoCount>18) { // 如果我們有一個以上的資料包，我們採取簡單的路徑：丟棄緩衝區
      mpu.resetFIFO();
      return;
    }

    loop_counter++;
    slow_loop_counter++;
    dt = (timer_value-timer_old);
    alltime += dt;
    timer_old = timer_value;  
    angle_adjusted_Old = angle_adjusted;
    angle_adjusted = dmpGetPhi();
      
    start=start+1;
    if(start==6) {
      start=6; 
    }
    if(start>5&&angle_adjusted>angle_adjusted_Old+5||start>5&&angle_adjusted<angle_adjusted_Old-5) {
      angle_adjusted= angle_adjusted_Old;
    }
    mpu.resetFIFO();  // 我們始終復位FIFO
    // 我們計算估計機器人的速度
    actual_robot_speed_Old = actual_robot_speed;
    actual_robot_speed = (speed_m[1] - speed_m[0])/2;  // 正面：前鋒 
    int16_t angular_velocity = (angle_adjusted-angle_adjusted_Old)*90.0;     // 角速度角度調整角度調整舊
    
     
     

    int16_t estimated_speed = actual_robot_speed_Old - angular_velocity;     //我們利用機器人速度（T-1）或（T-2），以補償該延遲
    estimated_speed_filtered = estimated_speed_filtered*0.95 + (float)estimated_speed*0.05;//估計速度估計過濾濾速


    target_angle = speedLQRControl(dt,estimated_speed_filtered,throttle,K1,K3,K5); //目標角速度
    target_angle = constrain(target_angle,-max_target_angle,max_target_angle);   // 有限的輸出  目標角度約束目標角度最大目標角度最大目標角度
    //我們整合輸出（加速度）
    
    phi = angle_adjusted;
    phi_dot = (angle_adjusted-angle_adjusted_Old)/dt*1000; 
    theta_dot = estimated_speed_filtered;

    K1efuzzy=fuzzyrule(theta,fuzzyewNB,fuzzyewNM,fuzzyewNS,fuzzyewZO,fuzzyewPS,fuzzyewPM,fuzzyewPB,NBKpo,NMKpt,NSKpt,ZOKpt,PSKpt,PMKpt,PBKpt);  //FUZZY  
    K2efuzzy=fuzzyrule(phi,fuzzyeoNB,fuzzyeoNM,fuzzyeoNS,fuzzyeoZO,fuzzyeoPS,fuzzyeoPM,fuzzyeoPB,NBKpo,NMKpo,NSKpo,ZOKpo,PSKpo,PMKpo,PBKpo);    
    K3efuzzy=fuzzyrule(theta_dot,fuzzyewNB,fuzzyewNM,fuzzyewNS,fuzzyewZO,fuzzyewPS,fuzzyewPM,fuzzyewPB,NBKdt,NMKdt,NSKdt,ZOKdt,PSKdt,PMKdt,PBKdt); 
    K4efuzzy=fuzzyrule(phi_dot,fuzzyepNB,fuzzyepNM,fuzzyepNS,fuzzyepZO,fuzzyepPS,fuzzyepPM,fuzzyepPB,NBKdo,NMKdo,NSKdo,ZOKdo,PSKdo,PMKdo,PBKdo);

     

    
    //fuzzy的輸出需轉單位
    K1=K1+(K1efuzzy*PI/180*0.3);
    K2=K2+(K2efuzzy*PI/180*0.8);
    K3=K3+(K3efuzzy*PI/180*0.3);
    K4=K4+(K4efuzzy*PI/180*0.5);

    
    
    control_output += stabilityLQRControl(dt,theta,throttle,K1,K2,K3,K4,K5);	
    control_output = constrain(control_output,-600,600);   // 限制最大輸出控制      
    // 控制的轉向部分的輸出直接注射
        if(alltime > 3000 && alltime < 3008)
    {
      control_output -= 50;
    }
    
        
    motor1 = control_output + steering;
    motor2 = -control_output + steering;   //馬達2反轉    
    // 限制最大速度
    if(angle_adjusted>45||angle_adjusted<-45) {
      motor1=0;
      motor2=0;
    }
    
    
    motor1 = constrain(motor1,-600,600);   
    motor2 = constrain(motor2,-600,600);
    setMotorSpeed(0,motor1);
    setMotorSpeed(1,motor2); 

    Serial.println(angle_adjusted);
    //Serial.println("  ");

    K1 = LQR_K1 * PI/180 / 1.8;        //theta      步進值to角度
    K2 = LQR_K2 * PI/180;              //phi        弧度to角度
    K3 = LQR_K3 * PI/180 / 1.8;        //theta_dot  步進值to角度
    K4 = LQR_K4 * PI/180;              //phi_dot    弧度to角度
    K5 = LQR_K5 * PI/180 / 1.8;        //theta_int  步進值to角度
      
    mobile=mobile-1;
    if(mobile<=0) {
      mobile=0;
    }
  }
 }


void kongzhi() {
  lkf = Serial1.read();
  switch(lkf) {
  case 'F':
    qian();
    lkf=0;   
    break; 
  case 'B':
    hou();
    lkf=0; 
    break;     
  case 'R':
    zuo();
    lkf=0; 
    break;
  case 'L':
    you();
    lkf=0; 
    break; 
  case 'S':
    ting();
    lkf=0; 
    break;  
   case '0': 
     kkll=0;
     lkf=0; 
     break;
   case '1': 
     kkll=55;
     lkf=0; 
     break;
   case '2': 
    kkll=110;
    lkf=0; 
    break;
   case '3':
     kkll=165;
    lkf=0;
     break;
  case '4':
     kkll=180;
    lkf=0;
     break;
  case '5':
    kkll=200;
    lkf=0;
    break;
  case '6': 
     kkll=240;
    lkf=0;
     break;
   case '7': 
     kkll=280;
    lkf=0;
     break;
   case '8': 
    kkll=320;
    lkf=0;
    break;
   case '9':
     kkll=360;
     lkf=0;
     break;
  }
}
void qian() {
  kkll=210;  /////////////////////////////////////////110  200 360
  throttle=-kkll;
}
void hou() {
  throttle = 210;
}
void zuo() {
  steering = -80;
}
void you() {
  steering = 80;
}
void ting() {
  throttle = 0;
  steering =0;
}
