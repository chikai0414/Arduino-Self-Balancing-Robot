
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
String cmd = "";

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}





/*-- 腳位定義 -------------------------------------------------------------------------------------------*/
 // If batterys are at backward & user see the car from the backward
 // motor A(right motor) connected between A01 and A02
 // motor B(left motor) connected between B01 and B02
 const int PWMA = 5, PWMB = 9;               // Speed control 
 const int AIN1 = 6, BIN1 = 11;              // Direction +
 const int AIN2 = 4, BIN2 = 10;              // Direction -
 const int STBY = 7;                         // standby(停止)
#include <SoftwareSerial.h> // 引用程式庫
#include "MsTimer2.h"
#include "BalanbotEncoder.h"
#include "BlanbotMotor.h"

SoftwareSerial BT(12,13); // 接收腳, 傳送腳
char val;
char mode=0;
double ref=3;
BalanbotMotor MotorL;
BalanbotMotor MotorR;
/*-- 車輪速度轉換相關 -----------------------------------------------------------------------------------*/
 int movecmd=0;
 bool movenext=true;
 int LEFT = 1;
 int RIGHT = 2;
 float maxPowerRemap = 255/100;
 char state;
 int countR = 0;
 int countL = 0;
 int c=0;
 int test=0;
double asd=0;
int timecount=0;
 double oldAL=0;
 double extraang=0;
 double Kp=55;
 double Ki=1400;
 double Kd=0.4;
 double interror = 0;
 double olderror = 0;
 double error;
double power;

 double wang=0;
 double desL=0,desR=0;
 double wKp=30;
 double wKi=10000;
 double wKd=0;
 double winterror = 0;
 double wolderror = 0;
 double werror;
 bool reach=false;
 float dT=0.001;
 float speedL=0.0;
 float speedR=0.0;
 float currentAngleL=0;
 float currentAngleR=0;
 float lastAngleL=0;
 float lastAngleR=0;
 float angypr=0;
 double oldvalue1=0;
 float oldvalue2=0;
/*-- 副函式定義 -----------------------------------------------------------------------------------------*/
 void posCmd()
{
  int cmdLen = cmd.length();
  desL = cmd.substring(0, cmdLen).toDouble()-MotorL.GetAngle();
  desR = cmd.substring(0, cmdLen).toDouble()+MotorR.GetAngle();
  mode='g';
  cmd = "";
}
 void InterruptFuncR(){
        currentAngleR=MotorR.GetAngle();
    MotorR.Update();
 }
 void InterruptFuncL(){
   currentAngleL=MotorL.GetAngle();

    MotorL.Update();
 }

 void TimerInterrupt(){
    sei();
    
    lastAngleL=currentAngleL;
    currentAngleL=MotorL.GetAngle();

    lastAngleR=currentAngleR;
    currentAngleR=MotorR.GetAngle();
    
    speedL =(currentAngleL - lastAngleL)/dT;
    speedR =(currentAngleR - lastAngleR)/dT;

      // if programming failed, don't try to do anything
    //   if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
   
  // reset interrupt flag and get INT_STATUS byte
   // mpuInterrupt = false;
    // while (!mpuInterrupt && fifoCount < packetSize) {
        
   // }

   mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount > 500) {
        // reset so we can continue cleanly
         mpu.resetFIFO();
       // Serial.println(F("FIFO overflow!"));
       
        mpuIntStatus = mpu.getIntStatus();

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
         while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

 
 //       #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
   //     #endif
           angypr=-ypr[2] * 180/M_PI;
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

 
 //       #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
   //     #endif
           angypr=-ypr[2] * 180/M_PI;
    }


     double AL=-MotorL.GetAngle();
     double RL=MotorR.GetAngle();
     timecount++;


    bool control=true;
    movenext=false;
   
    werror = AL-desL;

     if(timecount==525&&mode!='a'&&mode!='d'&&!control){
          if(RL-oldAL>1){ 
              //extraang+=1.5;
              ref+=1.5;
          }
          else if(RL-oldAL<-0.5){
               
              ref-=0.5;
          }
          if(RL-oldAL<-1){
               
              ref-=1;
          }
          if(RL-oldAL<-1.5){
               
              ref-=1;
          }
          else{
              extraang/=3;
          }
         
          timecount-=25;
          oldAL=RL;
     }

    if(werror<20&&werror>-20&&mode!='r'&&mode!='l'&&mode!='/'&&!control){
        wang=0;
        if(werror<0.3&&werror>-0.3){
          wang=0;
          extraang=0;
          reach=true;
          mode=' ';
          if(!movenext){
            movenext=true;
            movecmd++;
          }
        }
     }else if(!control){
        if(werror>0)
          wang=1;
        if(werror<0)
          wang=-0.75;
     }
   
     
     double error = angypr+ref+wang+extraang;
     error=error*PI/180;
     interror += error*0.001;
     double lasterror = (error - olderror)/0.001;
     olderror = error; //注意 olderror 的順序
     power = error * Kp + interror * Ki + lasterror * Kd;
     
    //限制PID輸出值:
    if (power > 12)
      power = 12;
    if (power < -12)
      power = -12;
    power=(int)(power*255/12);
    double powerL=power;
    double powerR=power;

    if(mode=='a'){
       powerR+=15;
       powerL/=2;
     }
     if(mode=='d'){
       powerL+=15;
       powerR/=2;
     }
    if (powerL > 0) //判斷偏左或右
      {
        MotorL.InverseRotationDirectionDefinition(false);
        MotorL.Rotate(powerL);
      }
    else if(power<0){
        MotorL.InverseRotationDirectionDefinition(true);
        MotorL.Rotate(-powerL);
     }
     if (powerR > 0) //判斷偏左或右
      {
        MotorR.InverseRotationDirectionDefinition(false);
        MotorR.Rotate(powerR);
      }
    else if(powerR<0){
        MotorR.InverseRotationDirectionDefinition(true);
        MotorR.Rotate(-powerR);
     }
    //asd+=0.0000505;
    Serial.print(desL); Serial.print(desL);
    Serial.print(" ");
    Serial.print(movecmd);
    Serial.print(AL);
    Serial.print(" ");
    Serial.print(" ");
    Serial.println(RL);
    Serial.print(" ");

    Serial.print("\n");
    //BT.print(AL);
    //BT.print(" ");
   // BT.print(power);
   // BT.print(" ");
   // BT.print(angypr);
  //  BT.println("");
    
 }
 float Filter1(float n) {
  if(((n - oldvalue1) > 5) || ((oldvalue1- n) > 5)){
     return oldvalue1; 
  }
    oldvalue1=n;
    return n;
 }
float Filter2(float n) {
  if(((n - oldvalue2) > 5) || ((oldvalue2- n) > 5)){
     return oldvalue2; 
  }
    oldvalue2=n;
    return n;
 }
 
 void parseCmd();
 void initMotor();
 void stop();
 void moveMotor(int, float);
// void move(int motor, int speed, int direction);
  void rotate(){
      MotorR.Rotate(50);

      MotorL.Rotate(50);
   
    c++;
    if(c>1000)
      c/=1000;
    else if(c==800){
       MotorL.InverseRotationDirectionDefinition(true);
       MotorR.InverseRotationDirectionDefinition(true);
  
      
    }
    else if(c==500){
       MotorL.InverseRotationDirectionDefinition(false);
       MotorR.InverseRotationDirectionDefinition(true);
    }
    else if(c==100){
       MotorL.InverseRotationDirectionDefinition(true);
       MotorR.InverseRotationDirectionDefinition(false);
    }
 }
/*=========================================================================================================
==== Setup ================================================================================================
=========================================================================================================*/




// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
   
    Serial.begin(57600);
    BT.begin(57600);

  

    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

//    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
     MotorL.SetMotorPins(9,11,10,7);
    MotorL.SetEncoderPins(3,8);
    MotorR.SetMotorPins(5,6,4,7);
    MotorR.SetEncoderPins(2,A3);
  
  
    MsTimer2::set(dT*1000,TimerInterrupt);
    MsTimer2::start();
    attachInterrupt(digitalPinToInterrupt(2),InterruptFuncR,RISING);
    attachInterrupt(digitalPinToInterrupt(3),InterruptFuncL,RISING);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void parseCmd()
{
  int first_commaAt = -1, second_commaAt = -1;
  int cmdLen = cmd.length();
  for(int i = 0; i<cmdLen; ++i){
    if(cmd[i] ==','){
      first_commaAt = i;
      break;
    }
  }
  for(int i = first_commaAt+1; i<cmdLen; ++i){
    if(cmd[i] ==','){
      second_commaAt = i;
      break;
    }
  }

  if(first_commaAt != -1 && second_commaAt != -1){
    Kp = cmd.substring(0, first_commaAt).toDouble();
    Ki = cmd.substring(first_commaAt+1, second_commaAt).toDouble();
    Kd = cmd.substring(second_commaAt+1).toDouble();
  
    
  }
  cmd = "";
}

void refCmd()
{
  int cmdLen = cmd.length();
  ref = -cmd.substring(0, cmdLen).toDouble()+4;
  cmd = "";
}
void loop() {

    String v="";
   /* v+=angypr;
    v+="!";
    v+=-MotorL.GetAngle();
    v+="?";
 */
    v+=ref;
    v+=" ";
    v+=mode;
    v+='\n';
 // v+=MotorR.GetAngle();
    BT.print(v);
    delay(10);

   
   if(BT.available()){
    char ch = BT.read();
    bool isEnd = (ch == '!'||ch=='p')?true:false;
    if(isEnd){
      if(ch=='!')
        parseCmd();
      if(ch=='p')
        posCmd();

    }else{
      cmd += String(ch);
       if(ch=='r'||ch=='l'){
          desL = -MotorL.GetAngle();
          desR = MotorR.GetAngle();
          mode=ch;
          cmd="";
       }
       if(ch=='w'||ch=='s'){
          if(ch=='w')
            ref-=0.15;
          else
            ref+=0.15;
          cmd="";
       }
       if(ch==' '){
          ref=(ref+3)/2;
       }
       if(ch=='a'){
        if(mode!='d'&&mode!='a'){
           mode=ch;
           cmd="";
        }else{
          mode=' ';
        }
       }   
       if(ch=='d'){
        if(mode!='d'&&mode!='a'){
           mode=ch;
           cmd="";
        }else{
        
          mode=' ';
        }
       }  
       if(ch=='z'){
          mode=' ';
          cmd="";
          ref=3;
       }
       if(ch=='+'){
          mode='+';
          cmd="";
          ref=1.5;
       }
       if(ch=='-'){
          mode='-';
          cmd="";
          ref=4.5;
       }
       if(ch=='*'){
          if(mode=='+'){
            mode=' ';
            cmd="";
            ref=3.75;
          }else if(mode=='-'){
            mode=' ';
            cmd="";
            ref=2.25;
          }
          else{
            ref=3;
          }
       }
    
  }
    /*
    test++;
    if(test>1000){
    MotorR.Rotate(106);
    MotorL.Rotate(106);
    }*/
    /*if ((BT.available())) {
    val = BT.read();
    if(val=='w'||val=='s'||val=='a'||val=='d'){
      MotorR.Rotate(50);
      MotorL.Rotate(50);
      state = val;
    }
    else if(val=='x')
      state = val;
    //BT.write(state);
    //BT.write(-Filter2(MotorL.GetSpeed()));
   // Serial.print(val);
   // Serial.print("\n");
     if(state=='w'){
      
      MotorL.InverseRotationDirectionDefinition(false);
      MotorR.InverseRotationDirectionDefinition(false);
      MotorR.Rotate(50);
      MotorL.Rotate(50);
      
    }
    else if(state=='a'){
      
      MotorL.InverseRotationDirectionDefinition(true);
      MotorR.InverseRotationDirectionDefinition(false);
      MotorR.Rotate(50);
      MotorL.Rotate(50);
    }
    else if(state == 's'){

      MotorL.InverseRotationDirectionDefinition(true);
      MotorR.InverseRotationDirectionDefinition(true);
      MotorR.Rotate(50);
      MotorL.Rotate(50);
      
    }
    else if(state == 'd'){
      MotorL.InverseRotationDirectionDefinition(false);
      MotorR.InverseRotationDirectionDefinition(true);
      MotorR.Rotate(50);
      MotorL.Rotate(50);
      
    }
    else{
      MotorR.Brake();
      MotorL.Brake();
    }     
 


 
  }
   */

   /*
     Serial.print(MotorR.GetAngle());
     Serial.print(" ");
     Serial.print(-MotorL.GetAngle());
    Serial.print(" ");
    Serial.print(angypr);
     */
     
     
    
 //    Serial.print("\n");
   }}
