#define DEBUG 0
#define MECHANICAL_DEBUG 1
#define MAGICADDRESS 7
#include <math.h>
#define ID_ADDRESS   200 
int ID;  // use less to assign a value.

#include <EEPROM.h>

//        ////ENCODER
#include <Encoder.h>
Encoder encL(2,4);
Encoder encR(3,5);
long encCurrL, encCurrR;
long int encOldL,encOldR;
int enc_left_sign = 1;
int enc_right_sign = 1;
//       ENCODER//////

//      ////MUSAFIR MOTOR
#include "MusafirMotor.h"
MusafirMotor motorL(7, 6, 9);
MusafirMotor motorR(13, 12, 10);
//      MUSAFIR MOTOR////

//      ////NAVIGATOR/ODOMETRY
#include "Navigator.h"
Navigator  navigator;
//from https://github.com/solderspot/NavBot/blob/master/NavBot_v1/BlankBot.h
// Navigator defines
//      NAVIGATOR/ODOMETRY////

int WHEELBASE = 327 ;
int WHEEL_DIAMETER = 148 ;
long int TICKS_PER_REV = 5490 ;
float WHEEL_DIAMETER_CM = 14.8 ;
float DISTANCE_PER_TICK = (M_PI*WHEEL_DIAMETER_CM)/((float)TICKS_PER_REV) ;

float WHEEL_RL_SCALER = 1.0f ; // Ed
float WHEELBASE_SCALER = 1.0f ; // Eb
float DISTANCE_SCALER = 1.0f ; // Es


//      ////PID
#include <PID_v1.h>
struct motorParams {
  double kp;
  double ki;
  double kd;
};

motorParams motorPIDL;
motorParams motorPIDR;
double measuredVelL=0, measuredVelR=0;
double pwmL=0, pwmR=0;
double velL=0, velR=0;
// PID (&input, &output, &setpoint, kp, ki, kd, DIRECT/REVERSE)
PID pidL(&measuredVelL, &pwmL, &velL, 2.05,1,0, DIRECT);
PID pidR(&measuredVelR, &pwmR, &velR, 2,1,0, DIRECT);

//      PID////

boolean pidActive= false;

unsigned long previousMillis = 0;
int interval = 10; // in ms
int debugInterval = 1000; // in ms
unsigned long debugPreviousMillis = 0;

int poseEnable=1, duration=40;
unsigned long posePreviousMillis=0;

int tempRunningTime=5000; // ms

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;

#define cm_meter 100.0 //100.0 = meter , 1.0 = cm
int max_speed;
int min_speed;

void setup() {
  Serial.begin(115200);
  defineRobot();
  motorL.setDir(FORWARD);
  motorR.setDir(FORWARD);
  
  navigator.InitEncoder( WHEEL_DIAMETER, WHEELBASE, TICKS_PER_REV );
  navigator.SetDistanceScaler( DISTANCE_SCALER );
  navigator.SetWheelbaseScaler( WHEELBASE_SCALER );
  navigator.SetWheelRLScaler( WHEEL_RL_SCALER );
  navigator.SetMinInterval(interval);
  navigator.Reset(millis());

initEEPROM();
  initPID();
  velL = 00; //cm/s for TESTING of nav Speed calculations.
  velR = 00;
  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);
  
  inputString.reserve(200);
}

void loop() {   
  if(pidActive){
    pidL.SetMode(AUTOMATIC);
    pidR.SetMode(AUTOMATIC);
    pidL.Compute();
    pidR.Compute();
    }
  else{
    pidL.SetMode(AUTOMATIC);
    pidR.SetMode(AUTOMATIC);
    }
    
  if (stringComplete) {
    interpretSerialData();
    stringComplete = false;
    inputString = "";
    }
    
  if(velL>0) motorL.setPWM(pwmL);
  else motorL.setPWM(0);
  
  if(velR>0) motorR.setPWM(pwmR);
  else motorR.setPWM(0);

  unsigned long currentMillis = millis();
            //// ENCODER MODE
    if (currentMillis - previousMillis >= interval && poseEnable==2) {
    previousMillis = currentMillis;
    encCurrL =enc_left_sign*encL.read(); encL.write(0); 
    encCurrR =enc_right_sign*encR.read(); encR.write(0);
    encOldL+=encCurrL;
    encOldR+=encCurrR;
    float distanceL = (float)encCurrL*DISTANCE_PER_TICK;
    distanceL=abs(distanceL);
    measuredVelL = (float)distanceL*(1000.0/interval);
    float distanceR = (float)encCurrR*DISTANCE_PER_TICK;
    distanceR=abs(distanceR);
    measuredVelR = (float)distanceR*(1000.0/interval);
    navigator.Reset(millis());
  }
            ////  ODOMETRY MODE
  else if (currentMillis - previousMillis >= interval && poseEnable>=0) {
    previousMillis = currentMillis;
    encCurrL =enc_left_sign*encL.read(); encL.write(0); 
    encCurrR =enc_right_sign*encR.read(); encR.write(0);
    navigator.UpdateTicks(encCurrL, encCurrR, millis());
    float distanceL = (float)encCurrL*DISTANCE_PER_TICK;
    distanceL=abs(distanceL);
    measuredVelL = (float)distanceL*(1000.0/interval);
    float distanceR = (float)encCurrR*DISTANCE_PER_TICK;
    distanceR=abs(distanceR);
    measuredVelR = (float)distanceR*(1000.0/interval);
  }

  pose_broadcast(currentMillis);
}



void initPID(void){
  pidL.SetMode(MANUAL); // PID CONTROL OFF
  pidR.SetMode(MANUAL);
  pidL.SetSampleTime(interval); // sample time for PID
  pidR.SetSampleTime(interval);
  pidL.SetOutputLimits(0,250);  // min/max PWM
  pidR.SetOutputLimits(0,250);  
}

void VelL(int velocity)
{
  if(velocity>=0){velL=velocity;motorL.setDir(FORWARD);}
  else if(velocity<=0){velL=(-1*velocity);motorL.setDir(BACKWARD);}
  }
  
void VelR(int velocity)
{
  if(velocity>=0){velR=velocity;motorR.setDir(FORWARD);}
  else if(velocity<=0){velR=(-1*velocity);motorR.setDir(BACKWARD);}
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void interpretSerialData(void){
    int c1=1, c2=1;
    int val1=0, val2=0;
    switch(inputString[0]){
      case 'A':
        // COMMAND:  A,x,y,theta
        float xx,yy,tt;
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        xx = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        yy = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        tt = inputString.substring(c1).toFloat();
        navigator.SetPose(xx,yy,tt);
        delay(1);
        Serial.println("a");
        break;
        case 'B':
        // COMMAND: B\n //Return b,x,y
        delay(1);
        Serial.print("b,");
        Serial.print(String(int(navigator.Position().x/10)));
        Serial.print(",");
        Serial.println( String(int(navigator.Position().y/10)) );
        break;
        case 'C':
        // COMMAND: C\n //Return c,theta
        delay(1);
        Serial.print("c,");
        Serial.println( String(navigator.Heading()) );
        break;
      case 'D':
        // COMMAND:  D,speed_motor_left,speed_motor_right\n
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        val1 = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        val2 = inputString.substring(c1).toFloat();
        if(val1<0) { motorL.setDir(BACKWARD); val1 = -val1; }
        else if(val1==0) { motorL.setDir(BRAKE);}
        else         motorL.setDir(FORWARD);
        
        if(val2<0) { motorR.setDir(BACKWARD); val2 = -val2; }
        else if(val2==0) { motorR.setDir(BRAKE);}
        else         motorR.setDir(FORWARD);
        
        velL = (val1)*cm_meter;
        if(velL>0)  { velL=constrain(velL,min_speed,max_speed); }
        velR = (val2)*cm_meter;
        if(velR>0)  { velR=constrain(velR,min_speed,max_speed); }
        
        if(DEBUG){
          Serial.print("Velocity 1 ");
          Serial.println(velL);
          Serial.print("Velocity 2 ");
          Serial.println(velR);
        }         
        if(velL>0)
          pidL.SetMode(AUTOMATIC);
        else
          pidL.SetMode(MANUAL);
        if(velR>0)
          pidR.SetMode(AUTOMATIC);
        else
          pidR.SetMode(MANUAL);
        pidActive= true;
        delay(1);
        Serial.println('d');
        break;
      case 'E':   //POSE BROADCAST
      //COMMAND E , Enable/disable/EncoderMode(1/0/-1) , duration(ms)
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        poseEnable = inputString.substring(c1,c2).toInt();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        duration = inputString.substring(c1).toFloat();
        break;
      case 'H':
        // COMMAND:  H,P,I,D,1/2\n
        float p,i,d;
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        p = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        i = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        d = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        val1 = inputString.substring(c1).toInt();
        if(val1==1) {
          motorPIDL.kp = p;
          motorPIDL.ki = i;
          motorPIDL.kd = d;
          pidL.SetTunings(motorPIDL.kp, motorPIDL.ki, motorPIDL.kd);
          EEPROM.put((const int)MAGICADDRESS, motorPIDL);
          if(DEBUG) Serial.println("motorPIDL ");
        }
        else if(val1==2){
          motorPIDR.kp = p;
          motorPIDR.ki = i;
          motorPIDR.kd = d;
          pidR.SetTunings(motorPIDR.kp, motorPIDR.ki, motorPIDR.kd);
          EEPROM.put((const int)(MAGICADDRESS+sizeof(motorParams)), motorPIDR);
          if(DEBUG) Serial.println("motorPIDR ");
        }
        delay(1);
        Serial.print("h,");
        Serial.print(val1);
        Serial.print(',');
        Serial.println(p);
        Serial.print(',');
        Serial.println(i);
        Serial.print(',');
        Serial.println(d);
        break;
      case 'L':
        // COMMAND:  L,speed_motor_left,speed_motor_right\n
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        val1 = inputString.substring(c1,c2).toInt();
        c1 = inputString.indexOf(',',c2)+1;
        val2 = inputString.substring(c1).toInt();
        if(val1<0) { motorL.setDir(BACKWARD); val1 = -val1; }
        else         motorL.setDir(FORWARD);
        if(val2<0) { motorR.setDir(BACKWARD); val2 = -val2; }
        else         motorR.setDir(FORWARD);
        pidActive= false;
        pidL.SetMode(MANUAL);
        pidR.SetMode(MANUAL);
        pwmL = val1;
        pwmR = val2;
        motorL.setPWM(pwmL);
        motorR.setPWM(pwmR);
        if(DEBUG){
          Serial.print("PWM1: "); Serial.println(val1);        
          Serial.print("PWM2: "); Serial.println(val2);
        }
        delay(1);
        Serial.println('l');
        break;
      case 'R':
        // COMMAND:  R\n
        delay(1);
        Serial.print("r,");
        Serial.print(encL.read());
        Serial.print(',');
        Serial.print(encR.read());
        Serial.println();
        break;
      case 'I':
        // COMMAND: I\n
        // ZP=add navigator.reset here
        navigator.Reset(millis());
        delay(1);
        Serial.println('i');
        break;
      case 'G':
        // COMMAND: G\n
        // make reset in both encoders
        encOldL = 0 ;
        encOldR = 0 ;
        delay(1);
        Serial.println('g');
        break;
      case 'J':
        // COMMAND: J\n
        sendRobotParameters();
        Serial.println('j');
        break;
      case 'S':
        // COMMAND:  S,1/2\n
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        val1 = inputString.substring(c1).toInt();
        delay(1);
        if(val1==1) {
          Serial.print("s,");
          Serial.print(motorPIDL.kp);
          Serial.print(',');
          Serial.print(motorPIDL.ki);
          Serial.print(',');
          Serial.println(motorPIDL.kd);
        }
        else if(val1==2) {
          Serial.print("s,");
          Serial.print(motorPIDR.kp);
          Serial.print(',');
          Serial.print(motorPIDR.ki);
          Serial.print(',');
          Serial.println(motorPIDR.kd);
        }
        break;
       default:
        Serial.print("UNKNOWN COMMAND: ");
        Serial.println(inputString);
        break;
    }
}

void initEEPROM(void){
  int checkEEPROM=0;
  EEPROM.get(0, checkEEPROM);
  if(checkEEPROM==MAGICADDRESS){
    if(DEBUG) Serial.println("Reading from EEPROM.");
    EEPROM.get((const int)MAGICADDRESS, motorPIDL);   //(address, data)
    EEPROM.get((const int)(MAGICADDRESS+sizeof(motorParams)), motorPIDR);
  }
  else{
    // Set default values
    if(DEBUG) Serial.println("Setting Default Values.");
    EEPROM.put(0, MAGICADDRESS);
    motorPIDL.kp = 1.0;
    motorPIDL.ki = 0.0;
    motorPIDL.kd = 0.0;
    EEPROM.put((const int)MAGICADDRESS, motorPIDL);

    motorPIDR.kp = 1.0;
    motorPIDR.ki = 0.0;
    motorPIDR.kd = 0.0;
    EEPROM.put((const int)(MAGICADDRESS+sizeof(motorParams)), motorPIDR);
  }  
}

void pose_broadcast(unsigned long inputMillis)
{
  if(poseEnable==1)
  {
    if(inputMillis - posePreviousMillis >= duration)
    {
      String dataTX="e,"+String(ID)+","+String(inputMillis)+","+String(float(navigator.Position().x/(10*cm_meter)))+","+String(float(navigator.Position().y/(10*cm_meter)))+","+String(navigator.Heading())
      +","+String(measuredVelL)+","+String(measuredVelR);       
      Serial.println(dataTX);
      posePreviousMillis=inputMillis;
      }
    }
  else if(poseEnable==2)
   {
     if(inputMillis - posePreviousMillis >= duration)
     {
       String dataTX="e,"+String(ID)+","+String(inputMillis)+","+String(encOldL)+","+String(encOldR)
       +","+String(measuredVelL)+","+String(measuredVelR);
       Serial.println(dataTX);
       posePreviousMillis=inputMillis;
       }
     }
  }
  void defineRobot (void)
  {
    EEPROM.get(ID_ADDRESS,ID);
   // delay(1);
    if(ID==3)  {
      WHEELBASE = 327 ;
      WHEEL_DIAMETER = 148 ;
      TICKS_PER_REV = 5490 ;
      WHEEL_DIAMETER_CM = 14.8 ;
      DISTANCE_PER_TICK = (M_PI*WHEEL_DIAMETER_CM)/((float)TICKS_PER_REV) ;

      WHEEL_RL_SCALER = 1.0f ; // Ed
      WHEELBASE_SCALER = 1.0f ; // Eb
      DISTANCE_SCALER = 1.0f ; // Es
      enc_left_sign = -1;
      enc_right_sign = 1;

      max_speed=20/cm_meter;      //fill blank in cm/sec.
      min_speed=10/cm_meter;
      }
    else if(ID==1 || ID == 2)  {
      WHEELBASE = 189 ;
      WHEEL_DIAMETER = 89 ;
      TICKS_PER_REV = 1520 ;
      WHEEL_DIAMETER_CM = 8.9 ;
      DISTANCE_PER_TICK = (M_PI*WHEEL_DIAMETER_CM)/((float)TICKS_PER_REV) ;

      WHEEL_RL_SCALER = 1.0f ; // Ed
      WHEELBASE_SCALER = 1.0f ; // Eb
      DISTANCE_SCALER = 1.0f ; // Es
      enc_left_sign = 1;
      enc_right_sign = -1;

      max_speed=100/cm_meter;     //fill blank in cm/sec.
      min_speed=10/cm_meter;
      }
        //DEFINE HERE FOR NEW ROBOT
    /*else if(ID==4)  {
      WHEELBASE =  ;
      WHEEL_DIAMETER =  ;
      TICKS_PER_REV =  ;
      WHEEL_DIAMETER_CM =  ;
      DISTANCE_PER_TICK = (M_PI*WHEEL_DIAMETER_CM)/((float)TICKS_PER_REV) ;

      WHEEL_RL_SCALER =  ; // Ed
      WHEELBASE_SCALER =  ; // Eb
      DISTANCE_SCALER =  ; // Es
      enc_left_sign =  ;
      enc_right_sign =  ;

      max_speed= __/cm_meter; //fill blank in cm/sec.
      min_speed= __/cm_meter; 
      }*/
      if(MECHANICAL_DEBUG==1){
        sendRobotParameters();
        }
    }
void sendRobotParameters(void)
{
    Serial.print("ID : ");
    Serial.println(ID);
    Serial.print("WHEELBASE: ");
    Serial.println(WHEELBASE);
    Serial.print("WHEEL_DIAMETER: ");
    Serial.println(WHEEL_DIAMETER);
    Serial.print("TICKS_PER_REV: ");
    Serial.println(TICKS_PER_REV);
    Serial.print("WHEEL_DIAMETER_CM: ");
    Serial.println(WHEEL_DIAMETER_CM);
    Serial.print("DISTANCE_PER_TICK: ");
    Serial.println(DISTANCE_PER_TICK);
    Serial.print("WHEEL_RL_SCALER: ");
    Serial.println(WHEEL_RL_SCALER);
    Serial.print("WHEELBASE_SCALER: ");
    Serial.println(WHEELBASE_SCALER);
    Serial.print("DISTANCE_SCALER: ");
    Serial.println(DISTANCE_SCALER);
    Serial.print("Maximum Speed: ");
    Serial.println(max_speed);
    Serial.print("Minimum Speed: ");
    Serial.println(min_speed);
  }
    
