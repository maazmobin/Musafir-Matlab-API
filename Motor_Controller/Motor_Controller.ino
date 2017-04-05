#define DEBUG 0
#define MECHANICAL_DEBUG 1
#define MAGICADDRESS 7        //Address For the PID
#include <math.h>
#define ID_ADDRESS   200      //Address for the robot ID
#define nrf_delay 1
int ID;  // use less to assign a value.

#include <EEPROM.h>

//        ////ENCODER
#include <Encoder.h>
Encoder encL(2,4);      //
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

                   ////Scalar will be Declared According to Robot ID. No need to change here.
int WHEELBASE = 1 ;                 //mm
int WHEEL_DIAMETER = 1 ;            //mm
long int TICKS_PER_REV = 1 ;        //mm
float WHEEL_DIAMETER_CM = 1 ;       //cm
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
int interval = 10; // in ms         // PID sample interval

int echoStatusMode=1, duration=40;      // Parameters for Br
unsigned long posePreviousMillis=0;

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
    pidL.Compute();
    pidR.Compute();
    }
    
  if (stringComplete) {
    interpretSerialData();
    stringComplete = false;
    inputString = "";
    }
          ////CHECK      //!!!!!!!!!!!!!!!!!!!!
  if(velL>0) motorL.setPWM(pwmL);
  //else motorL.setPWM(0);          //
  
  if(velR>0) motorR.setPWM(pwmR);
  //else motorR.setPWM(0);          //

  unsigned long currentMillis = millis();
            //// ENCODER MODE
    if (currentMillis - previousMillis >= interval && echoStatusMode==2) {
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
  else if (currentMillis - previousMillis >= interval && echoStatusMode==0 && echoStatusMode==1) {
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
        if(nrf_delay){delay(1);}
        Serial.println("a");
        break;
        case 'B':
        // COMMAND: B\n //Return b,x,y
        delay(1);
        Serial.print("b,");
        Serial.print(String(int(navigator.Position().x/(10*cm_meter))));
        Serial.print(",");
        Serial.println( String(int(navigator.Position().y/(10*cm_meter))) );
        break;
        case 'C':
        // COMMAND: C\n //Return c,theta
        if(nrf_delay){delay(1);}
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
        if(velL>0)  
        { 
          velL=constrain(velL,min_speed,max_speed);
          pidL.SetMode(AUTOMATIC);
          }
        else
        {
          pidL.SetMode(MANUAL);
          motorL.setPWM(200);
        }
          
        velR = (val2)*cm_meter;
        if(velR>0)  
        { 
          velR=constrain(velR,min_speed,max_speed);
          pidR.SetMode(AUTOMATIC);
          }
         else
         {
          pidR.SetMode(MANUAL);
          motorR.setPWM(200);
          }

        if(DEBUG){
          Serial.print("Velocity 1 ");
          Serial.println(velL);
          Serial.print("Velocity 2 ");
          Serial.println(velR);
        }         
        pidActive= true;
        if(nrf_delay){delay(1);}
        Serial.println('d');
        break;
      case 'E':   //POSE BROADCAST
      //COMMAND E , Enable/disable/EncoderMode(1/0/-1) , duration(ms)
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        echoStatusMode = inputString.substring(c1,c2).toInt();
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
        if(nrf_delay){delay(1);}
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
        val1 = constrain( val1,-511 , 511 );
        c1 = inputString.indexOf(',',c2)+1;
        val2 = inputString.substring(c1).toInt();
        val2 = constrain( val2,-511 , 511 );

        if(val1 < -255  || val1 > 255)  //Implementing Brake.
        {
          val1= 511-abs(val1);
          motorL.setDir(BRAKE);
          }
        else if(val1<0) { motorL.setDir(BACKWARD); val1 = -val1; }
        else         motorL.setDir(FORWARD);

        if(val2 < -255  || val2 > 255)  //Implementing Brake.
        {
          val2= 511-abs(val2);
          motorR.setDir(BRAKE);
          }
        else if(val2<0) { motorR.setDir(BACKWARD); val2 = -val2; }
        else         motorR.setDir(FORWARD);
        
        pidActive= false;
        pidL.SetMode(MANUAL);
        pidR.SetMode(MANUAL);
        val1=constrain(val1,0,250);
        val2=constrain(val2,0,250);
        pwmL = val1;
        pwmR = val2;
        motorL.setPWM(pwmL);
        motorR.setPWM(pwmR);
        if(DEBUG){
          Serial.print("PWM1: "); Serial.println(val1);        
          Serial.print("PWM2: "); Serial.println(val2);
        }
        if(nrf_delay){delay(1);}
        Serial.println('l');
        break;
      case 'R':
        // COMMAND:  R\n
        if(nrf_delay){delay(1);}
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
        if(nrf_delay){delay(1);}
        Serial.println('i');
        break;
      case 'G':
        // COMMAND: G\n
        // make reset in both encoders
        encOldL = 0 ;
        encOldR = 0 ;
        if(nrf_delay){delay(1);}
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
        if(nrf_delay){delay(1);}
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
  if(echoStatusMode==1)
  {
    if(inputMillis - posePreviousMillis >= duration)
    {
      String dataTX="e,"+String(ID)+","+String(inputMillis)+","+String(float(navigator.Position().x/(10*cm_meter)))+","+String(float(navigator.Position().y/(10*cm_meter)))+","+String(navigator.Heading())
      +","+String(measuredVelL)+","+String(measuredVelR);       
      Serial.println(dataTX);
      posePreviousMillis=inputMillis;
      }
    }
  else if(echoStatusMode==2)
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
    delay(10);
    if(ID==3)  {
      WHEELBASE = 327 ;             //mm
      WHEEL_DIAMETER = 148 ;        //mm
      TICKS_PER_REV = 5490 ;        //mm
      WHEEL_DIAMETER_CM = 14.8 ;    //cm
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
      WHEELBASE = 189 ;             //mm
      WHEEL_DIAMETER = 89 ;         //mm
      TICKS_PER_REV = 1520 ;        //mm
      WHEEL_DIAMETER_CM = 8.9 ;     //cm
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
      WHEELBASE =  ;            //mm
      WHEEL_DIAMETER =  ;       //mm
      TICKS_PER_REV =  ;        //mm
      WHEEL_DIAMETER_CM =  ;    //cm
      DISTANCE_PER_TICK = (M_PI*WHEEL_DIAMETER_CM)/((float)TICKS_PER_REV) ;

      WHEEL_RL_SCALER =  ; // Ed
      WHEELBASE_SCALER =  ; // Eb
      DISTANCE_SCALER =  ; // Es
      enc_left_sign =  ;
      enc_right_sign =  ;

      max_speed= __/cm_meter; //fill blank in cm/sec.
      min_speed= __/cm_meter; 
      }*/
      else
      {
        while(1){
          Serial.println("Incorrect ID");
          delay(1000);
          }
        }
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
    
