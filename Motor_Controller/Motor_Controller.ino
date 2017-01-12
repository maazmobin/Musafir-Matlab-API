#define DEBUG 0
#define MAGICADDRESS 7
#include <math.h>

#include <Encoder.h>
#include <EEPROM.h>

Encoder encL(2,4);
Encoder encR(3,5);
long encCurrL, encCurrR;

#include "MusafirMotor.h"
MusafirMotor motorL(7, 6, 9);
MusafirMotor motorR(13, 12, 10);

#include "Navigator.h"
Navigator  navigator;
//from https://github.com/solderspot/NavBot/blob/master/NavBot_v1/BlankBot.h
// Navigator defines
#define WHEELBASE               nvMM(189)      // millimeters
#define WHEEL_DIAMETER          nvMM(89)      // millimeters
#define TICKS_PER_REV           1520          //ROBOT 1
#define WHEEL_DIAMETER_CM       8.9           // centi-meters
#define DISTANCE_PER_TICK       (M_PI*WHEEL_DIAMETER_CM)/((float)TICKS_PER_REV)

// correct for systematic errors
#define WHEEL_RL_SCALER         1.0f  // Ed
#define WHEELBASE_SCALER        1.0f  // Eb
// correct distance 
#define DISTANCE_SCALER         1.0f  // Es

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

int ROBOT=1;
boolean pidActive= false;

unsigned long previousMillis = 0;
int interval = 10; // in ms
int debugInterval = 1000; // in ms
unsigned long debugPreviousMillis = 0;

int poseEnable=0, duration=0;
unsigned long posePreviousMillis=0;

int tempRunningTime=5000; // ms

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;


void setup() {
  Serial.begin(115200);
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
  pidR.Compute();}
  else{pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);}
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
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    encCurrL = encL.read(); encL.write(0); 
    encCurrR =-encR.read(); encR.write(0);
    navigator.UpdateTicks(encCurrL, encCurrR, millis());
    float distanceL = (float)encCurrL*DISTANCE_PER_TICK;
    distanceL=abs(distanceL);
    measuredVelL = (float)distanceL*(1000.0/interval);
    float distanceR = (float)encCurrR*DISTANCE_PER_TICK;
    distanceR=abs(distanceR);
    measuredVelR = (float)distanceR*(1000.0/interval);
  }
  
  pose_broadcast(currentMillis);
 
  if (currentMillis - debugPreviousMillis >= debugInterval) {
    debugPreviousMillis = currentMillis;
    String dataTX=String(ROBOT)+","+String(int(navigator.Position().x/10))+","+String(int(navigator.Position().y/10))+","+String(navigator.Heading());//+","+String(navigator.TurnRate())+","+String(navigator.Speed()/10);
    Serial.println(dataTX);
  }
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
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
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
        val1 = inputString.substring(c1,c2).toInt();
        c1 = c2+1;
        val2 = inputString.substring(c1).toInt();
        if(val1<0) { motorL.setDir(BACKWARD); val1 = -val1; }
        else         motorL.setDir(FORWARD);
        if(val2<0) { motorR.setDir(BACKWARD); val2 = -val2; }
        else         motorR.setDir(FORWARD);
        velL = val1;
        velR = val2;
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
      //COMMAND E , Enable/disable(1/0) , duration(ms)
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
        }else if(val1==2) {
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
    if(DEBUG) //Serial.println("Reading from EEPROM.");
    EEPROM.get((const int)MAGICADDRESS, motorPIDL);   //(address, data)
    EEPROM.get((const int)(MAGICADDRESS+sizeof(motorParams)), motorPIDR);
  }
  else{
    // Set default values
    if(DEBUG) //Serial.println("Setting Default Values.");
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
          String dataTX="e,"+String(inputMillis)+","+String(int(navigator.Position().x/10))+","+String(int(navigator.Position().y/10))+","+String(navigator.Heading());
    Serial.println(dataTX);
    posePreviousMillis=inputMillis;
      }
    }
  }
  
