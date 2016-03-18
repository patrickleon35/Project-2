/*

 MSE 2202 MSEBot base code for Labs 3 and 4
 Language: Arduino
 Authors: Michael Naish and Eugen Porter
 Date: 16/01/17
 
 Rev 1 - Initial version
 Rev 2 - Update for MSEduino v. 2
 
 */

#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;    
Servo servo_GripMotor;
Servo servo_LeverMotor;


I2CEncoder encoder_LeftMotor;
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_MidMotor;

// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
//#define DEBUG_MOTOR_CALIBRATION





//other variables

boolean startRunning = false;
long prevTimer = 0;
int sensorDetected;





//port pin constants


const int ci_Right_Motor = 2;
const int ci_Start_Button = 6;

const int ci_Left_Motor = 3;
const int ci_Arm_Motor = 10;
const int ci_Grip_Motor = 11;
const int ci_Mode_Switch = 7;
const int ci_Lever_Motor = 8;
const int ci_Hall_Sensor_0 = A0;
const int ci_Hall_Sensor_1 = A1;
const int ci_Hall_Sensor_2 = A2;
const int ci_Hall_Sensor_3 = A3;
const int ci_Hall_Sensor_4 = A4;



const int ci_Right_Line_Tracker = A0;
const int ci_Middle_Line_Tracker = A1;
const int ci_Left_Line_Tracker = A2;
const int ci_Light_Sensor = A3;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow



//constants

const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 140;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Closed = 90;   
const int ci_Arm_Servo_Retracted = 55;      //  "
const int ci_Arm_Servo_Extended = 120;      //  "
const int ci_Display_Time = 500;
const int ci_Line_Tracker_Calibration_Interval = 100;
const int ci_Line_Tracker_Cal_Measures = 20;
const int ci_Line_Tracker_Tolerance = 50;   // May need to adjust this
const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_Echo_Time;
unsigned int ui_Left_Line_Tracker_Data;
unsigned int ui_Middle_Line_Tracker_Data;
unsigned int ui_Right_Line_Tracker_Data;
unsigned int ui_Motors_Speed = 1900;        // Default run speed
unsigned int ui_Left_Motor_Speed = 1600;
unsigned int ui_Right_Motor_Speed = 1600;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Cal_Cycle;
unsigned int ui_Left_Line_Tracker_Dark;
unsigned int ui_Left_Line_Tracker_Light;
unsigned int ui_Middle_Line_Tracker_Dark;
unsigned int ui_Middle_Line_Tracker_Light;
unsigned int ui_Right_Line_Tracker_Dark;
unsigned int ui_Right_Line_Tracker_Light;
unsigned int ui_Line_Tracker_Tolerance;

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF


boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

//PID Variables
int dl_Ticks;
int dr_Ticks;
int error_Left;
int error_Right;
int derivative_Right;
int derivative_Left;
int integral_Right;
int integral_Left;
int setpoint = 6;
int last_Error_Left;
int last_Error_Right;
int correction_Right;
int correction_Left;
int kp = 0;
int ki = 0;
int kd = 0;




void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  pinMode(ci_Start_Button,INPUT);
  digitalWrite(ci_Start_Button,HIGH);


  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  // set up arm motors
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);
  pinMode(ci_Grip_Motor, OUTPUT);
  servo_GripMotor.attach(ci_Grip_Motor);

  //set up lever arm
  pinMode(ci_Lever_Motor, OUTPUT);
  servo_LeverMotor.attach(ci_Lever_Motor);
  



  // set up encoders. Must be initialized in order that they are chained together, 
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward



}


void loop()
{



  
 
 

    
  if (digitalRead(ci_Start_Button) == LOW)
  {
    startRunning = true;
  }

  //mode 0
  if (startRunning == false)
  {  
    
      servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
    //  servo_MidMotor.writeMicroseconds(ci_Mid_Motor_Stop); 
      servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop); 
      servo_ArmMotor.write(ci_Arm_Servo_Retracted);
      servo_GripMotor.write(ci_Grip_Motor_Closed);
 //     servo_LeverMotor.write(ci_Lever_Motor_Up);
      encoder_LeftMotor.zero();
      encoder_RightMotor.zero();
  //    encoder_MidMotor.zero();
  }


  //mode 1
  if (startRunning == true && currentMode() == 1)
  {
        ui_Right_Motor_Speed = 1500;
        ui_Left_Motor_Speed = 1500;
             encoder_LeftMotor.zero();
      encoder_RightMotor.zero();
   
    
     

    
    
    

#ifdef DEBUG_ENCODERS           
        l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
        l_Right_Motor_Position = encoder_RightMotor.getRawPosition();
        Serial.print("Encoders L: ");
        Serial.print(l_Left_Motor_Position);
        Serial.print(", R: ");
        Serial.println(l_Right_Motor_Position);
#endif


        

      if (millis() - prevTimer > 100)//&& fluxDetected == false)
     {
        driveStraight();
        prevTimer = millis();
     }
     

     if (fluxDetected() == true) //for now just stop the robot
     {
        ui_Right_Motor_Speed = 1500;
        ui_Left_Motor_Speed = 1500;
     }
        
   




      
   

       


    
        
        
#ifdef DEBUG_MOTORS
       
        Serial.print(", Default: ");
        Serial.print(ui_Motors_Speed);
        Serial.print(", Left = ");
        Serial.print(ui_Left_Motor_Speed);
        Serial.print(", Right = ");
        Serial.println(ui_Right_Motor_Speed);
#endif  

 
  }





  
  //mode 2
  if (startRunning == true && currentMode() == 2)
  {
    ui_Right_Motor_Speed = 1400;
    ui_Right_Motor_Speed = 1400;
    
  }

 
 
  updateMotorSpeed();
  
}

    














boolean fluxDetected()
{
  if (analogRead(ci_Hall_Sensor_0) > 512 || analogRead(ci_Hall_Sensor_0) < 500)
  {
    sensorDetected = 0;
    return true;
  }
  if (analogRead(ci_Hall_Sensor_1) > 512 || analogRead(ci_Hall_Sensor_1) < 500)
  {
    sensorDetected = 1;
    return true;
  }
  if (analogRead(ci_Hall_Sensor_2) > 512 || analogRead(ci_Hall_Sensor_2) < 500)
  {
    sensorDetected = 2;
    return true;
  }
  if (analogRead(ci_Hall_Sensor_3) > 512 || analogRead(ci_Hall_Sensor_3) < 500)
  {
    sensorDetected = 3;
    return true;
  }
  if (analogRead(ci_Hall_Sensor_4) > 512 || analogRead(ci_Hall_Sensor_4) < 500)
  {
    sensorDetected = 4;
    return true;
  }
  else return true;
}




int currentMode()
{
  if (digitalRead(ci_Mode_Switch) == LOW) //when switch is in top position, operates in mode 1
  {
    return 1;
  }
  if (digitalRead(ci_Mode_Switch) == HIGH) // when switch is in bottom position, operates in mode 2
  {
    return 2;
  }
}




void updateMotorSpeed()
{
  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
//  servo_MidMotor.writeMicroseconds(ui_Mid_Motor_Speed);
}




void driveStraight() //function to update speed of motors using PID controllers for both motors
{
  dl_Ticks = encoder_RightMotor.getRawPosition();
  dr_Ticks = encoder_LeftMotor.getRawPosition();

  error_Left = dl_Ticks - setpoint; //positive if too fast, negative if too slow
  error_Right = dr_Ticks - setpoint; //positive if too fast, negative if too slow

  derivative_Left = error_Left - last_Error_Left;
  derivative_Right = error_Right - last_Error_Right;

  integral_Left += error_Left;
  integral_Right += error_Right;

  last_Error_Left = error_Left;
  last_Error_Right = error_Right;

  correction_Left = (kp * error_Left) + (ki * error_Left) + (kd * error_Right);
  correction_Right = (kp * error_Right) + (ki * error_Right) + (kd * error_Right);

  ui_Left_Motor_Speed -= correction_Left; // positive correction means motor is too fast
  ui_Right_Motor_Speed -= correction_Right;

  encoder_RightMotor.zero();
  encoder_LeftMotor.zero();
}










  





