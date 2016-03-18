#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>


Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_MidMotor;
Servo servo_ArmMotor; 
Servo servo_LeverMotor;   
Servo servo_GripMotor;


I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
I2CEncoder encoder_MidMotor;



//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_MOTOR_CALIBRATION

boolean bt_Motors_Enabled = true;

//port pin constants

const int ci_Ultrasonic_Data = 3;   //output plug
const int ci_Mode_Button = 6;
const int ci_Mode_Switch = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Mid_Motor = 10;
const int ci_Arm_Motor = 11,
const int ci_LeverMotor = 12;
const int ci_Motor_Enable_Switch = 13;
const int ci_Hall_Sensor_0 = A0;
const int ci_Hall_Sensor_1 = A1;
const int ci_Hall_Sensor_2 = A2;
const int ci_Hall_Sensor_3 = A3;
const int ci_Hall_Sensor_4 = A4;
const int ci_Hall_Sensor_5 = A5;


//constants

// EEPROM addresses
const int ci_Left_Line_Tracker_Dark_Address_L = 0;
const int ci_Left_Line_Tracker_Dark_Address_H = 1;
const int ci_Left_Line_Tracker_Light_Address_L = 2;
const int ci_Left_Line_Tracker_Light_Address_H = 3;
const int ci_Middle_Line_Tracker_Dark_Address_L = 4;
const int ci_Middle_Line_Tracker_Dark_Address_H = 5;
const int ci_Middle_Line_Tracker_Light_Address_L = 6;
const int ci_Middle_Line_Tracker_Light_Address_H = 7;
const int ci_Right_Line_Tracker_Dark_Address_L = 8;
const int ci_Right_Line_Tracker_Dark_Address_H = 9;
const int ci_Right_Line_Tracker_Light_Address_L = 10;
const int ci_Right_Line_Tracker_Light_Address_H = 11;
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Mid_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 140;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Closed = 90; //  "
const int ci_Arm_Motor_Bottom = 55;  //  "
const int ci_Arm_Motor_Top = 180;    //   "
const int ci_Lever_Motor_Up = 180;   //   "
const int ci_Lever_Motor_Down = 55;    //  "
const int ci_Display_Time = 500;
const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;

//variables
byte b_LowByte;
byte b_HighByte;

unsigned long u1_Echo_Time = 0;
unsigned int ui_Motors_Speed = 1750;      // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
unsigned int ui_Mid_Motor_Speed;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Cal_Cycle;

unsigned int  ui_Robot_State_Index = 0; //0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;
boolean startRunning = false;

//other variables
int sensorDetected;
int state = 1;

void setup() {
  Wire.begin();	      // Wire library required for I2CEncoder library
  Serial.begin(9600);

  // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);
  pinMode(ci_Mid_Motor, OUTPUT);
  servo_MidMotor.attach(ci_Mid_Motor);

  // set up arm motors
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);
  pinMode(ci_Grip_Motor, OUTPUT);
  servo_GripMotor.attach(ci_Grip_Motor);
  pinMode(ci_Lever_Motor, OUTPUT);
  servo_LeverMotor.attach(ci_Lever_Motor);

  // set up hall sensors
  pinMode(ci_Hall_Sensor_0,INPUT);
  pinMode(ci_Hall_Sensor_1,INPUT);
  pinMode(ci_Hall_Sensor_2,INPUT);
  pinMode(ci_Hall_Sensor_3,INPUT);  
  pinMode(ci_Hall_Sensor_4,INPUT);
  pinMode(ci_Hall_Sensor_5,INPUT);
  
  // set up motor enable switch
  pinMode(ci_Motor_Enable_Switch, INPUT);

  // set up encoders. Must be initialized in order that they are chained together, 
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_MidMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_MidMotor.setReversed(false); // adjust for positive count when moving right


  // read saved values from EEPROM
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte); 
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
}



void loop()
{

  if (digitalRead(ci_Mode_Button) == HIGH)
  {
    startRunning = true;
  }

  //mode 0
  if (startRunning == false)
  {  
      Ping();
      servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
      servo_MidMotor.writeMicroseconds(ci_Mid_Motor_Stop); 
      servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop); 
      servo_ArmMotor.write(ci_Arm_Servo_Retracted);
      servo_GripMotor.write(ci_Grip_Motor_Closed);
      servo_LeverMotor.write(ci_Lever_Motor_Up);
      encoder_LeftMotor.zero();
      encoder_RightMotor.zero();
      encoder_MidMotor.zero();
      ui_Mode_Indicator_Index = 0;
  }

    
  







  //mode 1
  if (startRunning == true && currentMode() == 1)
  {
      
#ifdef DEBUG_ENCODERS           
        l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
        l_Right_Motor_Position = encoder_RightMotor.getRawPosition();
        Serial.print("Encoders L: ");
        Serial.print(l_Left_Motor_Position);
        Serial.print(", R: ");
        Serial.println(l_Right_Motor_Position);
#endif


        //********************Mode 1 code************************//          
        





      
   

        //*******************************************************//


        
        if(bt_Motors_Enabled)
        {
          servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
          servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
        }
        else
        {
          servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop); 
          servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop); 
        }
        
#ifdef DEBUG_MOTORS
        Serial.print("Motors enabled: ");
        Serial.print(bt_Motors_Enabled);
        Serial.print(", Default: ");
        Serial.print(ui_Motors_Speed);
        Serial.print(", Left = ");
        Serial.print(ui_Left_Motor_Speed);
        Serial.print(", Right = ");
        Serial.println(ui_Right_Motor_Speed);
#endif  

          
#ifdef DEBUG_MOTOR_CALIBRATION
          Serial.print("Motor Offsets: Left = ");
          Serial.print(ui_Left_Motor_Offset);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Motor_Offset);
#endif   



 
  }

  
  //mode 2
  if (startRunning == true && currentMode() == 2)
  {
    
  }






      
 

  
}
    









// ulstrasonic sensor 
void Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  u1_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);

 // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time/148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time/58); //divide time by 58 to get distance in cm 
#endif
}






boolean fluxDetected()
{
  if (analogRead(ci_Hall_Sensor_0) > 520 || analogRead(ci_Hall_Sensor_0) < 500)
  {
    sensorDetected = 0;
    return true;
  }
  if (analogRead(ci_Hall_Sensor_1) > 520 || analogRead(ci_Hall_Sensor_1) < 500)
  {
    sensorDetected = 1;
    return true;
  }
  if (analogRead(ci_Hall_Sensor_2) > 520 || analogRead(ci_Hall_Sensor_2) < 500)
  {
    sensorDetected = 2;
    return true;
  }
  if (analogRead(ci_Hall_Sensor_3) > 520 || analogRead(ci_Hall_Sensor_3) < 500)
  {
    sensorDetected = 3;
    return true;
  }
  if (analogRead(ci_Hall_Sensor_4) > 520 || analogRead(ci_Hall_Sensor_4) < 500)
  {
    sensorDetected = 4;
    return true;
  }
  if (analogRead(ci_Hall_Sensor_5) > 520 || analogRead(ci_Hall_Sensor_5) < 500)
  {
    sensorDetected = 5;
    return true;
  }
  else return false;
}


int currentMode()
{
  if (digitalRead(ci_Mode_Switch) == LOW)
  {
    return 1;
  }
  if (digitalRead(ci_Mode_Switch) == HIGH)
  {
    return 2;
  }
}



  





