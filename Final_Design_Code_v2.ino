na#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <I2CEncoder.h>
#include <Wire.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_MidMotor;
Servo servo_ElevatorMotor;
Servo servo_ArmMotor;
Servo servo_GripMotor;
Servo servo_LeverMotor;
Servo servo_HallMotor;


I2CEncoder encoder_LeftMotor;
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_MidMotor;
I2CEncoder encoder_LeverMotor;
I2CEncoder encoder_ElevatorMotor;

//scan variables

int scan_State = 1;
int scan_counter1 = 0;
int scan_counter2 = 0;
long scan_timer1 = 0;
long scan_timer2 = 0;
long straight_timer = 0;
int straight_counter = 0;
int counter4 = 0;
int state8_counter = 0;
long state6_Timer = 0;
int scanDirection = 0;

int absolute_y = 0;
int absolute_x = 0;


int numberDropped = 0;


// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
//#define DEBUG_MOTOR_CALIBRATION

//timer variables

long dropOffPosition_timer = 0;

//other variables

boolean lastFluxDetected = false;

boolean startRunning = false;
long prevTimer = 0;
int sensorDetected;
int mode_1_State = 1;
int dropOff_State = 1;
boolean correctPosition = false;
int currentAngle = 0;
boolean dimeTurnComplete = false;


int initialHall_0;
int initialHall_1;
int initialHall_2;
int initialHall_3;


boolean elevator_at_bottom;
boolean elevator_at_middle;
boolean elevator_at_top;
boolean raiseLeverFinished = false;
boolean elevatorMiddleFinished = false;
boolean elevatorTopFinished = false;
boolean elevatorDropOffFinished = false;
boolean elevatorDrivingFinished = false;
//*******************************STATE DEFINITIONS*************************************//

/* state 1 -- driving straight
   state 2 -- driving sideways to correct orientation
   state 3 -- driving forward to pick up tesseract
   state 4 -- driving back to home base
   state 5 -- dropping off tesseract
*/
//************************************************************************************//


//counter variables
int counter1 = 0;
int lever_counter = 0;
int lever_error = 0;
int lever_correction = 0;
int rezero_counter1 = 0;
int rezero_counter2 = 0;
int detachcounter = 0;
int attachcounter = 0;
int fluxDetectedCounter = 0;
int state2_counter = 0;
int state2_counter2 = 0;





//port pin constants


const int ci_Start_Button = 6;
const int ci_Hall_Motor = 2;

const int ci_Left_Motor = 11; //correct
const int ci_Right_Motor = 12; //correct
const int ci_Mid_Motor = 13; //correct
const int ci_Lever_Motor = 9; //correct
const int ci_Elevator_Motor = 10; // correct
const int ci_Arm_Motor = 5; //correct
const int ci_Grip_Motor = 4; //correct
const int ci_Back_Button = 3; //correct
const int ci_Mode_Switch = 7;
const int ci_Hall_Sensor_0 = A0;
const int ci_Hall_Sensor_1 = A1;
const int ci_Hall_Sensor_2 = A2;
const int ci_Hall_Sensor_3 = A3;
const int ci_Light_Sensor = A4; // correct
const int ci_Left_Button = 8; //correct

const int ci_Ultrasonic_Input = 6; //correct
const int ci_Ultrasonic_Output = 7;  //correct


//constants

const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Mid_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 140;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Closed = 90;
const int ci_Lever_Motor_Up = 180;
const int ci_Lever_Motor_Down = 90;
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
unsigned int ui_Lever_Motor_Speed = 1500;
unsigned int ui_Left_Motor_Speed = 1500;
unsigned int ui_Right_Motor_Speed = 1500;
unsigned int ui_Mid_Motor_Speed = 1500;
unsigned int ui_Elevator_Motor_Speed = 1500;
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
int setpoint = 20;
int last_Error_Left;
int last_Error_Right;
int correction_Right;
int correction_Left;
int l_Ticks = 0;
int r_Ticks = 0;
int last_l_Ticks = 0;
int last_r_Ticks = 0;
int kp = 2;
int ki = .9;
int kd = 1;
int kl = 20;


//straight sub counterts

int straight_sub_counter1 = 0;
int straight_sub_counter2 = 0;



void setup()
{
  delay(5000);
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);




  // set up drive motors
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Mid_Motor, OUTPUT);
  servo_MidMotor.attach(ci_Mid_Motor);

  //set up lever arm
  pinMode(ci_Lever_Motor, OUTPUT);
  servo_LeverMotor.attach(ci_Lever_Motor);

  //set up elevator motor
  pinMode(ci_Elevator_Motor, OUTPUT);
  servo_ElevatorMotor.attach(ci_Elevator_Motor);

  // set up arm motors
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);
  pinMode(ci_Grip_Motor, OUTPUT);
  servo_GripMotor.attach(ci_Grip_Motor);



  //set up hall motor
  pinMode(ci_Hall_Motor, OUTPUT);
  servo_HallMotor.attach(ci_Hall_Motor);

  //set up left button
  pinMode(ci_Left_Button, INPUT);
  digitalWrite(ci_Left_Button, HIGH);
  pinMode(ci_Back_Button, INPUT);
  digitalWrite(ci_Back_Button,HIGH);
  

  //set up hall sensors
  pinMode(ci_Hall_Sensor_0, INPUT);
  pinMode(ci_Hall_Sensor_1, INPUT);
  pinMode(ci_Hall_Sensor_2, INPUT);
  pinMode(ci_Hall_Sensor_3, INPUT);




  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_MidMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_MidMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_LeverMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeverMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_ElevatorMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_ElevatorMotor.setReversed(false);  // adjust for positive count when moving forward


  initialHall_0 = analogRead(A0);
  initialHall_1 = analogRead(A1);
  initialHall_2 = analogRead(A2);
  initialHall_3 = analogRead(A3);

}


void loop()
{
  
  Serial.print(encoder_LeftMotor.getRawPosition());
  Serial.print("   ");
  Serial.print(encoder_MidMotor.getRawPosition());
  Serial.print("   ");
  Serial.println(encoder_RightMotor.getRawPosition());
  
  startPosition();
  if (millis() > 10000)
  {
    raiseLever();

    if (mode_1_State == 1)
    {
      scan();

      if (fluxDetected())
      {
        mode_1_State = 2;
        encoder_RightMotor.zero();
        encoder_LeftMotor.zero();
        encoder_MidMotor.zero();
        stopDriveMotors();
        state2_counter = millis();
      }
    }

    if (mode_1_State == 2)
    {
      if ((millis() - state2_counter) < 3000)
      {
        stopDriveMotors();
      }
      if ((millis() - state2_counter) > 3000)
      {
        pickupTesseract1();
        if (correctPosition)
        {
          stopDriveMotors();
          mode_1_State = 3;
          encoder_RightMotor.zero();
          encoder_LeftMotor.zero();
          encoder_MidMotor.zero();
          state2_counter2 = millis();
        }
      }

    }




    if (mode_1_State == 3) //drive straight and grip the tesseract
    {

      if ((millis() - state2_counter2) < 2000)
      {
        stopDriveMotors();
        elevatorBottom();
      }
      if ((millis() - state2_counter2) > 2000)
      {
        if (encoder_RightMotor.getRawPosition() < 300)
        {
          driveStraightOffset();
        }
        
        if (encoder_RightMotor.getRawPosition() > 300)
        {
          mode_1_State = 4;
        }
      }
    }




    if (mode_1_State == 4)
    {
      clawClose();
      stopDriveMotors();
      elevatorDropOff();
      if (elevatorDropOffFinished)
      {
        mode_1_State = 5;
      }
    }

    if (mode_1_State == 5)
    {
      if (!leftWallDetected())
      {
        driveSideways(1);
      }
      if (leftWallDetected())
      {
        mode_1_State = 6;
        ui_Mid_Motor_Speed = 1500;
      }
      if (backWallDetected() && leftWallDetected())
      {
        mode_1_State = 7;
        state6_Timer = millis();
        encoder_MidMotor.zero();  
      }
      
    }

    if (mode_1_State == 6)
    {
      
   
      if (!backWallDetected())
      {
        driveBackwards();
      }
      if (backWallDetected())
      {
        mode_1_State = 5;
      }
      if (backWallDetected() && leftWallDetected())
      {
        mode_1_State = 7;
        state6_Timer = millis();
        encoder_MidMotor.zero();
      }
     
  

    }

    if (mode_1_State == 7)
    {
      
      if ((millis() - state6_Timer) < 2000)
      {
        stopDriveMotors();
      }
      if ((millis() - state6_Timer) > 2000)
      {
        if (encoder_MidMotor.getRawPosition() > -800)
        {
          driveSideways(0);
        }
        if (encoder_MidMotor.getRawPosition() <= -800)
        {
          mode_1_State = 8;
          ui_Mid_Motor_Speed = 1500;
          encoder_RightMotor.zero();
          encoder_LeftMotor.zero();
        }
      }
    }

    if (mode_1_State == 8)
    {
     
      if (encoder_RightMotor.getRawPosition() < 300)
      {
        driveStraightOffset();
      }
      if(encoder_RightMotor.getRawPosition() >= 300)
      {
        mode_1_State = 9;
        state8_counter = millis();
      }
    }

    if (mode_1_State == 9)
    {
      if ((millis() - state8_counter) < 2000)
      {
        stopDriveMotors();
      }
      if ((millis() - state8_counter) > 2000)
      {
        dimeTurn(1);
        if (dimeTurnComplete)
        {
          mode_1_State = 10;
        }
      }
      
    }

    if (mode_1_State == 10)
    {
      stopDriveMotors();
      hallRetract();
      if (!frontWallDetected())
      {
        driveStraightOffset();
      }
      if (frontWallDetected())
      {
        if (!leftWallDetected())
        {
          driveSideways(1);
        }
        if (leftWallDetected())
        {
          mode_1_State = 11;
        }
      }
    }

    if (mode_1_State == 11)
    {
      stopDriveMotors();    
    }
    

 

  }
    




  updateMotorSpeed();

}



void scan()
{
  if (scan_State == 1)
  {
    driveStraight();
    if (frontWallDetected())
    {
      scan_State = 2;
    }
  }
  
  if (scan_State == 2)
  {
    driveBackwards();
    if (backWallDetected())
    {
      scan_State = 3;
      encoder_MidMotor.zero();
    }
  }

  if (scan_State == 3)
  {
    if (encoder_MidMotor.getRawPosition() > -800)
    {
      driveSideways(scanDirection);
      absolute_x -= encoder_MidMotor.getRawPosition();
    }
    if (encoder_MidMotor.getRawPosition() <= -800)
    {
      scan_State = 1;
    }
  }


  if (atLine())
  {
    scan_State = 1;
    scanDirection = 1;
  }

  if (leftWallDetected()
  {
    scanDirection = 0;
    scan_State = 1;    
  }
}
    

  


  

  
    

  
      
    
    
  
  












  boolean fluxDetected()
  {
    if (analogRead(ci_Hall_Sensor_0) > (initialHall_0 + 7) || analogRead(ci_Hall_Sensor_0) < (initialHall_0 - 7))
    {
      sensorDetected = 0;
      return true;
    }
    if (analogRead(ci_Hall_Sensor_1) > (initialHall_1 + 7) || analogRead(ci_Hall_Sensor_1) < (initialHall_1 - 7))
    {
      sensorDetected = 1;
      return true;
    }
    if (analogRead(ci_Hall_Sensor_2) > (initialHall_2 + 7) || analogRead(ci_Hall_Sensor_2) < (initialHall_2 - 7))
    {
      sensorDetected = 2;
      return true;
    }
    if (analogRead(ci_Hall_Sensor_3) > (initialHall_3 + 7) || analogRead(ci_Hall_Sensor_3) < (initialHall_3 - 7))
    {
      sensorDetected = 3;
      return true;
    }
    else
    {
      return false;
      sensorDetected = 0;
    }
  }


 void atLine()
 {
  if (absolute_x > ((absolute_y - 871) / 2))
  {
    return true;
  }
  if (absolute_x < ((absolute_y - 871) / 2))
  {
    return false;
  }
 }







  void updateMotorSpeed()
  {
    servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
    servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
    servo_MidMotor.writeMicroseconds(ui_Mid_Motor_Speed);
    servo_LeverMotor.writeMicroseconds(ui_Lever_Motor_Speed);
    servo_ElevatorMotor.writeMicroseconds(ui_Elevator_Motor_Speed);
  }

  void driveStraightOffset()
  {
    
    ui_Left_Motor_Speed = 1740;
    ui_Right_Motor_Speed = 1685;
    
  }

  void driveStraight()
  {
   if (straight_counter == 0) //do once
  {
    ui_Left_Motor_Speed = 1740;
    ui_Right_Motor_Speed = 1685;
    straight_counter = 1;
  }

  if ((millis() - straight_timer) > 100)
  {
    ui_Left_Motor_Speed = 1740;
    setpoint = encoder_LeftMotor.getRawPosition();
    if (setpoint > encoder_RightMotor.getRawPosition())
    {
      ui_Right_Motor_Speed++;
    }
    if (setpoint < encoder_RightMotor.getRawPosition())
    {
      ui_Right_Motor_Speed--;
    }
    
    encoder_LeftMotor.zero();
    encoder_RightMotor.zero();
    straight_timer = millis();
  }

    
  }

  void driveBackwards()
  {
    ui_Left_Motor_Speed = 1240;
    ui_Right_Motor_Speed = 1270;
  }

  void driveSideways(boolean dir) //0 = right, 1 = left
  {
    if (dir == 0)
    {
      ui_Mid_Motor_Speed = 1700;
      ui_Right_Motor_Speed = 1500;
      ui_Left_Motor_Speed = 1500;
    }
    if (dir == 1)
    {
      ui_Mid_Motor_Speed = 1300;
      ui_Right_Motor_Speed = 1500;
      ui_Left_Motor_Speed = 1500;
    }
  }

  void dimeTurn(boolean dir) //dir = 0 -> right, dir = 1 -> left, turns robot 90 degrees. Encoders must be zeroed before calling
  {
    if (dir == 1) //left
    {
      if (encoder_RightMotor.getRawPosition() < 380)
      {
        ui_Right_Motor_Speed = 1700;
        dimeTurnComplete = false;

      }
      if (encoder_LeftMotor.getRawPosition() > -292)
      {
        ui_Left_Motor_Speed = 1300;
        dimeTurnComplete = false;
      }
      if (encoder_RightMotor.getRawPosition() >= 380)
      {
        ui_Right_Motor_Speed = 1500;
      }
      if (encoder_LeftMotor.getRawPosition() <= -292)
      {
        ui_Left_Motor_Speed = 1500;
      }
      if (encoder_LeftMotor.getRawPosition() <= -292 && encoder_RightMotor.getRawPosition() >= 380)
      {
        dimeTurnComplete = true;
      }
    }
    if (dir == 0)  //right
    {
      if (encoder_LeftMotor.getRawPosition() < 353)
      {
        ui_Left_Motor_Speed = 1755;
        dimeTurnComplete = false;
      }
      if (encoder_RightMotor.getRawPosition() > -283)
      {
        ui_Right_Motor_Speed = 1300;
        dimeTurnComplete = false;
      }
      if (encoder_LeftMotor.getRawPosition() >= 353)
      {
        ui_Left_Motor_Speed = 1500;
      }
      if (encoder_RightMotor.getRawPosition() <= -283)
      {
        ui_Right_Motor_Speed = 1500;
      }
      if (encoder_RightMotor.getRawPosition() <= -283 && encoder_LeftMotor.getRawPosition() >= 353)
      {
        dimeTurnComplete = true;
      }
    }
  }


















  void pickupTesseract1() //should bring the bot in the correct horizonral orientation, still need to determine values
  {
    if (sensorDetected == 0)
    {
      if (encoder_MidMotor.getRawPosition() < 320)
      {
        driveSideways(1);
      }
      else
      {
        correctPosition = true;
      }
    }

    if (sensorDetected == 1)
    {
      if (encoder_MidMotor.getRawPosition() < 160)
      {
        driveSideways(1);
      }
      else
      {
        correctPosition = true;
      }
    }

    if (sensorDetected == 2)
    {
      if (encoder_MidMotor.getRawPosition() > -160)
      {
        driveSideways(0);
      }
      else
      {
        correctPosition = true;
      }
    }

    if (sensorDetected == 3)
    {
      if (encoder_MidMotor.getRawPosition() > -260)
      {
        driveSideways(0);
      }
      else
      {
        correctPosition = true;
      }
    }
  }


  void pickupTesseract2() 
  {

    if (encoder_RightMotor.getRawPosition() < 150 && encoder_LeftMotor.getRawPosition() < 150)
    {
        ui_Left_Motor_Speed = 1740;
        ui_Right_Motor_Speed = 1685;
    }
    else
    {
      ui_Right_Motor_Speed = 1500;
      ui_Left_Motor_Speed = 1500;
      clawClose();
      encoder_RightMotor.zero();
      encoder_LeftMotor.zero();
      encoder_MidMotor.zero();
      mode_1_State = 4;
    }
  }





    void raiseLever()
  {

    currentAngle = encoder_LeverMotor.getRawPosition();
    if (currentAngle < 60 && lever_counter == 0)
    {
      ui_Lever_Motor_Speed = 1950;
    }
    if (currentAngle < 120 && currentAngle > 60 && lever_counter == 0)
    {
      ui_Lever_Motor_Speed = 1700;
    }
    if (currentAngle > 120 && lever_counter == 0)
    {
      lever_counter = 1;
    }
    if (lever_counter == 1)
    {
      raiseLeverFinished = true;
      lever_error = 120 - currentAngle;
      lever_correction = kl * lever_error + 1500;
      ui_Lever_Motor_Speed = lever_correction;
    }
  }


  void stopDriveMotors()
  {
    ui_Right_Motor_Speed = 1500;
    ui_Left_Motor_Speed = 1500;
    ui_Mid_Motor_Speed = 1500;
  }

  void hallExtend()
  {
    servo_HallMotor.write(168);
  }

  void hallRetract()
  {
    servo_HallMotor.write(0);
  }

  void clawOpen()
  {
    servo_GripMotor.write(180);
  }

  void clawClose()
  {
    servo_GripMotor.write(115);
  }

  void armDown()
  {
    servo_ArmMotor.write(165);
  }

  void armUp()
  {
    servo_ArmMotor.write(0);
  }



  boolean frontWallDetected()
  {
    Ping();
    if (ul_Echo_Time / 58 < 3)
    {
      return true;
    }
    else return false;
  }


  boolean leftWallDetected()
  {
    if (digitalRead(ci_Left_Button) == 0)
    {
      return true;
    }
    if (digitalRead(ci_Left_Button == 1))
    {
      return false;
    }

  }

  boolean backWallDetected()
  {
    if (digitalRead(ci_Back_Button) == LOW)
    {
      return true;
    }
    if (digitalRead(ci_Back_Button) == HIGH)
    {
      return false;
    }
  }


  void raiseClaw() //implement once get motor controller
  {
    ui_Elevator_Motor_Speed = 1650;
  }

  void lowerClaw()  //implement once get motor controller
  {
    ui_Elevator_Motor_Speed = 1350;
  }



  void elevatorBottom()
  {
    if (encoder_ElevatorMotor.getRawPosition() > 5)
    {
      ui_Elevator_Motor_Speed = 1250;
    }
    else if (encoder_ElevatorMotor.getRawPosition() <= 5)
    {
      ui_Elevator_Motor_Speed = 1500;
    }

  }

  void elevatorMiddle()
  {

    if (encoder_ElevatorMotor.getRawPosition() < 1000)
    {
      ui_Elevator_Motor_Speed = 1750;
    }
    else if (encoder_ElevatorMotor.getRawPosition() >= 1000)
    {
      ui_Elevator_Motor_Speed = 1500;
    }
  }



  void elevatorTop()
  {
    if (encoder_ElevatorMotor.getRawPosition() < 3360)
    {
      ui_Elevator_Motor_Speed = 1750;
    }
    else if (encoder_ElevatorMotor.getRawPosition() >= 3360)
    {
      ui_Elevator_Motor_Speed = 1500;
      elevatorTopFinished = true;
    }
  }

  void elevatorDropOff()
  {
    if (encoder_ElevatorMotor.getRawPosition() < 2500)
    {
      ui_Elevator_Motor_Speed = 1750;
    }
    else if (encoder_ElevatorMotor.getRawPosition() >= 2500)
    {
      ui_Elevator_Motor_Speed = 1500;
      elevatorDropOffFinished = true;
    }

  }

  void elevatorDriving()
  {
    if (encoder_ElevatorMotor.getRawPosition() < 500)
    {
      ui_Elevator_Motor_Speed = 1750;
    }
    else if (encoder_ElevatorMotor.getRawPosition() >= 500)
    {
      ui_Elevator_Motor_Speed = 1500;
      elevatorDrivingFinished = true;
    }

  }


  void detachMotors()
  {
    servo_RightMotor.detach();
    servo_LeftMotor.detach();
    servo_MidMotor.detach();
    servo_ElevatorMotor.detach();
    servo_HallMotor.detach();
    servo_ArmMotor.detach();
    servo_GripMotor.detach();
  }

  void attachMotors()
  {
    servo_RightMotor.attach(ci_Right_Motor);
    servo_LeftMotor.attach(ci_Left_Motor);
    servo_MidMotor.attach(ci_Mid_Motor);
    servo_ElevatorMotor.attach(ci_Elevator_Motor);
    servo_HallMotor.attach(ci_Hall_Motor);
    servo_ArmMotor.attach(ci_Arm_Motor);
    servo_GripMotor.attach(ci_Grip_Motor);
  }


  void startPosition()
  {
    if (millis() < 6500)
    {
      armDown();
      clawOpen();
      hallExtend();
    }

    if (millis() < 10000 && millis() > 6500)
    {
      if (detachcounter == 0)
      {
        detachMotors();
        detachcounter = 1;
      }
      raiseLever();
      if (raiseLeverFinished == true)
      {
        attachMotors();
        elevatorDriving();

      }
    }
  }

  void dropOffPosition()
  {
    if (dropOff_State == 1)
    {
      elevatorDropOff();
    }
    if (elevatorDropOffFinished == true)
    {
      dropOff_State = 2;
    }
    if (dropOff_State == 2)
    {
      hallRetract();
      dropOff_State = 3;
    }
    if (dropOff_State == 3)
    {
      clawOpen();
    }


  }

  void printReadings()
  {
    Serial.print(analogRead(A0));
    Serial.print("   ");
    Serial.print(analogRead(A1));
    Serial.print("   ");
    Serial.print(analogRead(A2));
    Serial.print("   ");
    Serial.println(analogRead(A3));

  }





  void Ping()
  {
    digitalWrite(ci_Ultrasonic_Input, HIGH);
    delayMicroseconds(10);
    digitalWrite(ci_Ultrasonic_Input, LOW);
    ul_Echo_Time = pulseIn(ci_Ultrasonic_Output, HIGH, 10000);

#ifdef DEBUG_ULTRASONIC
    Serial.print("Time (microseconds): ");
    Serial.print(ul_Echo_Time, DEC);
    Serial.print(", Inches: ");
    Serial.print(ul_Echo_Time / 148); //divide time by 148 to get distance in inches
    Serial.print(", cm: ");
    Serial.println(ul_Echo_Time / 58); //divide time by 58 to get distance in cm
#endif
  }






