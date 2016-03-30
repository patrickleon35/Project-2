#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

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


// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
//#define DEBUG_MOTOR_CALIBRATION


//other variables

boolean startRunning = false;
long prevTimer = 0;
int sensorDetected;
int state = 1;
boolean correctPosition = false;
int currentAngle = 0;

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
int leverCounter = 0;





//port pin constants


const int ci_Start_Button = 6;
const int ci_Hall_Motor = 2;

const int ci_Left_Motor = 9; //correct
const int ci_Right_Motor = 10; //correct
const int ci_Mid_Motor = 11; //correct
const int ci_Lever_Motor = 12; //correct  
const int ci_Elevator_Motor = 13; 
const int ci_Arm_Motor = 5; //correct
const int ci_Grip_Motor = 4; //correct
const int ci_Mode_Switch = 7;
const int ci_Lever_Motor = 8;
const int ci_Hall_Sensor_0 = A0;
const int ci_Hall_Sensor_1 = A1;
const int ci_Hall_Sensor_2 = A2;
const int ci_Hall_Sensor_3 = A3;
const int ci_Light_Sensor = A4;
const int ci_Left_Button = 8; //correct
const int ci_Right_Button = 9; //correct
const int ci_Ultrasonic_Input = 6; //correct
const int ci_Ultrasoncic_Output = 7;  //correct


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
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
unsigned int ui_Mid_Motor_Speed;
unsigned int ui_Elevator_Motor_Speed;
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



void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  pinMode(ci_Start_Button, INPUT);
  digitalWrite(ci_Start_Button, HIGH);


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

  //set up elevator motor
  pinMode(ci_Elevator_Motor, OUTPUT);
  servo_ElevatorMotor.attach(ci_Elevator_Motor);

  //set up hall motor
  pinMode(ci_Hall_Motor, OUTPUT)
  servo_HallMotor.attach(ci_Hall_Motor);

  //set up left button
  pinMode(ci_Left_Button, INPUT);
  digitalWrite(ci_Left_Button, HIGH);

  //set up hall sensors
  pinMode(ci_Hall_Sensor_0, INPUT);
  pinMode(ci_Hall_Sensor_1, INPUT);
  pinMode(ci_Hall_Sensor_2, INPUT);
  pinMode(ci_Hall_Sensor_3, INPUT);

  / set up light sensor
  pinMode(ci_Light_Sensor, INPUT);

  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_MidMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_MidMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_LeverMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeverMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_ElevatorMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_ElevatorMotor.setReversed(true);  // adjust for positive count when moving forward
}


void loop()
{
  if (digitalRead(ci_Start_Button) == LOW)
  {
    startRunning = true;
  }

  //mode 0
  if (currentMode() == 0)
  {
    raiseLever();
    hallExtend();
    servo_LeftMotor.writeMicroseconds(1500);
    servo_MidMotor.writeMicroseconds(1500);
    servo_RightMotor.writeMicroseconds(1500);
    servo_ArmMotor.write(ci_Arm_Servo_Retracted);
    servo_GripMotor.write(ci_Grip_Motor_Closed);
    encoder_LeftMotor.zero();
    encoder_RightMotor.zero();
    encoder_MidMotor.zero();
  }


  //mode 1
  if (currentMode() == 1)
  {
    raiseLever();

    if (fluxDetected() == true && state == 1)
    {
      state = 2;
      encoder_RightMotor.zero();
      encoder_LeftMotor.zero();
      encoder_MidMotor.zero();
    }

    if (state == 1)
    {
      if (millis() - prevTimer > 100)
      {
        driveStraight();
        prevTimer = millis();
      }
    }

    if (state == 2)
    {
      pickupTesseract1();
      if (correctPosition)
      {
        state = 3;
      }
    }

    if (state == 3)
    {
      pickupTesseract2();
    }
  }



  //mode 2
  if (currentMode() == 2)
  {

    


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
  else return false;
}

int currentMode()
{
  if (startRunning == true)
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
  else return 0;
}

void updateMotorSpeed()
{
  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
  servo_MidMotor.writeMicroseconds(ui_Mid_Motor_Speed);
  servo_LeverMotor.writeMicroseconds(ui_Lever_Motor_Speed);
  servo_ElevatorMotor.writeMicroseconds(ui_Elevator_Motor_Speed);
}

void driveStraight()
{
  ui_Left_Motor_Speed = 1890;
  ui_Right_Motor_Speed = 1820;
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
    ui_Mid_Motor_Speed = 1300;
    ui_Right_Motor_Speed = 1500;
    ui_Left_Motor_Speed = 1500;
  }
  if (dir == 1)
  {
    ui_Mid_Motor_Speed = 1700;
    ui_Right_Motor_Speed = 1500;
    ui_Left_Motor_Speed = 1500;
  }
}

void dimeTurn(boolean dir) //dir = 0 -> right, dir = 1 -> left, turns robot 90 degrees
{
  if (dir == 1)
  {
    if (encoder_RightMotor.getRawPosition() < 380)
    {
      ui_Right_Motor_Speed = 1700;
    }
    if (encoder_LeftMotor.getRawPosition() > -292)
    {
      ui_Left_Motor_Speed = 1300;
    }
    if (encoder_RightMotor.getRawPosition() >= 380)
    {
      ui_Right_Motor_Speed = 1500;
    }
    if (encoder_LeftMotor.getRawPosition() <= -292)
    {
      ui_Left_Motor_Speed = 1500;
    }
  }
  if (dir == 1)
  {
    if (encoder_LeftMotor.getRawPosition() < 353)
    {
      ui_Left_Motor_Speed = 1755;
    }
    if (encoder_RightMotor.getRawPosition() > -283)
    {
      servo_RightMotor.writeMicroseconds(1300);
    }
    if (encoder_LeftMotor.getRawPosition() >= 353)
    {
      ui_Left_Motor_Speed = 1500;
    }
    if (encoder_RightMotor.getRawPosition() <= -283)
    {
      ui_Right_Motor_Speed = 1500;
    }
  }
}


















void pickupTesseract1() //should bring the bot in the correct horizonral orientation, still need to determine values
{
  if (sensorDetected == 0)
  {
    if (encoder_MidMotor.getRawPosition() < 100)
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
    if (encoder_MidMotor.getRawPosition() < 200)
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
    if (encoder_MidMotor.getRawPosition() < 100)
    {
      driveSideways(1);
    }
    else
    {
      correctPosition = true;
    }
  }

  if (sensorDetected == 3)
  {
    if (encoder_MidMotor.getRawPosition() < 100)
    {
      driveSideways(1);
    }
    else
    {
      correctPosition = true;
    }
  }

  if (sensorDetected == 4)
  {
    if (encoder_MidMotor.getRawPosition() < 100)
    {
      driveSideways(1);
    }
    else
    {
      correctPosition = true;
    }
  }
}

void pickupTesseract2() //should pick the tesseract up, for now just stop the robot
{
  ui_Right_Motor_Speed = 1500;
  ui_Left_Motor_Speed = 1500;
}





void raiseLever()
{
  currentAngle = encoder_LeverMotor.getRawPosition();
  if (currentAngle < 60 && counter == 0)
  {
    ui_Lever_Motor_Speed = 1850;
  }
  if (currentAngle < 129 && currentAngle > 60 && counter == 0)
  {
    counter = 1;
  }
  if (currentAngle > 129 && counter == 0)
  {
    counter = 1;
  }
  if (counter == 1)
  {
    error = 129 - currentAngle;
    correction = kl * error + 1500;
    ui_Lever_Motor_Speed = correction;
  }
}

void hallExtend()
{
  servo_Hall.write(145);
}

void hallRetract()
{
  servo_Hall.write(30);
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
  servo_armMotor.write(180);
}

void armUp()
{
  servo_armMotor.write(5);
}




boolean frontWallDetected()
{
  Ping();
  if (ul_Echo_Time/58 < 3)
  {
    return true;
  }
  else return false;
}


boolean leftWallDetected()
{
  if (digitalRead(ci_Left_Button == LOW)
{
  return true;
}
else
{
  return false;
}

}

boolean rightWallDetected()
{
  if (digitalRead(ci_Right_Button == LOW)
{
  return true;
}
else
{
  return false;
}
}



boolean lineDetected()
{


}



void reZero()
{

}

void raiseClaw() //implement once get motor controller
{
  ui_Elevator_Motor_Speed = 1650;
}

void lowerClaw()  //implement once get motor controller
{
  ui_Elevator_Motor_Speed = 1350;
}


void dropOffPosition() //puts lever and claw in correct position for drop off
{
    
}







void ultrasonicLeft()
{
}

void ultrasonicRight()
{
}

void ultrasonicStraight()
{
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
  Serial.print(ul_Echo_Time/148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time/58); //divide time by 58 to get distance in cm 
#endif
}  








//******************************************Mode 2 Function****************************************//


void scanner() // function to wait and scan for tesseracts to be dropped off
{
  
  
}
















