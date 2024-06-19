/*
* ServoProject.cpp - Library for controlling the Dynamixel Servo on an OpenRB-150 Arduino using an analog joystick
* Created by Paul Galiasso, June 6, 2024
* For use of Hydra Service and subsequent products
*/

#include <Dynamixel2Arduino.h>
#include <ServoProject.h>
#include "Arduino.h"

#define JOYSTICK_MIN            0.0
using namespace ControlTableItem;


ServoProject::ServoProject()
{

}

ServoProject::ServoProject(int _maxServoAngle, int _minServoAngle, const uint8_t _ID, int _angle)
{
  maxServoAngle = _maxServoAngle;
  minServoAngle = _minServoAngle;
  DXL_ID = _ID;
  angle = _angle;
} 

ServoProject::ServoProject(int _joystickMax, int _deadzone, pin_size_t _signalPin, pin_size_t _centerPin)
{
  joystickMax = _joystickMax;
  joystickDeadzone = _deadzone;
  signalPin = _signalPin;
  centerPin = _centerPin;
}

void ServoProject::init(Dynamixel2Arduino dxl)
{
  if (!start && digitalRead(5))
  {
    startTime = millis();
    start = true;
    angle = 182;
    up = true;
  }
  else if (!up && abs(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE)-182) < 0.25)
  {
    start = false;
  }

}
void ServoProject::cycleMotion(Dynamixel2Arduino dxl, int velocityDecrease)
{
  // Center area moving right
  if(switchPress_4 && !switchPress_2 && !switchPress_3 && !switchPress_1)
  {
    // maximum velocity to the right
    dxl.setGoalPosition(DXL_ID, maxServoAngle, UNIT_DEGREE);
  }

  // on the right side moving right
  else if (switchPress_4 && switchPress_2 && !switchPress_3 && !switchPress_1)
  {
    // slower velocity to the right
    dxl.setGoalPosition(DXL_ID, maxServoAngle - velocityDecrease, UNIT_DEGREE);
  }

  // on the right side moving left
  else if (switchPress_1 && switchPress_2 && !switchPress_3 && !switchPress_4)
  {
    // slower velocity to the left
    dxl.setGoalPosition(DXL_ID, minServoAngle + velocityDecrease, UNIT_DEGREE);
  }

  // Center area moving left
  else if (switchPress_1 && !switchPress_3 && !switchPress_2 && !switchPress_4)
  {
    // maximum velicity to the left
    dxl.setGoalPosition(DXL_ID, minServoAngle, UNIT_DEGREE);
  }
  
  //left side moving left
  else if (switchPress_1 && switchPress_3 && !switchPress_2 && !switchPress_4)
  {
    // slower velocity to the left
    dxl.setGoalPosition(DXL_ID, minServoAngle + velocityDecrease, UNIT_DEGREE);
  }
  
  // left side moving right
  else if (switchPress_4 && switchPress_3 && !switchPress_2 && !switchPress_1)
  {
    // slower velocity to the right
    dxl.setGoalPosition(DXL_ID, maxServoAngle - velocityDecrease, UNIT_DEGREE);
  }
  
}


void ServoProject::readCyclePosition()
{

  // Reads the limit switches to find the current position. 
  if(switchPress_4 && !switchPress_3 && !switchPress_2 && !switchPress_1)
  {
    switchPress_2 = digitalRead(2);
  }
  else if (switchPress_4 && !switchPress_3 && switchPress_2 && !switchPress_1)
  {
    switchPress_1 = digitalRead(0);
    if(switchPress_1)
    {
      switchPress_4 = false;
    }
  }
  else if (!switchPress_4 && !switchPress_3 && switchPress_2 && switchPress_1)
  {
    switchPress_2 = !digitalRead(2);
  }
  else if (!switchPress_4 && !switchPress_3 && !switchPress_2 && switchPress_1)
  {
    switchPress_3 = digitalRead(3);
  }
  else if (!switchPress_4 && switchPress_3 && !switchPress_2 && switchPress_1)
  {
    switchPress_4 = digitalRead(4);
    if (switchPress_4)
    {
      switchPress_1 = false;
    }
  }
  else if (switchPress_4 && switchPress_3 && !switchPress_2 && !switchPress_1)
  {
    switchPress_3 = !digitalRead(3);
  }

  // Error readouts
  else if (switchPress_4 && switchPress_1)
  {
    Serial.print("Error: impossible direction");
  }
  else if (!switchPress_4 && !switchPress_1)
  {
    Serial.print("Error: no direction");
  }
  else if (switchPress_2 && switchPress_3)
  {
    Serial.print("Error: impossible location");
  }
}


// Uses the center tap of the potentiometer to calibrate the joystick
void ServoProject::calibrate ()
{
  int middleVal = analogRead(centerPin);
  int maxVal = middleVal * 2;

  // back up in case that something with the center tap pin goes wrong
  if (abs(maxVal - joystickMax) >= 100 || counter > 500)
  {
    counter = 0;
    joystickMax = maxVal;
  }
  else
  {
    counter++; 
  }
}

int ServoProject::setDeadzone(int joyVal)
{
  // Find the values that the deadzone cuts off
  float high_deadzone_val = (joystickMax / 2) + joystickDeadzone;
  float low_deadzone_val = (joystickMax / 2) - joystickDeadzone;

  // Find the coefficient to adjust the values with
  float high_deadzone_coef = (joystickMax - (joystickMax / 2)) / (joystickMax - high_deadzone_val);
  float low_deadzone_coef = (JOYSTICK_MIN - (joystickMax / 2)) / (JOYSTICK_MIN - low_deadzone_val);

// Scales the value from the deadzone to max
  if (joyVal >= high_deadzone_val)
  {
    joyVal = high_deadzone_coef * (joyVal - high_deadzone_val) + (joystickMax / 2);
  }
  else if (joyVal <= low_deadzone_val)
  {
    joyVal = low_deadzone_coef * (joyVal - low_deadzone_val) + (joystickMax / 2);
  }
  else
  {
    joyVal = joystickMax / 2;
  }
  return joyVal;
}

// Prints the CSV of the time and the servo angle
void ServoProject::printTable(int time, Dynamixel2Arduino dxl)
{
    Serial.print(time / 1000);
    Serial.print(",");
    Serial.print(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - 182);
    Serial.println();
}

// increments the desired angle and decreses back to 0 when having reached 30 degrees
int ServoProject::runThrough(bool forwards, Dynamixel2Arduino dxl, int value)
{
  if (abs(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - 182) >= 29)
  {
    up = false;
  }

  if (forwards)
  {
    up ? value++ : value--;
  }
  else
  {
    up ? value-- : value++;
  }
  return value;
}

void ServoProject::autoLoop(bool forwards, Dynamixel2Arduino dxl)
{
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 20);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 5);
  if (forwards)
  {
    //Serial.print("test");
    if (up)
    {
      dxl.setGoalPosition(DXL_ID, maxServoAngle, UNIT_DEGREE);
      //Serial.print("chicken");
      if(abs(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - maxServoAngle) < 3)
      {
        up = false;
        Serial.println("test");
      }
    }
    else
    {
      Serial.print(up);
      dxl.setGoalPosition(DXL_ID, (180), UNIT_DEGREE);
      if (abs(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - (180)) < 3)
      {
        up = true;
      }

    }
  }
  else
  {
    if (up)
    {
      dxl.setGoalPosition(DXL_ID, minServoAngle, UNIT_DEGREE);
      if(abs(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - minServoAngle) < 3)
      {
        up = false;
      }
    }
    else
    {
      dxl.setGoalPosition(DXL_ID, 180, UNIT_DEGREE);
      if (abs(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - (180)) < 3)
      {
        up = true;
      }

    }
  }
}

// Based on set intervals, calls the function to set the angle, increase the angle, or record the angle
void ServoProject::gatherData(bool forward, int time, Dynamixel2Arduino dxl)
{
  if (start && (time % 1000) < 800 && !pass) // if within the 20 millisecond interval 
  {
    printTable(time, dxl);    
    pass = true;
  }
  else if(start && time % 2000 > 1800 && pass) // every 2 seconds
  {
    pass = false;
    angle = runThrough(forward, dxl, angle);
    dxl.setGoalPosition(DXL_ID, angle, UNIT_DEGREE);
  }
  else if(start && time % 1000 > 800 && pass) // ensures that a second is not counted twice
  {
    pass = false;
  }
}

void ServoProject::testEndurance(bool forwards, int time, Dynamixel2Arduino dxl)
{
  if (forwards)
  {
    if((time % 500) < 20 && !pass)
    {
      dxl.setGoalPosition(DXL_ID, rand() % 30 + 182, UNIT_DEGREE);
    }
    else if(time % 500 > 20 && pass)
    {
      pass = false;
    }
  }
  else
  {
    if((time % 500) < 20 && !pass)
    {
      dxl.setGoalPosition(DXL_ID, 182 - (rand() % 30), UNIT_DEGREE);
    }
    else if(time % 500 > 20 && pass)
    {
      pass = false;
    }
  }
}

unsigned long ServoProject::getStartTime ()
{
  return startTime;
}

int ServoProject::getJoystickMax()
{
  return joystickMax;
}

int ServoProject::getMinServoAngle()
{
  return minServoAngle;
}

int ServoProject::getMaxServoAngle()
{
  return maxServoAngle;
}

bool ServoProject::getOverride()
{
  return overRide;
}

void ServoProject::setJoystickMax(int max)
{
  joystickMax = max;
}

void ServoProject::setOverride(bool status)
{
  overRide = status;
}