/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

 // Joystick settings
#define JOYSTICK_PIN            A3
#define JOYSTICK_MIN            0.0
#define JOYSTICK_MAX            1020.0
#define JOYSTICK_DEADZONE       75.0
#define CENTER_TAP              A0

// Servo settings
#define SERVO_MIN_ANGLE         150  //149
#define SERVO_MAX_ANGLE         211  //225
#define SERVO_MAX_VELOCITY      50

// Testing settings
#define FORWARDS                true
#define BACKWARDS               false

void setup() {
  // Initialize serial communication for debugging
  DEBUG_SERIAL.begin(115200);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);

  dxl.setGoalPosition(DXL_ID, 182.0, UNIT_DEGREE); // Set home position to 180 degrees

  // turn on LED to confirm servo initialization then turn off after 2s
  Serial.println(dxl.writeControlTableItem(LED, DXL_ID, 1));
  delay(2000);
  dxl.writeControlTableItem(LED, DXL_ID, 0);

  // Limit the maximum velocity in Position Control Mode.
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, SERVO_MAX_VELOCITY);

  // sets the pin for the limit switch
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

}

// loop global variables
bool switchPress_1 = false; // left most switch
bool switchPress_2 = false; // second switch to the left
bool switchPress_3 = false; // second switch to the right
bool switchPress_4 = true; // right most switch
int vel = 0; // velocity?

// global variables for auto-calibration
int joystickMax = JOYSTICK_MAX; // Maximum angle of the joystick
int counter = 0; // how many times the backups is used

// testing global Variables
bool start = true; // if the testing functions have started
unsigned long startTime; //The starting time 
int angle = 182; // The desired angle of the servo
bool up = true; // If moving away from angle 0
bool pass = false; // if the second has already passed
bool overRide = false; // gives back control to joystick

// cycles the motion of the pump using the limit switches
void cycleMotion()
{
  if(switchPress_4 && !switchPress_2)
  {
    dxl.setGoalPosition(DXL_ID, SERVO_MAX_ANGLE, UNIT_DEGREE);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 80);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 20);
  }
  else if (switchPress_4 && switchPress_2)
  {
    dxl.setGoalPosition(DXL_ID, SERVO_MAX_ANGLE, UNIT_DEGREE);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 40);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, -50);
  }
  else if (switchPress_1 && switchPress_2)
  {
    dxl.setGoalPosition(DXL_ID, SERVO_MIN_ANGLE, UNIT_DEGREE);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 40);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 50);
  }
  else if (switchPress_1 && !switchPress_3)
  {
    dxl.setGoalPosition(DXL_ID, SERVO_MIN_ANGLE, UNIT_DEGREE);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 80);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 20);
  }
  else if (switchPress_1 && switchPress_3)
  {
    dxl.setGoalPosition(DXL_ID, SERVO_MIN_ANGLE, UNIT_DEGREE);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 40);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, -50);
  }
  else if (switchPress_4 && switchPress_3)
  {
    dxl.setGoalPosition(DXL_ID, SERVO_MAX_ANGLE, UNIT_DEGREE);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 40);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 50);
  }
}

// reads the current postion in the cycle based on the limit switches
void readCyclePosition()
{

  // Reads the limit switches to find the current position. 
  if(switchPress_4 && !switchPress_3 && !switchPress_2 && !switchPress_1)
  {
    switchPress_2 = digitalRead(2);
  }
  else if (switchPress_4 && !switchPress_3 && switchPress_2 && !switchPress_1)
  {
    switchPress_1 = digitalRead(1);
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
int calibrate (int backup)
{
  int middleVal = analogRead(CENTER_TAP);
  int maxVal = middleVal * 2;

  // back up in case that something with the center tap pin goes wrong
  if (abs(maxVal - backup) >= 100 || counter > 500)
  {
    counter = 0;
    return maxVal;
  }
  else
  {
    counter++; 
    return backup;
  }
}


// Adjust the values to account for the deadzone
int deadzone(int joyVal)
{
  // Find the values that the deadzone cuts off
  float high_deadzone_val = (joystickMax / 2) + JOYSTICK_DEADZONE;
  float low_deadzone_val = (joystickMax / 2) - JOYSTICK_DEADZONE;

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

// Prints the CVS of the time and the servo angle
void printTable(int time)
{
    Serial.print(time / 1000);
    Serial.print(",");
    Serial.print(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - 182);
    Serial.println();
}


// increments the desired angle and decreses back to 0 when having reached 30 degrees
void runThrough(bool forwards)
{
  if (abs(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) - 182) >= 29)
  {
    up = false;
  }

  if (forwards)
  {
    up ? angle++ : angle--;
  }
  else
  {
    up ? angle-- : angle++;
  }
}

// Based on set intervals, calls the function to set the angle, increase the angle, or record the angle
void gatherData(bool forward, int time)
{
  if (start && (time % 1000) < 800 && !pass) // if within the 20 millisecond interval 
  {
    printTable(time);    
    pass = true;
  }
  else if(start && time % 2000 > 1800 && pass) // every 2 seconds
  {
    pass = false;
    runThrough(forward);
    dxl.setGoalPosition(DXL_ID, angle, UNIT_DEGREE);
  }
  else if(start && time % 1000 > 800 && pass) // ensures that a second is not counted twice
  {
    pass = false;
  }
}

void testEndurance(bool forwards, int time)
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


void loop() {
  int joystickValue = analogRead(JOYSTICK_PIN);

 
  //joystickMax = calibrate(joystickMax);
  //Serial.println(joystickMax);

  // Apply deadzone to joystick value
  if (abs(joystickValue - (joystickMax / 2)) < JOYSTICK_DEADZONE) {
    joystickValue = joystickMax / 2;
  }
  
  // Adjust the values to account for the deadzone
  joystickValue = deadzone(joystickValue);
  
  //readCyclePosition();
  //cycleMotion();

// Map joystick value to servo angle
  float servoAngle = map(joystickValue, JOYSTICK_MIN, joystickMax, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  // the starting for the testing functions
  
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
  int currentTime = millis() - startTime;
  //if (abs(servoAngle - 180) < 2 && !overRide)
  //{
    //gatherData(FORWARDS, currentTime);
    //testEndurance(FORWARDS, currentTime);
 // }
  //else
  //{
// Set servo goal position
 // dxl.setGoalPosition(DXL_ID, servoAngle, UNIT_DEGREE);
 // overRide = true;
 // }

   // Print the raw joystick value
  Serial.print("Joystick Value: ");
  Serial.println(joystickValue);

  // Read and print the current servo position
  float presentPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
  Serial.print("Present Position (degree): ");
  Serial.println(presentPosition-180);
  
  delay(10);
}
