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
#include <ServoProject.h>

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

/*
* DO NOT CHANGE THESE
* They are based on the hardware used and not subject to change
*/
 // Joystick settings
#define JOYSTICK_PIN            A3
#define JOYSTICK_MIN            0.0
#define JOYSTICK_MAX            1020.0
#define CENTER_TAP              A0

// Servo settings
#define ANGLE_CHANGE            30
#define MIDDLE_ANGLE            180
#define SERVO_MIN_ANGLE         150  //149
#define SERVO_MAX_ANGLE         211  //225

// Testing settings
#define FORWARDS                true
#define BACKWARDS               false
#define TEST                    2 // 1: slow incremetation from 0 - 30; 2: fast incremetation to test endurance; 0: limit switch setup

/*
* These are subject to change
* User may change the number to fit their prefrences.
*/
#define JOYSTICK_DEADZONE       75.0  // 75 is the default. Range: 0-510
#define SERVO_MAX_VELOCITY      190    // 50 is the default. Range: 1-32767
#define LOOP_SLOW_VELOCITY      20    // 20 is the default. Range: 0-30

ServoProject servo(MIDDLE_ANGLE + ANGLE_CHANGE, MIDDLE_ANGLE - ANGLE_CHANGE, DXL_ID, MIDDLE_ANGLE, MIDDLE_ANGLE);
ServoProject joystick(JOYSTICK_MAX, JOYSTICK_DEADZONE, JOYSTICK_PIN, CENTER_TAP);

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

  dxl.setGoalPosition(DXL_ID, MIDDLE_ANGLE, UNIT_DEGREE); // Set home position to 180 degrees

  // turn on LED to confirm servo initialization then turn off after 2s
  Serial.println(dxl.writeControlTableItem(LED, DXL_ID, 1));
  delay(2000);
  dxl.writeControlTableItem(LED, DXL_ID, 0);

  // Limit the maximum velocity in Position Control Mode.
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, SERVO_MAX_VELOCITY);

  // sets the pin for the limit switch
  pinMode(0, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

}

bool up = true;
void loop() {
  int joystickValue = analogRead(JOYSTICK_PIN);

 
  //joystick.calibrate();
  //Serial.println(joystickMax);
  
  // Adjust the values to account for the deadzone
  joystickValue = joystick.setDeadzone(joystickValue);
  
  //servo.readCyclePosition();
  //servo.cycleMotion();

// Map joystick value to servo angle
  float servoAngle = map(joystickValue, JOYSTICK_MIN, joystick.getJoystickMax(), servo.getMinServoAngle(), servo.getMaxServoAngle());

  // the starting for the testing functions
  servo.init(dxl);
  int currentTime = millis() - servo.getStartTime();
  if (abs(servoAngle - MIDDLE_ANGLE) < 2 && !servo.getOverride())
  {
    if(TEST == 1)
    {
      servo.gatherData(FORWARDS, currentTime, dxl);
    }
    else if(TEST == 2)
    {
      //servo.testEndurance(FORWARDS, currentTime, dxl);
      servo.autoLoop(false, dxl);
    }
    else
    {
      servo.readCyclePosition();
      servo.cycleMotion(dxl, LOOP_SLOW_VELOCITY);
    }
    //Serial.print("test");
        
  }
  else
  {
  // Set servo goal position
  dxl.setGoalPosition(DXL_ID, servoAngle, UNIT_DEGREE);
  servo.setOverride(true);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, SERVO_MAX_VELOCITY);
    if(digitalRead(5))
    {
      servo.setOverride(false);
    }
  float presentPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
  //Serial.print("Present Position (degree): ");
  //Serial.println(presentPosition);

  }
  //Serial.println(dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 30));

   // Print the raw joystick value
  //Serial.print("Joystick Value: ");
  //Serial.println(joystickValue);
/*
  Serial.print("switch 1: ");
  Serial.println(digitalRead(0));
  Serial.print("switch 2: ");
  Serial.println(digitalRead(2));
  Serial.print("switch 3: ");
  Serial.println(digitalRead(3));
  Serial.print("switch 4: ");
  Serial.println(digitalRead(4));
  */
  // Read and print the current servo position
  
  
  
  delay(10);
}
