

#ifndef ServoProject_h
#define ServoProject_h

#include "Arduino.h"

class ServoProject
{
  private:
    bool forwards; // Test for forwards
    bool switchPress_1; // first limit switch
    bool switchPress_2; // second limit switch
    bool switchPress_3; // third limit switch
    bool switchPress_4; // fourth limit switch
    bool start = true; // if the testing functions have started 
    bool up = true; // If moving away from angle 0
    bool pass = false; // if the second has already passed
    bool overRide = false; // gives back control to joystick
    //int joyVal; // value of the joystick
    //int time; // current time
    int vel; // velocity
    int counter = 0; // how many times the backups is used
    unsigned long startTime; //The starting time

    //varables for objects
    int maxServoAngle;
    int minServoAngle;
    int angle = 182; // The desired angle of the servo
    uint8_t DXL_ID;

    int joystickMax;// = JOYSTICK_MAX; // Maximum angle of the joystick
    int joystickDeadzone;
    pin_size_t signalPin;
    pin_size_t centerPin;

  public:
    ServoProject();
    ServoProject(int _maxServoAngle, int _minServoAngle, uint8_t _ID, int _angle);
    ServoProject(int _joystickMax, int _deadzone, pin_size_t _signalPin, pin_size_t _centerPin);
    void init(Dynamixel2Arduino dxl);
    void cycleMotion(Dynamixel2Arduino dxl);
    void readCyclePosition();
    void calibrate ();
    int setDeadzone(int joyVal);
    void printTable(int time, Dynamixel2Arduino dxl);
    void runThrough(bool forwards, Dynamixel2Arduino dxl);
    void gatherData(bool forward, int time, Dynamixel2Arduino dxl);
    void testEndurance(bool forwards, int time, Dynamixel2Arduino dxl);

    unsigned long getStartTime();
    int getJoystickMax();
    int getMinServoAngle();
    int getMaxServoAngle();

    void setJoystickMax(int);

   /* int getMaxServoAngle();
    int getMinServoAngle();
    int getID();

    int getJoystickMax();
    int getJoystickMin();
    int getDeadzone();
    int getCenterTap();
    int getJoystickPin();
*/


};

#endif