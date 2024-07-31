/*
* ServoProject.h - Library for controlling the Dynamixel Servo on an OpenRB-150 Arduino using an analog joystick
* Created by Paul Galiasso, June 6, 2024
* For use of Hydra Service and subsequent products
*/

#ifndef ServoProject_h
#define ServoProject_h

#include "Arduino.h"

class ServoProject
{
  private:

    /*
    * velocity 1 for the auto looping
    */
    double vel1 = 30;
   
    /*
    * velocity 2 for the auto looping
    */
    double vel2;

    /**
    * For the auto looping
    * True if the program is moving right
    */
    bool switchPress_1 = true; // first limit switch

    /*
    * For the auto looping
    * true if the program is on the left end
    */
    bool switchPress_2; // second limit switch

    /*
    * For the auot looping
    * true if the program is on the right end
    */
    bool switchPress_3; // third limit switch

    /*
    * For the auto looping
    * true if the program is moving left
    */
    bool switchPress_4; // fourth limit switch

    /*
    * For the testing functions
    * true if the testing functions have started
    */
    bool start = true; // if the testing functions have started 

    /*
    * For testing
    * true if moving away from angle 0
    */
    bool up = true; // If moving away from angle 0

    /*
    * For testing
    * true if the intended action for the second has been completed
    */
    bool pass = false; // if the second has already passed

    /*
    * For testing
    * Saftey mechanism that returns control of the servo to the joystick if true
    */
    bool overRide = true; // gives back control to joystick

    /*
    * velocity?
    */
    int vel; // velocity

    /*
    * For calibration
    * records the number of times the backup max value is used
    */
    int counter = 0; // how many times the backups is used

    /*
    * For testing
    * Records the starting time in milliseconds
    */
    unsigned long startTime; //The starting time

    //varables for objects

    /*
    * The maximum intended angle for the servo
    */
    int maxServoAngle;

    /*
    * the minimum intended angle for the servo
    */
    int minServoAngle;

    /*
    * the desired angle of the servo
    */
    int angle; // The desired angle of the servo

    /*
    * the center angle of the servo
    */
    int centerAngle;

    /*
    * the ID necessary for the Dynamixel Servo
    */
    uint8_t DXL_ID;

    /*
    * the maximum possible value of the joystick
    * Expected to be near 1023
    */
    int joystickMax;// = JOYSTICK_MAX; // Maximum angle of the joystick

    /*
    * the deadzone of the joystick
    * the value is subracted from both sides of the center of the joystick
    * Caution: the deadzone may be double what you expect it to be
    * Example: joystick value from 0 - 1000. center is 500
    *          deadzone value of 150: range of 350 - 650 is the deadzone of the joystick
    */
    int joystickDeadzone;

    /*
    * The velocity for the autoLooping
    */
    int loopVelocity;

    /*
    * The analog pin connected to the signal of the joystick
    * Values: A0, A1, A2, A3, A4, A5, A6
    */
    pin_size_t signalPin;

    /*
    * The analog pin connected to the center-tap of the joystick
    * Values: A0, A1, A2, A3, A4, A5, A6
    */
    pin_size_t centerPin;

  public:

    /*
    * Empty constructor
    */
    ServoProject();

    /*
    * @Brief Constructor for the use of the servo
    * @Param _maxServoAngle: The maximum intended angle for the servo
    * @Param _minServoAngle:The minimum intended angle for the servo
    * @Param ID: The ID for the Dynamixel Servo
    * @Param _angle: The desired angle of the servo
    */
    ServoProject(int _maxServoAngle, int _minServoAngle, uint8_t _ID, int _angle, int _centerAngle);

    /*
    * @Brief Constructor for the use of the analog joystick
    * @Param _joystickMax: The maximum value of the joystick.
    * @Param _deadzone: The size of the deadzone of the joystick 
    * @Param _signalPin: The analog pin used for the signal of the joystick
    * @Param _centerPin: The analog pin used for the center-tap of the joystick
    */
    ServoProject(int _joystickMax, int _deadzone, pin_size_t _signalPin, pin_size_t _centerPin);

    /*
    * @Breif Initializes the start of the testing functions, especially to keep track of the starting time
    * @Param dxl: The Dynamixel Servo object
    * @Returns: Nothing
    */
    void init(Dynamixel2Arduino dxl);

    /*
    * @Brief Cycles the motion of the auto looping
    * @Param dxl: The dynamixel Servo Object
    * @Returns: nothing
    */
    void cycleMotion(Dynamixel2Arduino dxl, int);

    /*
    * @Brief Reads the current postision of the auto looping and records it
    * @Param none
    * @Returns: nothing
    */
    void readCyclePosition();

    /* 
    * @Brief Calibrates the joystick using the center-tap
    * @Paremeters: none
    * @Returns: nothing
    */
    void calibrate ();

    /*
    * @Brief Sets the deadzone of the joystick, and 
    * @Param joyval: The raw value of the joystick
    * @Returns: The value of the joystick with the deadzone applied
    */
    int setDeadzone(int joyVal);

    /*
    * @Brief Prints the time and angle of the servo onto the serial mointor as a CSV
    * @Param time: the current time recorded in milliseconds by the arduino
    * @Param dxl: the Dynamixel Servo object
    * @Returns: nothing
    */
    void printTable(int time, Dynamixel2Arduino dxl);

    /*
    * @Brief changes the desired angle of the servo for the testing
    * @Param Forwards: if the servo is set to move forwards or backwards
    * @Paramdxl: the Dynamixel Servo object
    * @Returns: nothing
    */
    int runThrough(bool forwards, Dynamixel2Arduino dxl, int);

    /*
    * @Brief Testing fucntion that gathers the angle and time of the servo
    * @Param forwards: if the servo is set to move forwards or backwards
    * @Param time: the current time recorded in milliseconds by the arduino
    * @Param dxl: the Dynamixel Servo object
    * @Returns: nothing
    */
    void gatherData(bool forward, int time, Dynamixel2Arduino dxl);

    /*
    * @Brief Tests the endurance of the servo, changes to a random angle every half second
    * @Param forwards: if the servo is set to move forwards or backwards
    * @Param time: the current time recorded in milliseconds by the arduino
    * @Param dxl: the Dynamixel Servo object
    * @Returns: nothing 
    */
    void testEndurance(bool forwards, int time, Dynamixel2Arduino dxl);

    /*
    * @Brief gets the starting time of the testing functions
    * @Param none
    * @Returns: the starting time in milliseconds
    */
    unsigned long getStartTime();

    /*
    * @Returns the maximum value of the joystick
    */
    int getJoystickMax();

    /*
    * @Returns the minimum angle of the servo
    */
    int getMinServoAngle();

    /*
    * @Returns the maximum angle of the servo
    */
    int getMaxServoAngle();

    /*
    * @Brief Sets the maximum angle of the joystick
    * @Param The maximum angle of the joystick
    * @Returns: nothing
    */
    void setJoystickMax(int);

    /*
    * @return whether override control is given to joystick
    */
    bool getOverride();

    /*
    * @brief Sets the status of override control
    * @param the status of the override control
    * @return nothing
    */
    void setOverride(bool);


    /*
    * @brief the timing for the acceleration in the cycle motion function
    * @param Whether forwasrs or backwards
    * @param the Dynamixel Servo object
    * @return nothing
    */
    void autoLoop(bool, Dynamixel2Arduino);

};

#endif