//////////////////////////////////////////////////////
// Original Author: Jose Julio
// Modified by Alex Chen
//////////////////////////////////////////////////////

////////////////////////////
// debug
////////////////////////////
// #define SHOW_LOG            1 // comment it out if you don't want to show debug info
////////////////////////////
// Pin definition
////////////////////////////
#define BAUD_RATE           115200

//========================================================================================================================================
////////////////////////////
// Physical dimenssion:
////////////////////////////

// This is the center of the table. All units in milimeters
// This are custum dimensions
#define TABLE_LENGTH    1003                      // 39.5''
#define TABLE_WIDTH     597                       // 23.5''
#define ROBOT_CENTER_X  TABLE_WIDTH /2             // Center of robot = 298.5
#define ROBOT_CENTER_Y  TABLE_LENGTH /2

// Absolute Min and Max robot positions in mm (measured from center of robot pusher)
#define ROBOT_MIN_X     50
#define ROBOT_MIN_Y     80
#define ROBOT_MAX_X     TABLE_WIDTH - ROBOT_MIN_X
#define ROBOT_MAX_Y     ROBOT_CENTER_Y - 120

// Initial robot position in mm
// The robot must be at this position at start time
// Default: Centered in X and minimun position in Y
#define ROBOT_INITIAL_POSITION_X              TABLE_WIDTH /2
#define ROBOT_INITIAL_POSITION_Y              80            // Measured from center of the robot pusher to the table border

// Robot defense and attack lines
#define ROBOT_DEFENSE_POSITION_DEFAULT        120
#define ROBOT_DEFENSE_ATTACK_POSITION_DEFAULT 280
#define ROBOT_DEFENSE_POSITION_MIN            60
#define ROBOT_DEFENSE_POSITION_MAX            100
#define ROBOT_DEFENSE_ATTACK_POSITION_MIN     140
#define ROBOT_DEFENSE_ATTACK_POSITION_MAX     360

#define PUCK_SIZE       22                        // PuckSize (puck radius in mm)
//========================================================================================================================================
//////////////////////////////////
// Max Robot Speed & Acceleration
//////////////////////////////////

// THIS VALUES DEPENDS ON YOUR ROBOT CONSTRUCTION (MOTORS, MECHANICS...)
// RECOMMENDED VALUES FOR 12V POWER SUPPLY
#define MAX_ABS_ACCEL         120//275                        // Maximun motor acceleration in (steps/seg2)/1000. Max recommended value:280
#define MAX_ABS_SPEED         22000//32000                    // Maximun speed in steps/seg. Max absolute value: 32767!!

#define MIN_ACCEL         70//100						// make sure this is not 0
#define MIN_SPEED         5000//5000

#define SCURVE_LOW_SPEED  2500//2500

#define ZERO_SPEED        65535

#define MIN_PUCK_Y_SPEED1        -280                    // used in Robot::newDataStrategy()
#define MIN_PUCK_Y_SPEED2        -160                    // used in Robot::newDataStrategy()
#define BOT_MOVE_TIME_THRESHOLD   600
#define ATTACK_TIME_THRESHOLD     500
#define IMPACT_TIME_THRESHOLD     200
#define MIN_ABS_Y_SPEED           100
#define MIN_ABS_X_SPEED           140
#define MIN_PREDICT_TIME          150                   // used in Robot::robotStrategy()
#define STOP_COEF         1800//1800
//========================================================================================================================================
//////////////////////////////////
// Geometric calibration
//////////////////////////////////

// This depends on the pulley teeth. DEFAULT: 200(steps/rev)*8(microstepping) = 1600 steps/rev. 1600/32teeth*2mm(GT2) = 25 steps/mm
// Alex note: I got 20 teeth pully. Due to the dimension of my table, the move can be wobbling, and so this factor doesn't need to be super accurate, it will be off slightly (~1-2cm) anyway.
#define X_AXIS_STEPS_PER_UNIT 41//36//41
#define Y_AXIS_STEPS_PER_UNIT 41//36//41

// CORRECTION FOR VISION SYSTEM LAG
#define VISION_SYSTEM_LAG 60   // in miliseconds

// CORRECTION OF MISSING STEPS ON MOTORS
// Coment this lines if you don´t want to make the corrections
#define CORRECT_MISSING_STEPS 1
#define MISSING_STEPS_MAX_ERROR_X 5
#define MISSING_STEPS_MAX_ERROR_Y 5
#define ROBOT_POSITION_CAMERA_CORRECTION_Y 0 // Correction of the position of the camera because the camera point of view and mark position

//========================================================================================================================================
//////////////////////////////////
// Utils (don´t modify)
//////////////////////////////////

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
