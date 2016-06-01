#ifndef _MANIPULATORS_INCLUDED_
#define _MANIPULATORS_INCLUDED_

#include <stdbool.h>
#include <stdint.h>
#include "Regulator.h"
#include "Pins.h"

extern PidStruct cubesCatcherPID;

void softDelay(unsigned long int);

///////////////////////////TOWER BUILDER///////////////////////
#define ID_FRONT 11                     // Front manipulator
#define ID_BACK 11                      // Back manipulator
#define CLOSED_ANG 0
#define OPEN_ANG 300

bool open_tower(int8_t);
bool close_tower(int8_t);
///////////////////////////////////////////////////////////////

///////////////////////////CUBES CATCHER///////////////////////
#define ID_RIGHT 18                     // If to look on robot from the front
#define ID_LEFT 1
#define OPEN_ANG_RIGHT 290//300
#define OPEN_ANG_LEFT 10 //0
#define OPEN_ANG_RIGHT_WIDELY 300
#define OPEN_ANG_LEFT_WIDELY 0 //0
#define CLOSED_ANG_RIGHT 150
#define CLOSED_ANG_LEFT 150
#define ONE_CUBE_CATCHED_ANGLE 30      // angle defining difference in manipulators angles in 1 cube is caught
#define TWO_CUBES_CATCHED_ANGLE 90     // angle defining difference in manipulators angles in 2 cubes are caught
#define CUBES_CATCHER_ADC 1
#define CUBES_CATCHER_MOTOR_CH 6

extern uint16_t adcData[10];
extern PidStruct cubesCatcherPID;
bool openCubesCatcher();
bool closeCubesCatcher(uint8_t*);
bool initCubeCatcherPID(void);
bool pidLowLevelManipulator(void);
bool openCubesCatcherWidely(void);
///////////////////////////////////////////////////////////////

///////////////////////////CUBES CATCHER 2///////////////////////
#define ID_RIGHT_2 5                     // If to look on robot from the front
#define ID_LEFT_2 1
#define OPEN_ANG_RIGHT_2 290//300
#define OPEN_ANG_LEFT_2 10 //0
#define CLOSED_ANG_RIGHT_2 150
#define CLOSED_ANG_LEFT_2 150

bool openCubesCatcher_2(void);
bool closeCubesCatcher_2(void);
///////////////////////////////////////////////////////////////

///////////////////////////PNEUMO//////////////////////////////

bool pneumoIn();
bool pneumoOut();
bool switchOnPneumo();
bool switchOffPneumo();
///////////////////////////////////////////////////////////////

///////////////////////////CUBES MOVERS////////////////////////
//#define RIGHT_CUBES_MOVER_CH 7
//#define LEFT_CUBES_MOVER_CH 8
//#define RIGHT_MOVER_IS_OPEN 0.08
//#define LEFT_MOVER_IS_OPEN 0.09
//#define RIGHT_MOVER_IS_CLOSED 0.125
//#define LEFT_MOVER_IS_CLOSED 0.04
//
//
//bool OpenCubesMovers();
//bool CloseCubesMovers();
///////////////////////////////////////////////////////////////

///////////////////////////VIBRATING TABLE/////////////////////
#define VIBRATING_MOTOR_PIN PIN1_12V
extern uint16_t vibratingTime;
extern uint16_t startingTime;
bool switchOnVibration(uint8_t);
bool switchOffVibration();
///////////////////////////////////////////////////////////////

//////////////////////////BELTS////////////////////////////////
//#define RIGHT_BELT_PIN PIN1_12V
//#define LEFT_BELT_PIN PIN2_12V
//
//bool switchOnBelts(void);
//bool switchOffBelts(void);
///////////////////////////////////////////////////////////////

//////////////////////////WALL////////////////////////////////
#define WALL_CH 7
#define WALL_IS_OPEN 0.05
#define WALL_IS_CLOSED 0.123

bool openWall(void);
bool closeWall(void);
///////////////////////////////////////////////////////////////

///////////////////////////CONE MOVER///////////////////////
#define CONE_CH 8
#define KICK_CONE 0.07
#define CONE_BACK 0.97

bool moveCone(void);
bool closeCone(void);
///////////////////////////////////////////////////////////////

#endif
