#ifndef _MANIPULATORS_INCLUDED_
#define _MANIPULATORS_INCLUDED_

#include <stdbool.h>
#include <stdint.h>

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
#define OPEN_ANG_RIGHT 300
#define OPEN_ANG_LEFT 0
#define CLOSED_ANG_RIGHT 150
#define CLOSED_ANG_LEFT 150
#define ONE_CUBE_CATCHED_ANGLE 30      // angle defining difference in manipulators angles in 1 cube is caught
#define TWO_CUBES_CATCHED_ANGLE 90     // angle defining difference in manipulators angles in 2 cubes are caught
#define CEBES_CATCHER_ADC 1
#define CUBES_CATCHER_MOTOR_CH 5

extern uint16_t adcData[10];

bool open_cubes();
bool close_cubes(uint8_t*);
void initCubeCatcherPID(void);
void GetDataForManipulator(void);
void pidLowLevelManipulator(float, float);
///////////////////////////////////////////////////////////////

///////////////////////////PNEUMO//////////////////////////////

bool pneumoIn();
bool pneumoOut();
bool switchOnPneumo();
bool switchOffPneumo();
///////////////////////////////////////////////////////////////


#endif
