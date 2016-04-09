#ifndef _MANIPULATORS_INCLUDED_
#define _MANIPULATORS_INCLUDED_

#include <stdbool.h>

void soft_delay(unsigned long int);

///////////////////////////TOWER BUILDER///////////////////////
#define ID_FRONT 11      // Front manipulator
#define ID_BACK 11       // Back manipulator
#define CLOSED_ANG 0
#define OPEN_ANG 300

bool open_tower(int8_t);
bool close_tower(int8_t);
///////////////////////////////////////////////////////////////

///////////////////////////CUBES CATCHER///////////////////////
#define ID_RIGHT 18             // If to look on robot from the front
#define ID_LEFT 1
#define OPEN_ANG_RIGHT 300
#define OPEN_ANG_LEFT 300
#define CLOSED_ANG_RIGHT 0
#define CLOSED_ANG_LEFT 300
#define ONE_CUBE_CATCHED_ANGLE 30      // angle defining difference in manipulators angles in 1 cube is caught
#define TWO_CUBES_CATCHED_ANGLE 90     // angle defining difference in manipulators angles in 2 cubes are caught

bool open_cubes();
bool close_cubes();
///////////////////////////////////////////////////////////////

///////////////////////////PNEUMO//////////////////////////////

bool pneumoIn();
bool pneumoOut();
bool switchOnPneumo();
bool switchOffPneumo();
///////////////////////////////////////////////////////////////


#endif
