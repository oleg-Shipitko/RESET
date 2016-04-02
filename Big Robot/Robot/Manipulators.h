#ifndef _MANIPULATORS_INCLUDED_
#define _MANIPULATORS_INCLUDED_


void soft_delay(unsigned long int);

///////////////////////////TOWER BUILDER///////////////////////
#define ID_FRONT 11      // Front manipulator
#define ID_BACK 11       // Back manipulator
#define CLOSED_ANG 0
#define OPEN_ANG 300

bool open_tower(int);
bool close_tower(int)  ;
///////////////////////////////////////////////////////////////

///////////////////////////CUBES CATCHER///////////////////////
#define ID_RIGHT 18  // If to look on robot from the front
#define ID_LEFT 1
#define OPEN_ANG_RIGHT 300
#define OPEN_ANG_LEFT 300
#define CLOSED_ANG_RIGHT 0
#define CLOSED_ANG_LEFT 300

bool open_cubes(int, int);
bool close_cubes(int, int);
///////////////////////////////////////////////////////////////

#endif
