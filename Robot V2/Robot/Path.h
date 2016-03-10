#ifndef _PATHSTRUCTURE_INCLUDED_
#define _PATHSTRUCTURE_INCLUDED_

#define PI             3.1415926535897932384626

typedef float pathPoint[2];
typedef float Center[3];
//{
//  float x;
//  float y;
//
//} Point;

typedef struct
{
    Center coordCenter;         //Координаты центра системы координат
    float alphZad;              //угол между вектором глобальный координат и траекторией
    float phiZad;             //угол поворота робота вокруг своей оси
    float phiDir;
    float lengthTrace;          //длина трассы
    float (*traceVel);      //V_уст, V_нач, V_кон, А_уск, А_торм
    float (*omegaVel);      //V_уст, V_нач, V_кон, А_уск, А_торм
    float Coord_local_track[3]; // локальные координаты в системе связанной с текущим участком траектории
    float Speed_local_track[3];  // локальные траекторные скорости
} Path;



void initPaths(void);

#endif

