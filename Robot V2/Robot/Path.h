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
    Center coordCenter;         //���������� ������ ������� ���������
    float alphZad;              //���� ����� �������� ���������� ��������� � �����������
    float phiZad;             //���� �������� ������ ������ ����� ���
    float phiDir;
    float lengthTrace;          //����� ������
    float (*traceVel);      //V_���, V_���, V_���, �_���, �_����
    float (*omegaVel);      //V_���, V_���, V_���, �_���, �_����
    float Coord_local_track[3]; // ��������� ���������� � ������� ��������� � ������� �������� ����������
    float Speed_local_track[3];  // ��������� ����������� ��������
} Path;



void initPaths(void);

#endif

