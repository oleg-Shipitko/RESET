#ifndef __MATR
#define __MATR

void matrixCopy(float *m, char rows, char columns, float *new_m);
float matrixGetCell(float *m, char rows, char columns, char row, char column);
void matrixMultiplyM2M(float *m1, char rows1, char columns1, float *m2, char rows2, char columns2, float *new_m);
void matrixMultiplyS2M(float *m, char rows, char columns, float s, float *new_m);
void matrixPlusMinus(float *m1, float *m2, char rows, char columns, signed char sign,float *new_m);
void matrixTranspose(float *m, char rows, char columns, float *new_m);
void matrixDet_LU_Transform(float *A, char n,float *out);
void matrixCofactor(float *m, char size, float *new_m);
void matrixInverse(float *m, char size, float *new_m);
void matrixFill(float *m, char rows, char columns, float val);

#endif 
