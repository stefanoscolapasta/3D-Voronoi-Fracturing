#pragma once
#include <bullet/LinearMath/btVector3.h>
#ifndef UTILS_H
#define UTILS_H
#define N 4

//RETURN VALUE:
//>0 -> p is above the plane defined by a, b,c
//<0 -> p is under the plane defined by a, b,c
// = 0-> p is on the plane defined by a, b,c
int Orient(btVector3 a, btVector3 b, btVector3 c, btVector3 p);
float determinantOfMatrix(float matrix[N][N], int n);
void subMatrix(float mat[N][N], float temp[N][N], int p, int q, int n);
#endif
