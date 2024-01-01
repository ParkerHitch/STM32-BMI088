#ifndef __VECTORS
#define __VECTORS 

#include <math.h>

typedef struct Vector3 {
    double x;
    double y;
    double z;
} Vector3;

#define VECTOR_NULL {.x = NAN, .y = NAN, .z = NAN}

#define V_MUL(VEC, A) VEC.x*=A;VEC.y*=A;VEC.z*=A;

Vector3 vSub(Vector3 a, Vector3 b);

#endif