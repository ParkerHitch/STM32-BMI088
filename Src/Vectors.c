#include "Vectors.h"

Vector3 vSub(Vector3 a, Vector3 b){
    Vector3 out;
    out.x = a.x - b.x;
    out.y = a.y - b.y;
    out.z = a.z - b.z;
    return out;
}