#ifndef KUKA_RBDL_UTILS_H
#define KUKA_RBDL_UTILS_H

#include "config.h"


Vector3d GetTraslationFromHomogeniousMatrix( Matrix4x4d homogeniousMatrix );

//Rotation around X,Y,Z axes
Vector3d GetEulerRotationsFromHomogeniousMatrix(Matrix4x4d homogeniousMatrix );

Matrix4x4d ExtractRotationMatrix( Matrix4x4d homogeniousMatrix );


#endif //KUKA_RBDL_UTILS_H
