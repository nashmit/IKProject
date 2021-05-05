#ifndef KUKA_RBDL_UTILS_H
#define KUKA_RBDL_UTILS_H

#include "config.h"


Vector3d GetTraslationFromHomogeniousMatrix( Matrix4x4d homogeniousMatrix );

Vector3d GetRotationsFromHomogeniousMatrix( Matrix4x4d homogeniousMatrix );

Matrix4x4d ExtractRotationMatrix( Matrix4x4d homogeniousMatrix );


#endif //KUKA_RBDL_UTILS_H
