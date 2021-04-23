//
// Created by nash_mit on 23.04.2021.
//

#ifndef KUKA_RBDL_UTILS_H
#define KUKA_RBDL_UTILS_H

#include "config.h"


Vector3d GetTraslationFromHomogeniousMatrix( Matrix4x4d homogeniousMatrix );

Vector3d GetRotationsFromHomogeniousMatrix( Matrix4x4d homogeniousMatrix );


#endif //KUKA_RBDL_UTILS_H
