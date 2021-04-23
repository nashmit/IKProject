//
// Created by nash_mit on 23.04.2021.
//

#ifndef KUKA_RBDL_CONFIG_H
#define KUKA_RBDL_CONFIG_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace Eigen;
using namespace std;


// Define a few types to make it easier
typedef Matrix<double, 6, 1>  Vector6d;
typedef Matrix<double, 7, 1>  Vector7d;
typedef Matrix<double, 8, 1>  Vector8d;

typedef Matrix<double, 4, 4> Matrix4x4d;

#endif //KUKA_RBDL_CONFIG_H
