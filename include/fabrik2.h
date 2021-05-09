#ifndef FABRIKTEST_FABRIK2_H
#define FABRIKTEST_FABRIK2_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include <rbdl/rbdl.h>
#ifndef RBDL_BUILD_ADDON_LUAMODEL
#error "Error: RBDL addon LuaModel not enabled."
#endif
#include <rbdl/addons/luamodel/luamodel.h>

#include <limits> // for std::numeric_limits<unsigned int>::max() (DEBUGGING)
#include <cmath> // for sin() and cos()
#include "point.hpp"
#include "fabrik.hpp"
#include "axisLimit.hpp"

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



class fabrik2 {

protected:

    std::vector< Vector6d > ListOfPoints;

public:

    void Add( Vector6d& cpy);

    Vector3d ComputeNewPosition( Vector3d SecondToLast, Vector3d Last, Vector3d TargetPosition );

    void Forward();
    void Backwards();

    VectorXd GetAngles();

};


#endif //FABRIKTEST_FABRIK2_H
