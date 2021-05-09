#ifndef FABRIKTEST_FABRIK2_H
#define FABRIKTEST_FABRIK2_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include <Eigen/Geometry>


#include <rbdl/rbdl.h>
#ifndef RBDL_BUILD_ADDON_LUAMODEL
#error "Error: RBDL addon LuaModel not enabled."
#endif
#include <rbdl/addons/luamodel/luamodel.h>


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
typedef Matrix<double, 3, 3> Matrix3x3d;

Vector3d GetTraslationFromHomogeniousMatrix( Matrix4x4d homogeniousMatrix );
Matrix4x4d ExtractRotationMatrix( Matrix4x4d homogeniousMatrix );



class fabrik2 {

protected:

    std::vector< Matrix4x4d > Frames;

    Vector4d Target;

public:

    void Add( Vector4d Position, Matrix4x4d Rotation);
    void SetTarget( Vector4d _Target );

    Vector4d ComputeNewPosition( Vector4d SecondToLast, Vector4d Last, Vector4d TargetPosition );

    void Forward();
    void Backwards();

    VectorXd GetAngles();

    Vector4d FromLocalToGlobal(Vector4d Point, int FrameNumber );
    Vector4d FromGlobalToLocal(Vector4d Point, int FrameNumber );

    bool ApplyIK();

};


#endif //FABRIKTEST_FABRIK2_H
