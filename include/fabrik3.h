#ifndef FABRIKTEST_FABRIK3_H
#define FABRIKTEST_FABRIK3_H


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
typedef Matrix<double, 3, 2> Matrix3x2d;

Vector3d GetTraslationFromHomogeniousMatrix( Matrix4x4d homogeniousMatrix );
Matrix4x4d ExtractRotationMatrix( Matrix4x4d homogeniousMatrix );



class fabrik3 {

protected:
    std::vector< Vector3d > Points;
    std::vector< double > Distances;

    Vector3d Target;


public:
    void Add( Vector3d Point );

    void SetTarget( Vector3d _Target );

    void Forward();
    void Backwards();

    Vector3d Projection( Vector3d V1, Vector3d V2, Vector3d tobeProjected );

    Vector3d PreserveDistance( Vector3d LastPoint, Vector3d SecondToLast, double distance );

    VectorXd GetQs();
};


#endif //FABRIKTEST_FABRIK3_H
