#ifndef FABRIKTEST_CCD_H
#define FABRIKTEST_CCD_H


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

class Frame
{
    public:
    Vector3d Position;
    Matrix3x3d frame;

    Frame()
    {
        Position.setZero();
        frame.setIdentity();
    }

};

class ccdTransform
{
public:
    Quaterniond Rotation;
    Vector3d Translation;

    ccdTransform()
    {
        Rotation.setIdentity();
        Translation.setZero();
    }

    ccdTransform(Vector3d _Translation, Quaterniond _Rotation)
    {
        Translation = _Translation;
        Rotation = _Rotation;
    }

    Frame ApplyTransform( Frame TobeTransformed )
    {
        Frame newFrame;

        newFrame.frame = Rotation * TobeTransformed.frame;
        newFrame.Position = TobeTransformed.Position + Translation;

        return  newFrame;
    }
};

class Node
{
public:
    ccdTransform transform;
    Vector3d Axes;

    Node()
    {
        transform.Translation = Vector3d().setZero();
        transform.Rotation = Quaterniond().setIdentity();
    }

    Node( ccdTransform _transform, Vector3d _Axes )
    {
        transform = _transform;
        Axes = _Axes;
    }
};


class ccd {

protected:
    vector<Node> Nodes;
    Node Target;

public:
    void Add(Node node);
    void SetTarget(Node _Target);

    Quaterniond RotateNodeAroundNodeTo(Node pivot, Node NoteToRotate, Node Direction);
    Quaterniond EnforcingHinge(Node Parent, Node pivot);

    bool Iterate( double Epsilon = 0.001, int iterationNumber = 100 );

    Frame UpdateGlobalCoordinates( int nodeIndex );

    VectorXd GetQs();

};


#endif //FABRIKTEST_CCD_H
