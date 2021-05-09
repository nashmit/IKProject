#include "../include/fabrik2.h"


void fabrik2::Add( Vector6d& cpy)
{
    ListOfPoints.push_back(cpy);
}

Vector3d fabrik2::ComputeNewPosition( Vector3d SecondToLast, Vector3d Last, Vector3d TargetPosition )
{
    Vector3d Distance1 = Last - SecondToLast;
    Vector3d Distance2 = TargetPosition - SecondToLast;

    Vector3d Axes = Distance1.cross( Distance2 );
    Axes.normalized();

    double angle = Distance1.normalized().dot ( Distance2.normalized() );



}

void fabrik2::Forward()
{

}

void fabrik2::Backwards()
{

}

VectorXd fabrik2::GetAngles()
{

}