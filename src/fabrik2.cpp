#include "../include/fabrik2.h"


Vector3d GetTraslationFromHomogeniousMatrix( Matrix4x4d homogeniousMatrix )
{
    Vector3d  Translation;
    Translation <<
                homogeniousMatrix(1 - 1, 4 - 1 ),
            homogeniousMatrix(2 - 1, 4 - 1 ),
            homogeniousMatrix(3 - 1, 4 - 1 );

    return Translation;
}

Matrix4x4d ExtractRotationMatrix( Matrix4x4d homogeniousMatrix )
{
    Matrix4x4d RotationsMatrix;
    RotationsMatrix <<
                    0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0 ,0, 0,
            0, 0, 0, 1;

    RotationsMatrix.block<3,3>(0,0) = homogeniousMatrix.block<3,3>(0,0);

    return RotationsMatrix;
}


void fabrik2::Add( Vector4d Position, Matrix4x4d Rotation)
{
    Rotation.block<1,4>(0,3) = Position;

    Frames.push_back( Rotation );
}

void fabrik2::SetTarget( Vector4d _Target )
{
    Target = _Target;
}


//all the points are in the coord. system of the SecondToLast vector.
Vector4d fabrik2::ComputeNewPosition( Vector4d SecondToLast, Vector4d Last, Vector4d TargetPosition )
{
    Vector3d Distance1 = Last - SecondToLast;
    Vector3d Distance2 = TargetPosition - SecondToLast;

    Vector3d Axes = Distance1.cross( Distance2 );
    Axes.normalized();

    double angle = Distance1.normalized().dot ( Distance2.normalized() );

    Matrix4x4d Mat = AngleAxisd( angle, Axes).matrix();

    return Mat * TargetPosition;
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


Vector4d fabrik2::FromLocalToGlobal(Vector4d Point, int FrameNumber )
{
    Matrix4x4d Transform;
    Transform.setIdentity();

    for (int i = 0 ; i < FrameNumber ; i ++ )
        Transform *= Frames[i];

    return Transform * Point;
}


Vector4d fabrik2::FromGlobalToLocal(Vector4d Point, int FrameNumber )
{
    Matrix4x4d Transform;
    Transform.setIdentity();

    for (int i = 0 ; i < FrameNumber ; i ++ )
        Transform *= Frames[i];

    return Transform.inverse() * Point;

}

bool fabrik2::ApplyIK()
{
    assert(!"Not implemented!");
}
