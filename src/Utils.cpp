//
// Created by nash_mit on 23.04.2021.
//

#include "../include/IKProject/Utils.h"


Vector3d GetTraslationFromHomogeniousMatrix( Matrix4x4d homogeniousMatrix )
{
    Vector3d  Translation;
    Translation <<
        homogeniousMatrix(1 - 1, 4 - 1 ),
        homogeniousMatrix(2 - 1, 4 - 1 ),
        homogeniousMatrix(3 - 1, 4 - 1 );

    return Translation;
}

Vector3d GetEulerRotationsFromHomogeniousMatrix(Matrix4x4d homogeniousMatrix )
{
    Vector3d Rotations;

    double AroundX = atan2(homogeniousMatrix(3 - 1, 2 - 1 ), homogeniousMatrix(3 - 1,3 - 1 ));

    double AroundY = atan2(
            -homogeniousMatrix(3 - 1,1 - 1 ),
            homogeniousMatrix(3 - 1,2 - 1 ) * sin( AroundX ) + homogeniousMatrix(3 - 1,3 - 1 ) * cos( AroundX ) );

    double AroundZ = atan2( homogeniousMatrix( 2 - 1, 1 - 1 ), homogeniousMatrix(1 - 1 , 1 - 1 ) );

    Rotations << AroundX, AroundY, AroundZ;

    return Rotations;
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
