//
// Created by nash_mit on 09.05.2021.
//

#include "fabrik3.h"

void fabrik3::Add( Vector3d Point )
{
    Points.push_back( Point );

    if(Points.size() == 1)
        return;

    Distances.push_back( (Points[ Points.size() - 1 ] - Points[ Points.size() - 2 ]).norm() );
}

void fabrik3::SetTarget( Vector3d _Target )
{
    Target = _Target;
}

void fabrik3::Forward()
{

}



void fabrik3::Backwards()
{

    int Last = Points.size() - 1;
    int SecondToLast = Points.size() - 2;
    int PreSecondToLast = Points.size() - 3;

    // Project EndPoint ( Target ) on the "Last Allowed Motion Plane"
    Vector3d V1 = Points[ Last ]  - Points[ SecondToLast ];
    Vector3d V2 = Points[ SecondToLast ]  - Points[ PreSecondToLast ];

    Points[ Last ] = Projection( V1, V2, Target );


    // Reajust the distance
    Points[ SecondToLast ] =
            PreserveDistance(
                    Points[ Last ],
                    Points[ SecondToLast ],
                    Distances[ SecondToLast ] // the last one for Distances vector is 1 element less
                    );



    //Project SecondToLast point on the "Second to Last Allowed Motion Plane"
    V1 = Points[ SecondToLast ]  - Points[ PreSecondToLast ];
    V2 = Points[ SecondToLast - 1 ]  - Points[ PreSecondToLast - 1 ];

    Points[ SecondToLast ] = Projection( V1, V2, Points[ SecondToLast ] );


    // Project EndPoint ( Target ) on the "Last Allowed Motion Plane"
    V1 = Points[ Last ]  - Points[ SecondToLast ];
    V2 = Points[ SecondToLast ]  - Points[ PreSecondToLast ];

    Points[ Last ] = Projection(V1, V2, Target );


    //Reajust the distance
    Points[ SecondToLast ] =
            PreserveDistance(
                    Points[ Last ],
                    Points[ SecondToLast ],
                    Distances[ SecondToLast ] // the last one for Distances vector is 1 element less
            );

    //reajust distance for P1...




};

Vector3d fabrik3::Projection( Vector3d V1, Vector3d V2, Vector3d tobeProjected )
{
    Matrix3x2d A;
    A.block<1,3>(0,0) = V1;
    A.block<1,3>(0,1) = V2;

    Vector3d value = A * ( A.transpose() * A ).inverse() * A.transpose() * tobeProjected;

    return value;
}

Vector3d fabrik3::PreserveDistance( Vector3d LastPoint, Vector3d SecondToLast, double distance )
{
     Vector3d value = LastPoint + ( SecondToLast - LastPoint ).normalized() * distance;
     return value;
}

VectorXd fabrik3::GetQs()
{

}