//
// Created by nash_mit on 23.04.2021.
//

#include "../include/IKProject/D_H_Parameterization.h"

D_H_Parameterization::Type D_H_Parameterization::GetType()
{
    return type;
}

D_H_Parameterization::D_H_Parameterization( Type _type, double _alpha, double _d, double _a, double _theta)
{
    type = _type;

    alpha = _alpha;
    d = _d;
    a = _a;
    theta = _theta;

    DOF_Value = 0;
}

Matrix4x4d D_H_Parameterization::GetAsHomogeniousMatrix()
{
    //delta 'Theta'
    double DTheta = 0;
    //delta 'd'
    double Dd = 0;

    if (type==Type::Revolute)
    {
        DTheta = DOF_Value;
    }

    if (type==Type::Prismatic)
    {
        Dd = DOF_Value;
    }

    Matrix4x4d solution;
    solution <<
        cos( theta + DTheta ),    -cos( alpha ) * sin( theta + DTheta ),     sin( alpha ) * sin( theta + DTheta ),    a * cos(theta + DTheta ),
        sin( theta + DTheta ),     cos( alpha ) * cos( theta + DTheta ),    -sin( alpha ) * cos( theta + DTheta ),    a * sin(theta + DTheta ),
                0,                          sin( alpha ),                               cos( alpha ),                             d + Dd,
                0,                                  0,                                          0,                                  1;

    return solution;
}

void D_H_Parameterization::SetDOF(double DeltaParameter)
{
    DOF_Value = DeltaParameter;
}

double D_H_Parameterization::GetDOF()
{
    return DOF_Value;
}

Matrix4x4d D_H_Parameterization::GetDerivativeAtCurrentDHValueAsMatrix()
{
    Matrix4x4d result;

    if (type == Type::Prismatic)
        result <<
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 1,
        0, 0, 0, 0;

    if (type == Type::Revolute)
        result <<
            -sin(theta + DOF_Value ),     -cos( alpha ) * cos( theta + DOF_Value ),   sin( alpha ) * cos( theta + DOF_Value ),    -a * sin( theta + DOF_Value ),
             cos(theta + DOF_Value ),     -cos( alpha ) * sin( theta + DOF_Value ),   sin( alpha ) * sin( theta + DOF_Value ),     a * cos( theta + DOF_Value ),
                        0,                                      0,                                          0,                                          0,
                        0,                                      0,                                          0,                                          0;

    return result;
}

