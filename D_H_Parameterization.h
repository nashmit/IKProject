//
//

#ifndef KUKA_RBDL_D_H_PARAMETERIZATION_H
#define KUKA_RBDL_D_H_PARAMETERIZATION_H

#include "config.h"


class D_H_Parameterization
{

public:
    enum class Type { Prismatic, Revolute };

protected:

    //init robot pos
    double alpha;
    double d;
    double a;
    double theta;

    double DOF_Value;

    Type type;

public:

    Type GetType();

    D_H_Parameterization(
            Type _type,
            double _alpha,
            double _d,
            double _a,
            double _theta);

    Matrix4x4d GetAsHomogeniousMatrix();

    void SetDOF(double DeltaParameter);

    double GetDOF();

    Matrix4x4d GetDerivativeAtCurrentDHValueAsMatrix();
};


#endif //KUKA_RBDL_D_H_PARAMETERIZATION_H
