#include "../include/IK.h"

int main()
{
    KuKa_KR5_R850_D_H KuKa;

    Vector6d Q_state;

    //KuKa.SetQforJoint(2, EIGEN_PI/3);
    //KuKa.SetQforJoint(4, -EIGEN_PI/7);
    //KuKa.SetQforJoint(6, EIGEN_PI/8);

    //Q_state << 0, EIGEN_PI/3, 0, -EIGEN_PI/7, 0, EIGEN_PI/8;
    Q_state << 0, EIGEN_PI/3, 0, EIGEN_PI/7, 0, EIGEN_PI/8;

    Vector6d Target;
    //Target << 0.845, 0, 0.425 , 2.81984, -1.5708 , 0.321751;
    //Target << 0.345, 0, 0 ,1, -1.5708 , -3;
    //Target << 0.545, 0.2, 0 , 2.81984, -1.5708 , 0.321751;
    //Target << 0.545, 0.2, 0.1 , 2, -1.5708 , 0.321751;
    //Target << 0.145, 0.5, 0.2 , 2, -1.0 , 0.321751;
    Target << 0.5, -0.145, 0.2 , 2, -1.0 , 0.321751;

    KuKa.ComputeIK( Q_state, Target );

    //Vector3d Target;
    //Target << 0.845, 0, 0.425;
    //KuKa.ComputeIK3D( Q_state, Target );

    return 0;
}
