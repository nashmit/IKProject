#include "../include/IK.h"

int main()
{
    KuKa_KR5_R850_D_H KuKa;

    Vector6d Q_state;
    Q_state << 0, 0, 0, 0, 0, 0;

    //Vector6d Target;
    //Target << 0.845, 0, 0.425 , 2.81984, -1.5708 , 0.321751;
    //Target << 0.345, 0, 0 ,1, -1.5708 , -3;
    //KuKa.ComputeIK( Q_state, Target );

    Vector3d Target;
    Target << 0.845, 0, 0.425;
    KuKa.ComputeIK3D( Q_state, Target );

    return 0;
}
