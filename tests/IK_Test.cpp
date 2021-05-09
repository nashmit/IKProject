#include "../include/IK.h"

int main()
{
    KuKa_KR5_R850_D_H KuKa;

    Vector6d Q_state;
    Vector6d Q_Target_State;
    //init ( not needed but just in case )
    Q_Target_State << 0, 0, 0, 0, 0, 0;

    std::ofstream outfile("animationIKTest.csv");


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
    //Target << 0.5, -0.145, 0.2 , 2, -1.0 , 0.321751;
    Target << -0.5, -0.145, 0.2 , 2, -1.0 , 0.321751;

    bool ret = KuKa.ComputeIK( Q_state, Target, Q_Target_State );

    if (!ret)
    {
        std::cout << "InverseKinematics did not find a solution" << std::endl;
    }

    // Write result to file
    outfile
    << 0 << ", "
    << Q_state(0) << ", "
    << Q_state(1) << ", "
    << Q_state(2) << ", "
    << Q_state(3) << ", "
    << Q_state(4) << ", "
    << Q_state(5) << ", "
    << "\n";

    outfile
    << 1 << ", "
    << Q_Target_State(0) << ", "
    << Q_Target_State(1) << ", "
    << Q_Target_State(2) << ", "
    << Q_Target_State(3) << ", "
    << Q_Target_State(4) << ", "
    << Q_Target_State(5) << ", "
    << "\n";

    outfile.close();


    //Vector3d Target;
    //Target << 0.845, 0, 0.425;
    //KuKa.ComputeIK3D( Q_state, Target );

    return 0;
}
