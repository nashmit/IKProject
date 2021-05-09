//
// Created by nash_mit on 23.04.2021.
//

#ifndef KUKA_RBDL_KUKA_KR5_R850_D_H_H
#define KUKA_RBDL_KUKA_KR5_R850_D_H_H

#include "../IK.h"

class KuKa_KR5_R850_D_H{

protected:

    HierarchyOfDHParameterization Hierarchy;

    HierarchyOfDHParameterization Build_KuKa_KR5_R850_D_H();

public:

    D_H_Parameterization Build_KuKa_KR5_R850_JointAuxiliary1();

    D_H_Parameterization Build_KuKa_KR5_R850_Joint1();
    D_H_Parameterization Build_KuKa_KR5_R850_Joint2();
    D_H_Parameterization Build_KuKa_KR5_R850_Joint3();
    D_H_Parameterization Build_KuKa_KR5_R850_Joint4();

    D_H_Parameterization Build_KuKa_KR5_R850_JointAuxiliary2();

    D_H_Parameterization Build_KuKa_KR5_R850_Joint5();
    D_H_Parameterization Build_KuKa_KR5_R850_Joint6();

    D_H_Parameterization Build_KuKa_KR5_R850_JointAuxiliary3();


    KuKa_KR5_R850_D_H();

    Matrix4x4d GetHierarchyTransformationMatrix();
    HierarchyOfDHParameterization GetHierarchy();

    void SetQ( VectorXd Q_states );
    Vector6d GetQ();

    void SetQforJoint( int  JointNumber, double value );
    double GetQforJoint( int JointNumber );

    bool ComputeIK(
            VectorXd Q_initial, Vector6d Target_position_orientation, Vector6d& Q_out,
            double LinearError = 0.001, double AngularError = 0.001, int NrMaxIterations = 250 );

    VectorXd ComputeIK3D( VectorXd Q_initial, Vector3d Target_position );

};


#endif //KUKA_RBDL_KUKA_KR5_R850_D_H_H
