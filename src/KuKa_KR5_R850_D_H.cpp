//
// Created by nash_mit on 23.04.2021.
//

#include "../include/IKProject/KuKa_KR5_R850_D_H.h"

D_H_Parameterization KuKa_KR5_R850_D_H::Build_KuKa_KR5_R850_JointAuxiliary1() {

    D_H_Parameterization JointAuxiliary1 = D_H_Parameterization(
            D_H_Parameterization::Type::NO_DOF, EIGEN_PI, 0.203, 0, -EIGEN_PI/2 );

    return JointAuxiliary1;
}

D_H_Parameterization KuKa_KR5_R850_D_H::Build_KuKa_KR5_R850_Joint1() {

    D_H_Parameterization Joint1 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, EIGEN_PI/2, -0.132, 0.075, -EIGEN_PI/2 );

    return Joint1;
}

D_H_Parameterization KuKa_KR5_R850_D_H::Build_KuKa_KR5_R850_Joint2() {

    D_H_Parameterization Joint2 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, 0, 0, 0.365, 0 );

    return Joint2;
}

D_H_Parameterization KuKa_KR5_R850_D_H::Build_KuKa_KR5_R850_Joint3() {

    D_H_Parameterization Joint3 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, EIGEN_PI/2, 0, 0.09, -EIGEN_PI/2 );

    return Joint3;
}

D_H_Parameterization KuKa_KR5_R850_D_H::Build_KuKa_KR5_R850_Joint4() {

    D_H_Parameterization Joint4 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, 0, -0.208, 0, 0 );

    return Joint4;
}

D_H_Parameterization KuKa_KR5_R850_D_H::Build_KuKa_KR5_R850_JointAuxiliary2() {

    D_H_Parameterization JointAuxiliary2 = D_H_Parameterization(
            D_H_Parameterization::Type::NO_DOF,-EIGEN_PI/2,  -0.197, 0, 0 );

    return JointAuxiliary2;
}


D_H_Parameterization KuKa_KR5_R850_D_H::Build_KuKa_KR5_R850_Joint5() {

    D_H_Parameterization Joint5 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, -EIGEN_PI/2, 0, 0, 0 );

    return Joint5;
}


D_H_Parameterization KuKa_KR5_R850_D_H::Build_KuKa_KR5_R850_JointAuxiliary3() {

    D_H_Parameterization JointAuxiliary3 = D_H_Parameterization(
            D_H_Parameterization::Type::NO_DOF, 0, 0.08, 0, 0 );

    return JointAuxiliary3;
}


D_H_Parameterization KuKa_KR5_R850_D_H::Build_KuKa_KR5_R850_Joint6() {

    D_H_Parameterization Joint6 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, 0, 0, 0, 0 );

    return Joint6;
}


HierarchyOfDHParameterization KuKa_KR5_R850_D_H::Build_KuKa_KR5_R850_D_H() {

    D_H_Parameterization JointAuxiliary1 = Build_KuKa_KR5_R850_JointAuxiliary1();

    D_H_Parameterization Joint1 = Build_KuKa_KR5_R850_Joint1();
    D_H_Parameterization Joint2 = Build_KuKa_KR5_R850_Joint2();
    D_H_Parameterization Joint3 = Build_KuKa_KR5_R850_Joint3();
    D_H_Parameterization Joint4 = Build_KuKa_KR5_R850_Joint4();

    D_H_Parameterization JointAuxiliary2 = Build_KuKa_KR5_R850_JointAuxiliary2();

    D_H_Parameterization Joint5 = Build_KuKa_KR5_R850_Joint5();

    D_H_Parameterization JointAuxiliary3 = Build_KuKa_KR5_R850_JointAuxiliary3();

    D_H_Parameterization Joint6 = Build_KuKa_KR5_R850_Joint6();



    //build hierarchy
    Hierarchy
            //extension of D-H parametrization to accommodate a different position of joint wrt the original hierarchy
            .Add_D_H( JointAuxiliary1 )
            .Add_D_H( Joint1 )
            .Add_D_H( Joint2 )
            .Add_D_H( Joint3 )
            .Add_D_H( Joint4 )
            //extension of D-H parametrization to accommodate a different position of joint wrt the original hierarchy
            .Add_D_H( JointAuxiliary2 )
            .Add_D_H( Joint5 )
            //extension of D-H parametrization to accommodate a different position of joint wrt the original hierarchy
            .Add_D_H( JointAuxiliary3 )
            .Add_D_H( Joint6 );


    return Hierarchy;
}

KuKa_KR5_R850_D_H::KuKa_KR5_R850_D_H()
{
    Build_KuKa_KR5_R850_D_H();
}

Matrix4x4d KuKa_KR5_R850_D_H::GetHierarchyTransformationMatrix()
{
    //hierarchy transformation matrix
    Matrix4x4d mat = Hierarchy.MatrixProductInterval(0, Hierarchy.GetHierarchyLength() - 1 );

    return mat;
}

HierarchyOfDHParameterization KuKa_KR5_R850_D_H::GetHierarchy()
{
    return Hierarchy;
}

void KuKa_KR5_R850_D_H::SetQ( Vector6d Q_states )
{
    Hierarchy.SetQ( Q_states );
}

Vector6d KuKa_KR5_R850_D_H::GetQ()
{
    return Hierarchy.GetQ();
}

void KuKa_KR5_R850_D_H::SetQforJoint( int  JointNumber, double value )
{
    Hierarchy.SetQforJoint( JointNumber, value );
}

double KuKa_KR5_R850_D_H::GetQforJoint(int JointNumber)
{
    return Hierarchy.GetQforJoint( JointNumber );
}