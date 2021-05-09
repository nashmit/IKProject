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

void KuKa_KR5_R850_D_H::SetQ( VectorXd Q_states )
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

// method for calculating the pseudo-Inverse as recommended by Eigen developers
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}


VectorXd KuKa_KR5_R850_D_H::ComputeIK3D( VectorXd Q_initial, Vector3d Target_position )
{
    Vector6d Q_current = Q_initial;
    Vector6d Q_next = Q_initial;

    int nr = 0;

    while(1)
    {
        nr++;
        Q_current = Q_next;
        Hierarchy.SetQ( Q_current );

        Vector3d Current_Target;
        Current_Target << GetTraslationFromHomogeniousMatrix( GetHierarchyTransformationMatrix() );

        Vector3d DeltaError = Target_position - Current_Target;

        if( abs( DeltaError(0) ) < 0.001 && abs( DeltaError(1) ) < 0.001 && abs( DeltaError(2) ) < 0.001 )
            break;

        std::cout << "Iteration: " << nr << std::endl << DeltaError << std::endl << std::endl;
        if(nr>50)
            break;

        MatrixXd Jacobian = Hierarchy.GetJacobian3D();

        //double det = Jacobian.determinant();
        //assert( det != 0 );

        MatrixXd Inv_Jacobian = pseudoInverse(Jacobian);

        Q_next = Q_current + 0.1 * Inv_Jacobian * DeltaError;

    }
    return Q_current;

}


bool KuKa_KR5_R850_D_H::ComputeIK(
        VectorXd Q_initial, Vector6d Target_position_orientation, Vector6d& Q_out,
        double LinearError , double AngularError , int NrMaxIterations )
{
    Vector6d Q_current = Q_initial;
    Vector6d Q_next = Q_initial;

    int nr = 0;

    while(1)
    {
        nr++;
        Q_current = Q_next;
        Hierarchy.SetQ( Q_current );

        Vector6d Current_Target_position_orientation;
        Current_Target_position_orientation <<
            GetTraslationFromHomogeniousMatrix( GetHierarchyTransformationMatrix() ),
            GetEulerRotationsFromHomogeniousMatrix( GetHierarchyTransformationMatrix() );


        Vector6d DeltaError = Target_position_orientation - Current_Target_position_orientation;

        std::cout << "Iteration: " << nr << " DeltaError:" << std::endl << DeltaError << std::endl << std::endl;

        if( abs( DeltaError(0) ) < LinearError && abs( DeltaError(1) ) < LinearError && abs( DeltaError(2) ) < LinearError &&
            abs( DeltaError(3) ) < AngularError && abs( DeltaError(4) ) < AngularError && abs( DeltaError(5) ) < AngularError )
            break;

        if( nr > NrMaxIterations )
            break;

        //MatrixXd Jacobian = Hierarchy.GetNumericalJacobian(0.00001);
        MatrixXd Jacobian = Hierarchy.GetJacobian();

        std::cout << "Jacobian " << std::endl << Jacobian << std::endl;

        MatrixXd Inv_Jacobian = pseudoInverse(Jacobian);

        Q_next = Q_current + 0.1 * Inv_Jacobian * DeltaError;

    }

    Q_out = Q_current;

    if(nr > 250)
        return false;
    return true;

}