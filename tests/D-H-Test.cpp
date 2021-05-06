#include <iostream>

#include <rbdl/rbdl.h>

#include "../include/IK.h"


int main()
{

    D_H_Parameterization JointAuxiliary1 = D_H_Parameterization(
            D_H_Parameterization::Type::NO_DOF, EIGEN_PI, 0.203, 0, -EIGEN_PI/2 );

    D_H_Parameterization Joint1 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, EIGEN_PI/2, -0.132, 0.075, -EIGEN_PI/2 );

    D_H_Parameterization Joint2 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, 0, 0, 0.365, 0 );

    D_H_Parameterization Joint3 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, EIGEN_PI/2, 0, 0.09, -EIGEN_PI/2 );

    D_H_Parameterization Joint4 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, 0, -0.208, 0, 0 );

    D_H_Parameterization JointAuxiliary2 = D_H_Parameterization(
            D_H_Parameterization::Type::NO_DOF,-EIGEN_PI/2,  -0.197, 0, 0 );

    D_H_Parameterization Joint5 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, -EIGEN_PI/2, 0, 0, 0 );

    D_H_Parameterization JointAuxiliary3 = D_H_Parameterization(
            D_H_Parameterization::Type::NO_DOF, 0, 0.08, 0, 0 );

    D_H_Parameterization Joint6 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, 0, 0, 0, 0 );



    /*
    std::cout << Joint1.GetAsHomogeniousMatrix() << std::endl
              << GetTraslationFromHomogeniousMatrix( Joint1.GetAsHomogeniousMatrix() ) << std::endl
              << GetRotationsFromHomogeniousMatrix( Joint1.GetAsHomogeniousMatrix() ) << std::endl << std::endl;

    std::cout << Joint2.GetAsHomogeniousMatrix() << std::endl
                << GetTraslationFromHomogeniousMatrix( Joint2.GetAsHomogeniousMatrix() ) << std::endl
                << GetRotationsFromHomogeniousMatrix( Joint2.GetAsHomogeniousMatrix() ) << std::endl << std::endl;

    std::cout << Joint3.GetAsHomogeniousMatrix() << std::endl
                << GetTraslationFromHomogeniousMatrix( Joint3.GetAsHomogeniousMatrix() ) << std::endl << std::endl
                << GetEulerRotationsFromHomogeniousMatrix( Joint3.GetAsHomogeniousMatrix() );
    */


    //build hierarchy
    HierarchyOfDHParameterization Hierarchy;

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


    //std::cout << "\nSet Q-value to PI/2 for Joint 5: \n";
    //Hierarchy.SetQforJoint(1, EIGEN_PI/2);

    //hierarchy transformation matrix
    Matrix4x4d HomogeniousMatrix = Hierarchy.MatrixProductInterval(0, Hierarchy.GetHierarchyLength() - 1 );

    std::cout << "\nHierarchy transformation matrix: \n"
              << HomogeniousMatrix << std::endl << std::endl
              << "Position: \n"
              << GetTraslationFromHomogeniousMatrix( HomogeniousMatrix ) << std::endl << std::endl
              << "Orientation: \n"
              << GetEulerRotationsFromHomogeniousMatrix(HomogeniousMatrix) << std::endl << std::endl;


    //Joint orientation test after applying the hierarchy transformation.
    Trihedral Tri;
    std::cout << ExtractRotationMatrix( HomogeniousMatrix ) * Tri;

    Hierarchy.PrintJointsType();

    std::cout << "\nCurrent Q-value for joint 5: " << Hierarchy.GetQforJoint(5) << std::endl;

    std::cout << "\nSet Q-value to PI for Joint 5: \n";
    Hierarchy.SetQforJoint(5, EIGEN_PI);

    std::cout << "\nCurrent Q-value for joint 2: " << Hierarchy.GetQforJoint(5) << std::endl;

    std::cout << "\nCurrent Q state-values for all DOF: \n"
              << Hierarchy.GetQ() << std::endl;

    std::cout << "\nSetQ state-values for all DOF with: 0 0 0 0 0 0\n";
    VectorXd Q_States(6);
    Q_States << 0, 0, 0, 0, 0, 0;
    Hierarchy.SetQ( Q_States );

    std::cout << "\nCurrent Q state-values for all DOF: \n"
              << Hierarchy.GetQ() << std::endl;

    return 0;
}
