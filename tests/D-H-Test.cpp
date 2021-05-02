#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <rbdl/rbdl.h>

#include "../include/IK.h"



int main()
{
    /*
    Matrix4x4d a;
    a <<
      1, 2, 3, 4,
            5, 6, 7, 8,
            9, 10, 11, 12,
            13, 14, 15, 16;
    std::cout << a << std::endl << std::endl;
    */


    D_H_Parameterization Joint1 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, -EIGEN_PI/2, 0.132, 0.075, EIGEN_PI/2 );

    D_H_Parameterization Joint2 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, 0, 0, 0.365, 0 );


    std::cout << Joint1.GetAsHomogeniousMatrix() << std::endl
    << GetTraslationFromHomogeniousMatrix( Joint1.GetAsHomogeniousMatrix() ) << std::endl
    << GetRotationsFromHomogeniousMatrix( Joint1.GetAsHomogeniousMatrix() ) << std::endl << std::endl;

    std::cout << Joint2.GetAsHomogeniousMatrix() << std::endl
              << GetTraslationFromHomogeniousMatrix( Joint2.GetAsHomogeniousMatrix() ) << std::endl
              << GetRotationsFromHomogeniousMatrix( Joint2.GetAsHomogeniousMatrix() );

    /*

    D_H_Parameterization Joint2 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, 0, 0, 0.365, EIGEN_PI );

    D_H_Parameterization Joint3 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, EIGEN_PI, 0, 0.090, 0 );

    D_H_Parameterization Joint4 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute,-EIGEN_PI, 0.405, 0, 0 );

    D_H_Parameterization Joint5 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, EIGEN_PI, 0, 0, 0 );

    D_H_Parameterization Joint6 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, 0, 0.080,0, 0 );

    */


    /*
    std::cout <<
    Joint1.GetAsHomogeniousMatrix() << std::endl <<
    Joint2.GetAsHomogeniousMatrix() << std::endl <<
    Joint3.GetAsHomogeniousMatrix() << std::endl <<
    Joint4.GetAsHomogeniousMatrix() << std::endl <<
    Joint5.GetAsHomogeniousMatrix() << std::endl <<
    Joint6.GetAsHomogeniousMatrix() << std::endl;
    */


    /*

    HierarchyOfDHParameterization Hierarchy;

    Hierarchy.Add_D_H(Joint1).Add_D_H(Joint2).Add_D_H(Joint3).Add_D_H(Joint4).Add_D_H(Joint5).Add_D_H(Joint6);

    std::cout << Hierarchy.MatrixProductInterval( 0, Hierarchy.GetSize() - 1 ) << std::endl << std::endl;

    std::cout << "Compute Jacobian: " << std::endl << Hierarchy.GetJacobian();


    std::cout << std::endl << std::endl;

    HierarchyOfDHParameterization KuKA = Build_KuKa_KR5_R850_D_H();

    std::cout << KuKA.MatrixProductInterval( 0, KuKA.GetSize() - 1 ) << std::endl << std::endl;

    std::cout << "Compute Jacobian: " << std::endl << KuKA.GetJacobian();

    */


    //HierarchyOfDHParameterization HierarchyTest1;
    //HierarchyTest1.Add_D_H(Joint1);


    /*
    std::cout << Joint1.GetAsHomogeniousMatrix() << std::endl;
    std::cout
            << "Translation: " << std::endl
            << GetTraslationFromHomogeniousMatrix( Joint1.GetAsHomogeniousMatrix() )  <<  std::endl
            << std::endl << "Rotations " << std::endl
            << GetRotationsFromHomogeniousMatrix( Joint1.GetAsHomogeniousMatrix() ) << std::endl;
    */



    /*
    std::cout << Hierarchy.MatrixProductInterval( 0, Hierarchy.GetSize() - 1 ) << std::endl << std::endl;
    std::cout << GetTraslationFromHomogeniousMatrix( Hierarchy.MatrixProductInterval( 0, Hierarchy.GetSize() - 1 ) ) << std::endl << std::endl;
    std::cout << GetRotationsFromHomogeniousMatrix( Hierarchy.MatrixProductInterval( 0, Hierarchy.GetSize() - 1 ) ) << std::endl << std::endl;
    */



    return 0;
}
