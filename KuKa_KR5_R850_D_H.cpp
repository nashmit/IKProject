//
// Created by nash_mit on 23.04.2021.
//

#include "KuKa_KR5_R850_D_H.h"


D_H_Parameterization Build_KuKa_KR5_R850_Joint1() {

    D_H_Parameterization Joint1 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, EIGEN_PI, 0.335, 0.075, 0);

    return Joint1;
}

D_H_Parameterization Build_KuKa_KR5_R850_Joint2() {

    D_H_Parameterization Joint2 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, 0, 0, 0.365, EIGEN_PI);

    return Joint2;
}

D_H_Parameterization Build_KuKa_KR5_R850_Joint3() {

    D_H_Parameterization Joint3 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, EIGEN_PI, 0, 0.090, 0);

    return Joint3;
}

D_H_Parameterization Build_KuKa_KR5_R850_Joint4() {

    D_H_Parameterization Joint4 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, -EIGEN_PI, 0.405, 0, 0);

    return Joint4;
}

D_H_Parameterization Build_KuKa_KR5_R850_Joint5() {

    D_H_Parameterization Joint5 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, EIGEN_PI, 0, 0, 0);

    return Joint5;
}

D_H_Parameterization Build_KuKa_KR5_R850_Joint6() {

    D_H_Parameterization Joint6 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, 0, 0.080, 0, 0);

    return Joint6;
}


HierarchyOfDHParameterization Build_KuKa_KR5_R850_D_H() {

    D_H_Parameterization Joint1 = Build_KuKa_KR5_R850_Joint1();
    D_H_Parameterization Joint2 = Build_KuKa_KR5_R850_Joint2();
    D_H_Parameterization Joint3 = Build_KuKa_KR5_R850_Joint3();
    D_H_Parameterization Joint4 = Build_KuKa_KR5_R850_Joint4();
    D_H_Parameterization Joint5 = Build_KuKa_KR5_R850_Joint5();
    D_H_Parameterization Joint6 = Build_KuKa_KR5_R850_Joint6();

    HierarchyOfDHParameterization Hierarchy;

    Hierarchy.Add_D_H(Joint1).Add_D_H(Joint2).Add_D_H(Joint3).Add_D_H(Joint4).Add_D_H(Joint5).Add_D_H(Joint6);

    return Hierarchy;
}