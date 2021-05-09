#include "../include/IK.h"


int main()
{

    KuKa_KR5_R850_D_H KuKa;

    KuKa.SetQforJoint(2, EIGEN_PI/3);
    KuKa.SetQforJoint(4, -EIGEN_PI/7);
    KuKa.SetQforJoint(6, EIGEN_PI/8);

    HierarchyOfDHParameterization Hierarchy = KuKa.GetHierarchy();

    //Matrix4x4d HomogeniousMatrix = Hierarchy.MatrixProductInterval(0, Hierarchy.GetHierarchyLength() - 1 );

    std::cout << "Numerical Jacobian:" << std::endl;
    std::cout << Hierarchy.GetNumericalJacobian(0.0001) << std::endl << std::endl;

    std::cout << "Jacobian using D-H parametrization:" << std::endl;
    std::cout << Hierarchy.GetJacobian() << std::endl << std::endl;



    Model model;
    if (!Addons::LuaModelReadFromFile ("LUA/kuka.lua", &model, false)) {
        std::cerr << "Error loading lua file" << std::endl;
        abort();
    }

    /*
    unsigned int A6_id = model.GetBodyId("A6");
    Vector6d Q_state;
    Q_state << 0, 0, 0, 0, 0, 0;
    MatrixXd G( 6, Hierarchy.GetNumberDOF() );
    G.setZero();

    CalcPointJacobian6D( model, Q_state, A6_id, Vector3d::Zero(), G);

    std::cout << "Jacobian using RBDL:" << std::endl;
    std::cout << G << std::endl << std::endl;
    */

    return 0;
}
