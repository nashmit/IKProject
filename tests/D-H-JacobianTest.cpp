#include "../include/IK.h"


int main()
{



    KuKa_KR5_R850_D_H KuKa;

    HierarchyOfDHParameterization Hierarchy = KuKa.GetHierarchy();

    //Matrix4x4d HomogeniousMatrix = Hierarchy.MatrixProductInterval(0, Hierarchy.GetHierarchyLength() - 1 );

    std::cout << Hierarchy.GetNumericalJacobian(0.1) << std::endl << std::endl;

    std::cout << Hierarchy.GetJacobian() << std::endl << std::endl;


    return 0;
}
