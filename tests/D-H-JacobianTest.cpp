#include "../include/IK.h"


int main()
{



    KuKa_KR5_R850_D_H KuKa;

    HierarchyOfDHParameterization Hierarchy = KuKa.GetHierarchy();

    //Matrix4x4d HomogeniousMatrix = Hierarchy.MatrixProductInterval(0, Hierarchy.GetHierarchyLength() - 1 );

    std::cout << Hierarchy.GetJacobian();

    return 0;
}
