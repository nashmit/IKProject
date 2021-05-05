#include "../include/IK.h"

int main()
{
    KuKa_KR5_R850_D_H KuKa;

    HierarchyOfDHParameterization Hierarchy = KuKa.GetHierarchy();

    Matrix4x4d HomogeniousMatrix = Hierarchy.MatrixProductInterval(0,8 ); //(0, Hierarchy.GetHierarchyLength() - 1 );


    std::cout <<"\nHierarchy transformation matrix: \n"
              << HomogeniousMatrix << std::endl << std::endl
              << "Position: \n"
              << GetTraslationFromHomogeniousMatrix( HomogeniousMatrix ) << std::endl << std::endl
              << "Orientation: \n"
              << GetRotationsFromHomogeniousMatrix( HomogeniousMatrix ) << std::endl << std::endl;


    //Joint orientation test after applying the hierarchy transformation.
    Trihedral Tri;
    std::cout << ExtractRotationMatrix( HomogeniousMatrix ) * Tri;

    return 0;
}