#include "../include/IK.h"

int main()
{
    KuKa_KR5_R850_D_H KuKa;

    Matrix4x4d HomogeniousMatrix = KuKa.GetHierarchyTransformationMatrix();


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