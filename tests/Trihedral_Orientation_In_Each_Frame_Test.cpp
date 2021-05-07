#include "../include/IK.h"

int main()
{
    Vector6d Q_state;
    Q_state << 0, 0, 0, 0, 0, 0;

    /*
    Model model;
    if (!Addons::LuaModelReadFromFile ("LUA/kuka.lua", &model, false)) {
        std::cerr << "Error loading lua file" << std::endl;
        abort();
    }

    int EE_id = model.GetBodyId("A6");

    Vector3d EE_worldSpace_position = CalcBodyToBaseCoordinates(model,  Q_state, EE_id,Vector3d(0,0,0),true);
    Eigen::Matrix3d EE_worldSpace_orientation = CalcBodyWorldOrientation (model,  Q_state, EE_id,true);

    std::cout << "\nEnd effector position (using RBDL): \n"
              << EE_worldSpace_position << std::endl
              << "\nEnd effector orietantion matrix (using RBDL): \n"
              << EE_worldSpace_orientation << std::endl;

    */

    KuKa_KR5_R850_D_H KuKa;

    KuKa.SetQ(Q_state);
    HierarchyOfDHParameterization Hierarchy = KuKa.GetHierarchy();

    Matrix4x4d HomogeniousMatrix = Hierarchy.MatrixProductInterval(0,8 ); //(0, Hierarchy.GetHierarchyLength() - 1 );


    std::cout << "\nHierarchy transformation matrix: \n"
              << HomogeniousMatrix << std::endl << std::endl
              << "Position: \n"
              << GetTraslationFromHomogeniousMatrix( HomogeniousMatrix ) << std::endl << std::endl
              << "Orientation: \n"
              << GetEulerRotationsFromHomogeniousMatrix(HomogeniousMatrix) << std::endl << std::endl;


    //Joint orientation test after applying the hierarchy transformation.
    Trihedral Tri;
    std::cout << ExtractRotationMatrix( HomogeniousMatrix ) * Tri;

    return 0;
}