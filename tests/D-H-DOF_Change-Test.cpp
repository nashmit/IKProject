#include "../include/IK.h"

int main()
{
    Model model;
    if (!Addons::LuaModelReadFromFile ("LUA/kuka.lua", &model, false)) {
        std::cerr << "Error loading lua file" << std::endl;
        abort();
    }

    Vector6d Q_state;

    //generalized coordinates state values.
    Q_state << EIGEN_PI, 0, -EIGEN_PI/2, 0, 0, EIGEN_PI/2;

    // Get the IDs for the end effector given by the name in the lua file
    //end effector
    int EE_id = model.GetBodyId("A6");

    Vector3d EE_worldSpace_position = CalcBodyToBaseCoordinates(model,  Q_state, EE_id,Vector3d(0,0,0),true);
    Eigen::Matrix3d EE_worldSpace_orientation = CalcBodyWorldOrientation (model,  Q_state, EE_id,true);

    std::cout << "\nEnd effector position (using RBDL): \n"
    << EE_worldSpace_position << std::endl
    << "\nEnd effector orietantion matrix (using RBDL): \n"
    << EE_worldSpace_orientation << std::endl;


    //build Kuka robot
    KuKa_KR5_R850_D_H KuKa;

    //set generalized coordinates
    KuKa.SetQ( Q_state );

    Matrix4x4d HomogeniousMatrix = KuKa.GetHierarchyTransformationMatrix();


    std::cout <<"\nHierarchy transformation matrix (D-H parametrization): \n"
              << HomogeniousMatrix << std::endl << std::endl
              << "Position (D-H parametrization): \n"
              << GetTraslationFromHomogeniousMatrix( HomogeniousMatrix ) << std::endl << std::endl
              << "Orientation (D-H parametrization): \n"
              << GetRotationsFromHomogeniousMatrix( HomogeniousMatrix ) << std::endl << std::endl;


    //Joint orientation test after applying the hierarchy transformation.
    Trihedral Tri;
    std::cout << ExtractRotationMatrix( HomogeniousMatrix ) * Tri;


    return 0;
}
