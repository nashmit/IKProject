#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>

#include <rbdl/rbdl.h>

#include "../include/IK.h"

#ifndef RBDL_BUILD_ADDON_LUAMODEL
#error "Error: RBDL addon LuaModel not enabled."
#endif
#include <rbdl/addons/luamodel/luamodel.h>


int main(int argc, char *argv[]) {


    Model model;
    if (!Addons::LuaModelReadFromFile ("kuka.lua", &model, false)) {
        std::cerr << "Error loading lua file" << std::endl;
        abort();
    }

    std::ofstream outfile("animationLoadAndWriteTest.csv");

    // Vector to store the start
    // The Kuka robot has 6dof, the rotating cube has 1, so 6+1=7 DOF in total
    Vector6d q_start;
    Vector6d q_final;
    VectorXd q(6);

    q_start << 0, 0, 0, 0, 0, 0;
    //q_final << EIGEN_PI/2, 0, EIGEN_PI/2, -EIGEN_PI/2, EIGEN_PI, 0;

    // Get the IDs for the end effector given by the name in the lua file
    //end effector
    int EE_id = model.GetBodyId("A6");
    int A4_id = model.GetBodyId("A4");

    Vector3d EE_worldSpace_position = CalcBodyToBaseCoordinates(model,  q_start, EE_id,Vector3d(0,0,0),true);
    Eigen::Matrix3d EE_worldSpace_orientation = CalcBodyWorldOrientation (model,  q_start, EE_id,false);

    std::cout
        <<"Position and orientation of the end effector in world space: \n\nT(x,y,z) and 3x3 Rotation matrix \n " << std::endl
        << EE_worldSpace_position << std::endl << std::endl
        << EE_worldSpace_orientation << std::endl;

    Vector3d A4_worldSpace_position = CalcBodyToBaseCoordinates(model,  q_start, A4_id,Vector3d(0,0,0),true);
    Eigen::Matrix3d A4_wordSpace_orientation = CalcBodyWorldOrientation (model,  q_start, A4_id,false);

    InverseKinematicsConstraintSet cs;
    cs.AddPointConstraint (EE_id,Vector3d(0,0,0), A4_worldSpace_position);
    cs.AddOrientationConstraint  (EE_id, A4_wordSpace_orientation);

    // run Inverse Kinematic
    bool ret =  InverseKinematics(model, q_start, cs, q);

    if (!ret)
    {
        std::cout << "InverseKinematics did not find a solution" << std::endl;
    }

    // Write result to file
    outfile << 0 << ", " << 0 << ", " << 0  << ", " << 0 << ", " << 0 << ", " << 0 << ", " << 0 << ", " << "\n";
    outfile << 1 << ", " << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ", " << q[4] << ", " << q[5] << ", " << "\n";

    outfile.close();

    return 0;
}

