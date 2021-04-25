/*
 * Extract position and orientation in global space for each Joint for "robot initial position/configuration"
 * This is going to be used to build the Denavit-Hartenberg table corresponding to the robot for the "initial position" used by Kuka lua file.
*/

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

    // Vector to store the start
    Vector6d q_start;
    //Vector6d q_final;
    //VectorXd q(6);

    q_start << 0, 0, 0, 0, 0, 0;
    //q_final << EIGEN_PI/2, 0, EIGEN_PI/2, -EIGEN_PI/2, EIGEN_PI, 0;

    // Get the IDs given by the names in the lua file
    int Base_id = model.GetBodyId("Base");
    int A1_id = model.GetBodyId("A1");
    int A2_id = model.GetBodyId("A2");
    int A3_id = model.GetBodyId("A3");
    int A4_id = model.GetBodyId("A4");
    int A5_id = model.GetBodyId("A5");
    int EE_id = model.GetBodyId("A6");

    Vector3d Base_worldSpace_position = CalcBodyToBaseCoordinates(model,  q_start, Base_id,Vector3d(0,0,0),true);
    Eigen::Matrix3d Base_wordSpace_orientation = CalcBodyWorldOrientation (model,  q_start, Base_id,true);

    Vector3d A1_worldSpace_position = CalcBodyToBaseCoordinates(model,  q_start, A1_id,Vector3d(0,0,0),true);
    Eigen::Matrix3d A1_wordSpace_orientation = CalcBodyWorldOrientation (model,  q_start, A1_id,true);

    Vector3d A2_worldSpace_position = CalcBodyToBaseCoordinates(model,  q_start, A2_id,Vector3d(0,0,0),true);
    Eigen::Matrix3d A2_wordSpace_orientation = CalcBodyWorldOrientation (model,  q_start, A2_id,true);

    Vector3d A3_worldSpace_position = CalcBodyToBaseCoordinates(model,  q_start, A3_id,Vector3d(0,0,0),true);
    Eigen::Matrix3d A3_wordSpace_orientation = CalcBodyWorldOrientation (model,  q_start, A3_id,true);

    Vector3d A4_worldSpace_position = CalcBodyToBaseCoordinates(model,  q_start, A4_id,Vector3d(0,0,0),true);
    Eigen::Matrix3d A4_wordSpace_orientation = CalcBodyWorldOrientation (model,  q_start, A4_id,true);

    Vector3d A5_worldSpace_position = CalcBodyToBaseCoordinates(model,  q_start, A5_id,Vector3d(0,0,0),true);
    Eigen::Matrix3d A5_wordSpace_orientation = CalcBodyWorldOrientation (model,  q_start, A5_id,true);

    Vector3d EE_worldSpace_position = CalcBodyToBaseCoordinates(model,  q_start, EE_id,Vector3d(0,0,0),true);
    Eigen::Matrix3d EE_worldSpace_orientation = CalcBodyWorldOrientation (model,  q_start, EE_id,true);


    std::cout
        << "\nBase: [ Position(x,y,z) and orientation(3x3 rotation matrix) in world space ]\n"
        << Base_worldSpace_position << std::endl << std::endl
        << Base_wordSpace_orientation << std::endl << std::endl
        << "A1: [ Position(x,y,z) and orientation(3x3 rotation matrix) in world space ]\n"
        << A1_worldSpace_position << std::endl << std::endl
        << A1_wordSpace_orientation << std::endl << std::endl
        << "A2: [ Position(x,y,z) and orientation(3x3 rotation matrix) in world space ]\n"
        << A2_worldSpace_position << std::endl << std::endl
        << A2_wordSpace_orientation << std::endl << std::endl
        << "A3: [ Position(x,y,z) and orientation(3x3 rotation matrix) in world space ]\n"
        << A3_worldSpace_position << std::endl << std::endl
        << A3_wordSpace_orientation << std::endl << std::endl
        << "A4: [ Position(x,y,z) and orientation(3x3 rotation matrix) in world space ]\n"
        << A4_worldSpace_position << std::endl << std::endl
        << A4_wordSpace_orientation << std::endl << std::endl
        << "A5: [ Position(x,y,z) and orientation(3x3 rotation matrix) in world space ]\n"
        << A5_worldSpace_position << std::endl << std::endl
        << A5_wordSpace_orientation << std::endl << std::endl
        << "A6 ( End effector ): [ Position(x,y,z) and orientation(3x3 rotation matrix) in world space ]\n"
        << EE_worldSpace_position << std::endl << std::endl
        << EE_worldSpace_orientation << std::endl << std::endl;


    return 0;
}

