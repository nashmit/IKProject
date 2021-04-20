// NOTE: THIS MAIN STILL NEEDS TO BE WORKED OVER, SINCE THE
// MODEL (models/chain.lua) HAS BEEN MODIFIED!
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>

#include <rbdl/rbdl.h>
#ifndef RBDL_BUILD_ADDON_LUAMODEL
	#error "Error: RBDL addon LuaModel not enabled."
#endif
#include <rbdl/addons/luamodel/luamodel.h>

#include <limits> // for std::numeric_limits<unsigned int>::max() (DEBUGGING)
#include "point.hpp"
#include "fabrik.hpp"

using namespace RigidBodyDynamics;

using namespace Eigen;



// Define a few types to make it easier
typedef Matrix<double, 4, 1>  Vector4d;
typedef Matrix<double, 6, 1>  Vector6d;
typedef Matrix<double, 7, 1>  Vector7d;
typedef Matrix<double, 8, 1>  Vector8d;


int main(int argc, char *argv[]) {


	Model model;
	if (!Addons::LuaModelReadFromFile ("models/chain.lua", &model, false)) {
		std::cerr << "Error loading lua file" << std::endl;
		abort();
	}


	std::ofstream outfile("animation.csv");

	// Get IDs of the links of the robot
	std::vector<unsigned int> linkIDs;

	for(auto const& [key, value]: model.mBodyNameMap)
	{
		if (key != "Base" &&key != "ROOT")
		{
			linkIDs.push_back(value);
		}
	}
	std::sort(linkIDs.begin(), linkIDs.end());
	// For some reason, base ID isnt 0 as expected
	linkIDs.insert(linkIDs.begin(),model.GetBodyId("Base"));

	VectorXd q_start(linkIDs.size());
	VectorXd q(linkIDs.size());
	std::vector<Point3D> origins;
	for(auto linkID : linkIDs)
	{
		std::cout << "Found body " << linkID << " with name: " << model.GetBodyName(linkID) << std::endl;
		auto pos = CalcBodyToBaseCoordinates(model, q, linkID, Vector3d(0,0,0), true);
		std::cout << "Position:\n" << pos << std::endl;
		origins.push_back({pos[0], pos[1], pos[2]});
	}
	std::vector<Point3D> positions;
	for(int i = 0; i < origins.size(); i++)
	{
		if(i == 0)
		{
			positions.push_back(origins[i]);
			continue;
		}
		auto halfLength = origins[i] + (-1) * positions[i-1];
		std::cout << origins[i-1] << origins[i] << halfLength << std::endl;
		positions.push_back(positions[i-1] + 2*halfLength);
	}
	std::cout << "Get points:" << std::endl;
	for(auto position : positions)
	{
		std::cout << "Position:" << position << std::endl;
	}

	// Testing fabrik
	Point3D target = {0, 0, 3};
	auto angles = simpleVersion(positions, target, 0.01);

	std::cout << "Angles check:" << std::endl;
	for(auto angle : angles)
	{
		std::cout << angle << "," << std::endl;
	}
	outfile << 1;
	for(int i = 0; i < angles.size(); i++)
	{
		outfile << ", " << angles[i];
	}
	outfile << "\n";
	// Vector to store the start
	// The Kuka robot has 6dof, the rotating cube has 1, so 6+1=7 DOF in total
	//Vector4d q_start;

	//q_start << 0., 0., 0., 0.;

	/*const double tMax = 4;

	for (double t = 0; t < tMax; t +=0.1) {
		// Rotating angle for the cube
		double alpha = (t-tMax/2)/tMax * 2*M_PI;


		// Run forward kinematics to obtain the cube's position and orientation depending on alpha
		VectorXd q(4);
		q[3]=alpha;

		Vector3d pos = CalcBodyToBaseCoordinates(model,  q, cube_id,Vector3d(0,0,0),true);
		Eigen::Matrix3d ori = CalcBodyWorldOrientation (model,  q, cube_id,false);
		*/

		// Set up the inverse Kinematic constraint set for the kuka model

		/*InverseKinematicsConstraintSet cs;
		cs.AddPointConstraint (tcp_id,Vector3d(0,0,0), pos);
		cs.AddOrientationConstraint  (tcp_id, ori);
		*/

		// run Inverse Kinematic
        //bool ret =  InverseKinematics(model, q_start, cs, q);

		/*if (!ret) {
			std::cout << "InverseKinematics did not find a solution" << std::endl;
		}


		// Write result to file
        outfile << t << ", " << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ", " << q[4] << ", " << q[5] << ", " << alpha << "\n";


		// Set start angles to current result
		q_start = q;

	}*/

	outfile.close();

	return 0;
}
