#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

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
	// Create animation file (COLUMNS header + the actual animation data)
	std::ofstream outfile("animation.csv");

	outfile << "COLUMNS:\nTime, ";
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
	VectorXd q_start(linkIDs.size());
	VectorXd q(linkIDs.size());
	std::vector<Point3D> positions;

	std::stringstream DOFoverview(Utils::GetModelDOFOverview(model));
	std::string line, rotationAxes;
	for(int i = 0; i < linkIDs.size(); i++)
	{
		auto linkID = linkIDs[i];
		//std::cout << "Found body " << linkID << " with name: " << model.GetBodyName(linkID) << std::endl;
		auto pos = CalcBodyToBaseCoordinates(model, q, linkID, Vector3d(0,0,0), true);
		//std::cout << "Position:\n" << pos << std::endl;
		positions.push_back({pos[0], pos[1], pos[2]});
		if (i != linkIDs.size() - 1)
		{
			std::getline(DOFoverview, line);
			rotationAxes += line.substr(line.size() - 1, 1);
			outfile << model.GetBodyName(linkID)   << ":R:"
							<< rotationAxes.back();
		}
		if (i < linkIDs.size() - 2)
		{
			outfile << ", ";
		}
	}
	outfile << "\nDATA:\n";

	std::cout << "Get points:" << std::endl;
	for(auto position : positions)
	{
		std::cout << "Position:" << position << std::endl;
	}

	// Testing fabrik algorithm
	Point3D target = {0, 0, 3};
	auto angles = simpleVersion(positions, target, 0.0001, rotationAxes);

	std::cout << "Angles (in °):" << std::endl;
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
