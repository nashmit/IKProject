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
#include <cmath> // for sin() and cos()
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

	// Create the COLUMNS header for the animation file
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
	// Sort them (based on hierarchy)
	std::sort(linkIDs.begin(), linkIDs.end());

	// Store the initial positions of the joints as declared in the lua file,
	// as well as their degrees of freedom
	VectorXd q_start(linkIDs.size());
	VectorXd q(linkIDs.size());
	std::vector<Point3D> positions;

	std::stringstream DOFoverview(Utils::GetModelDOFOverview(model));
	std::string line, rotationAxes;
	for(int i = 0; i < linkIDs.size(); i++)
	{
		auto linkID = linkIDs[i];
		auto pos = CalcBodyToBaseCoordinates(model, q, linkID, Vector3d(0,0,0), true);
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

	// Create the DATA columns that describe the movement of the robot
	outfile << "\nDATA:\n";

	std::cout << "Get points:" << std::endl;
	for(auto position : positions)
	{
		std::cout << "Position:" << position << std::endl;
	}

	// Get joint angles using the FABRIK algorithm
	auto startPositions = positions;
	Point3D target;
	double radius = 2;
	for(double angle = 0; angle <= 2*M_PI; angle += M_PI/20)
	{
		if (angle <= M_PI/2)
		{
			target = {0, radius*cos(angle), radius*sin(angle) + 1};
		} else if (angle > M_PI/2 && angle <=  M_PI)
		{
			target = {0, -radius*sin(angle - M_PI/2), radius*cos(angle - M_PI/2) + 1};
		} else if (angle > M_PI && angle <= 1.5*M_PI)
		{
			target = {0, -radius*cos(angle - M_PI), -radius*sin(angle - M_PI) + 1};
		} else if (angle > 1.5*M_PI){
			target = {0, radius*sin(angle - 1.5*M_PI), -radius*cos(angle - 1.5*M_PI) + 1};
		}

		auto angles = simpleVersion(positions, startPositions, target, 0.0001, rotationAxes);

		std::cout << "Angles (in °):" << std::endl;
		for(auto linkAngle : angles)
		{
			std::cout << linkAngle << "," << std::endl;
		}
		// write result into animation file
		outfile << angle;
		for(int i = 0; i < angles.size(); i++)
		{
			outfile << ", " << angles[i];
		}
		outfile << "\n";

	}
	outfile.close();

	return 0;
}
