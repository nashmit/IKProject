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

using namespace RigidBodyDynamics;

using namespace Eigen;



// Define a few types to make it easier
typedef Matrix<double, 6, 1>  Vector6d;
typedef Matrix<double, 7, 1>  Vector7d;
typedef Matrix<double, 8, 1>  Vector8d;


int main(int argc, char *argv[]) {
	
	
	Model model;
	if (!Addons::LuaModelReadFromFile ("kuka+cube.lua", &model, false)) {
		std::cerr << "Error loading lua file" << std::endl;
		abort();
	}

	std::ofstream outfile("animation.csv");

	
	// Get the IDs for the edeffector given by the name in the lua file
	int cube_id = model.GetBodyId("cube");
	int tcp_id = model.GetBodyId("TCP");
	

	// Vector to store the start 
	// The Kuka robot has 6dof, the rotating cube has 1, so 6+1=7 DOF in total
	Vector7d q_start;
	
	q_start << 0, -M_PI/2., M_PI/2, M_PI, 0, 0,0;	
	
	const double tMax = 4;
	
	for (double t = 0; t < tMax; t +=0.1) {
		// Rotating angle for the cube
		double alpha = (t-tMax/2)/tMax * 2*M_PI;
		
		
		// Run forward kinematics to obtain the cube's position and orientation depending on alpha
		VectorXd q(7);
		q[6]=alpha;
		
		Vector3d pos = CalcBodyToBaseCoordinates(model,  q, cube_id,Vector3d(0,0,0),true);
		Eigen::Matrix3d ori = CalcBodyWorldOrientation (model,  q, cube_id,false);
		
		
		// Set up the inverse Kinematic constraint set for the kuka model
		
		InverseKinematicsConstraintSet cs;
		cs.AddPointConstraint (tcp_id,Vector3d(0,0,0), pos);
		cs.AddOrientationConstraint  (tcp_id, ori);

		// run Inverse Kinematic
        bool ret =  InverseKinematics(model, q_start, cs, q);
        
		if (!ret) {
			std::cout << "InverseKinematics did not find a solution" << std::endl;
		}


		// Write result to file
        outfile << t << ", " << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ", " << q[4] << ", " << q[5] << ", " << alpha << "\n";
		
		
		// Set start angles to current result
		q_start = q;
		
	}
		
	outfile.close();
	
	return 0;
}

