//
//

#ifndef KUKA_RBDL_D_H_JACOBIAN_H
#define KUKA_RBDL_D_H_JACOBIAN_H


#include "config.h"

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>

#include <rbdl/rbdl.h>

#include "D_H_Parameterization.h"

using namespace RigidBodyDynamics;
using namespace Eigen;
using namespace std;


/*

[ P ] * [ m ] * [ C ] = [F]

where:
[m] is the D-H matrix corresponding to "Q_i" DOF
[P] is the product of the D-H matrices in front of matrix that corresponds to Q_i DOF
[C] is the product of the D-H matricesce that start after the matrix corresponding to Q_i DOF
[F] is the result of the entire Hierarchy

*/

class HierarchyOfDHParameterization
{

protected:
    vector<D_H_Parameterization> D_Hs;

public:

    HierarchyOfDHParameterization& Add_D_H(D_H_Parameterization& D_H);

    int GetSize();

    //Q_i starts from 1 and ends with n == GetSize()
    double DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(int J, int K, int Q_i);

    double DerivativeOf_X_Direction_wrt_Q_i_DOF(int Q_i);

    double DerivativeOf_Y_Direction_wrt_Q_i_DOF(int Q_i);

    double DerivativeOf_Z_Direction_wrt_Q_i_DOF(int Q_i);

    double DerivativeOf_Psi_Direction_wrt_Q_i_DOF(int Q_i);

    double DerivativeOf_Theta_Direction_wrt_Q_i_DOF(int Q_i);

    double DerivativeOf_Phi_Direction_wrt_Q_i_DOF(int Q_i);

    Matrix4x4d MatrixProductInterval(int from, int to);

    MatrixXd GetJacobian();

    // get value of DOF for Joint JointNumber
    double GetQforJoint( int JointNumber );
    void SetQforJoint( int  JointNumber, double value );

    VectorXd GetQ();
    void SetQ( VectorXd Q );
};


#endif //KUKA_RBDL_D_H_JACOBIAN_H
