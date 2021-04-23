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

    HierarchyOfDHParameterization& Add_D_H(D_H_Parameterization& D_H)
    {
        D_Hs.push_back(D_H);
        return *this;
    }

    int GetSize() { return D_Hs.size(); }

    //Q_i starts from 1 and ends with n == GetSize()
    double DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(int J, int K, int Q_i)
    {
        double result =
                MatrixProductInterval( 0, Q_i - 1 - 1 ).row( J ) *
                D_Hs[ Q_i - 1 ].GetDerivativeAtCurrentDHValueAsMatrix() *
                MatrixProductInterval( Q_i, GetSize() - 1 ).col( K );

        return result;
    }

    double DerivativeOf_X_Direction_wrt_Q_i_DOF(int Q_i)
    {
        double result =
                DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(1 - 1, 4 - 1, Q_i);

        return result;
    }

    double DerivativeOf_Y_Direction_wrt_Q_i_DOF(int Q_i)
    {
        double result =
                DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(2 - 1, 4 - 1, Q_i);

        return result;
    }

    double DerivativeOf_Z_Direction_wrt_Q_i_DOF(int Q_i)
    {
        double result =
                DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(3 - 1, 4 - 1, Q_i);

        return result;
    }

    double DerivativeOf_Psi_Direction_wrt_Q_i_DOF(int Q_i)
    {
        double result;

        if (D_Hs[Q_i].GetType() == D_H_Parameterization::Type::Prismatic)
            result = 0;

        if (D_Hs[Q_i].GetType() == D_H_Parameterization::Type::Revolute)
        {
            Matrix4x4d product = MatrixProductInterval(0, GetSize() - 1);

            double F_32 = product(3 - 1, 2 - 1);
            double F_33 = product(3 - 1, 3 - 1);

            double DF_32_DQ_i = DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(3 - 1, 2 - 1, Q_i);
            double DF_33_DQ_i = DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(3 - 1, 3 - 1, Q_i);

            result = 1.0 / (1.0 + (F_32 / F_33) * (F_32 / F_33) ) * (1.0 / F_33 * DF_32_DQ_i - F_32 / (F_33 * F_33) * DF_33_DQ_i);
        }

        return result;
    }

    double DerivativeOf_Theta_Direction_wrt_Q_i_DOF(int Q_i)
    {
        double result;

        if (D_Hs[Q_i].GetType() == D_H_Parameterization::Type::Prismatic)
            result = 0;

        if (D_Hs[Q_i].GetType() == D_H_Parameterization::Type::Revolute)
        {
            Matrix4x4d product = MatrixProductInterval(0, GetSize() - 1);

            double F_32 = product(3 - 1, 2 - 1);
            double F_33 = product(3 - 1, 3 - 1);

            double Psi = atan2(F_32, F_33);

            double N = -product(3 - 1, 1 - 1);
            double D = F_32 * sin( Psi ) + product(3 - 1, 3 - 1) * cos( Psi );

            double DF_32_DQ_i = DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(3 - 1, 2 - 1, Q_i);
            double DF_33_DQ_i = DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(3 - 1, 3 - 1, Q_i);

            double D_Psi_D_Q_i = DerivativeOf_Psi_Direction_wrt_Q_i_DOF(Q_i);

            double D_N_D_Q_i = - DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(3 - 1, 1 - 1, Q_i);

            double D_D_D_Q_i = DF_32_DQ_i * sin( Psi ) + F_32 * cos( Psi ) * D_Psi_D_Q_i +
                               DF_33_DQ_i * cos( Psi ) - F_33 * sin( Psi ) * D_Psi_D_Q_i;

            result = 1.0 / (1 + (N / D) * (N / D) ) * ( 1 / D * D_N_D_Q_i - N / (D * D) * D_D_D_Q_i );
        }

        return result;
    }


    double DerivativeOf_Phi_Direction_wrt_Q_i_DOF(int Q_i)
    {
        double result;

        if (D_Hs[Q_i].GetType() == D_H_Parameterization::Type::Prismatic)
            result = 0;

        if (D_Hs[Q_i].GetType() == D_H_Parameterization::Type::Revolute)
        {
            Matrix4x4d product = MatrixProductInterval(0, GetSize() - 1);

            double F_21 = product(2 - 1, 1 - 1);
            double F_11 = product(1 - 1, 1 - 1);

            double DF_21_DQ_i = DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(2 - 1, 1 - 1, Q_i);
            double DF_11_DQ_i = DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(1 - 1, 1 - 1, Q_i);

            result = 1 / ( 1 + (F_21 / F_11) * (F_21 / F_11) ) * ( 1 / F_11 * DF_21_DQ_i - F_21 / ( F_11 * F_11 ) * DF_11_DQ_i );

        }

        return result;
    }


    Matrix4x4d MatrixProductInterval(int from, int to)
    {
        if (to < from)
            return Matrix4x4d::Ones();

        Matrix4x4d result = Matrix4x4d::Identity();

        for (int i = from; i <= to; i++)
            result = result * D_Hs[i].GetAsHomogeniousMatrix();

        return result;
    }

    MatrixXd GetJacobian()
    {
        MatrixXd Jacobian(6,GetSize() );

        for (int i = 1; i <= GetSize(); i++)
        {
            Jacobian(0, i-1 ) = DerivativeOf_X_Direction_wrt_Q_i_DOF(i);
            Jacobian(1, i-1 ) = DerivativeOf_Y_Direction_wrt_Q_i_DOF(i);
            Jacobian(2, i-1 ) = DerivativeOf_Z_Direction_wrt_Q_i_DOF(i);

            Jacobian(3, i-1 ) = DerivativeOf_Psi_Direction_wrt_Q_i_DOF(i);
            Jacobian(4, i-1 ) = DerivativeOf_Theta_Direction_wrt_Q_i_DOF(i);
            Jacobian(5, i-1 ) = DerivativeOf_Phi_Direction_wrt_Q_i_DOF(i);
        }

        return Jacobian;
    }

};



#endif //KUKA_RBDL_D_H_JACOBIAN_H