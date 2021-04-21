#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>

#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace Eigen;
using namespace std;



// Define a few types to make it easier
typedef Matrix<double, 6, 1>  Vector6d;
typedef Matrix<double, 7, 1>  Vector7d;
typedef Matrix<double, 8, 1>  Vector8d;

typedef Matrix<double, 4, 4> Matrix4x4d;


class D_H_Parameterization
{

public:
  enum class Type { Prismatic, Revolute };

protected:

    //init robot pos
    double alpha;
    double d;
    double a;
    double theta;

    double DOF_Value;

    Type type;

public:

  Type GetType() { return type; }

  D_H_Parameterization(
    Type _type,
    double _alpha,
    double _d,
    double _a,
    double _theta)
  {
    type = _type;

    alpha = _alpha;
    d = _d;
    a = _a;
    theta = _theta;

    DOF_Value = 0;

  }

  Matrix4x4d GetAsHomogeniousMatrix()
  {

      //delta 'Theta'
      double DTheta = 0;
      //delta 'd'
      double Dd = 0;

      if (type==Type::Revolute)
      {
          DTheta = DOF_Value;
      }
      if (type==Type::Prismatic)
      {
          Dd = DOF_Value;
      }

      Matrix4x4d solution;
      solution <<
      cos( theta + DTheta ),    -cos( alpha ) * sin( theta + DTheta ),     sin( alpha ) * sin( theta + DTheta ),    a * cos(theta + DTheta ),
      sin( theta + DTheta ),     cos( alpha ) * cos( theta + DTheta ),    -sin( alpha ) * cos( theta + DTheta ),    a * sin(theta + DTheta ),
            0,                          sin( alpha ),                               cos( alpha ),                             d + Dd,
            0,                                  0,                                          0,                                  1;

      return solution;
  }

  void SetDOF(double DeltaParameter)
  {
      DOF_Value = DeltaParameter;
  }

  double GetDOF()
  {
      return DOF_Value;
  }

  Matrix4x4d GetDerivativeAtCurrentDHValueAsMatrix()
  {
      Matrix4x4d result;

      if (type == Type::Prismatic)
          result <<
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 1,
          0, 0, 0, 0;

      if (type == Type::Revolute)
          result <<
          -sin(theta + DOF_Value ),     -cos( alpha ) * cos( theta + DOF_Value ),   sin( alpha ) * cos( theta + DOF_Value ),    -a * sin( theta + DOF_Value ),
           cos(theta + DOF_Value ),     -cos( alpha ) * sin( theta + DOF_Value ),   sin( alpha ) * sin( theta + DOF_Value ),     a * cos( theta + DOF_Value ),
                    0,                                      0,                                          0,                                          0,
                    0,                                      0,                                          0,                                          0;

      return result;
  }

};


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
                MatrixProductInterval( 0, Q_i - 2 ).row( J ) *
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

            result = 1.0 / (1.0 + (F_32 / F_33) * (F_32 / F_33)) * (1.0 / F_33 * DF_32_DQ_i - F_32 / (F_33 * F_33) * DF_33_DQ_i);
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
            double D = product(3 - 1, 2 - 1) * sin(Psi) + product(3 - 1, 3 - 1) * cos(Psi);

            double DF_32_DQ_i = DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(3 - 1, 2 - 1, Q_i);
            double DF_33_DQ_i = DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(3 - 1, 3 - 1, Q_i);

            double D_Psi_D_Q_i = DerivativeOf_Psi_Direction_wrt_Q_i_DOF(Q_i);

            double D_N_D_Q_i = DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(3 - 1, 1 - 1, Q_i);

            double D_D_D_Q_i = DF_32_DQ_i * sin(Psi) + F_32 * cos(Psi) * D_Psi_D_Q_i +
                    DF_33_DQ_i * cos(Psi) - F_33 * sin(Psi) * D_Psi_D_Q_i;

            result = 1.0 / (1 + (N / D) * (N / D)) * (1 / D * D_N_D_Q_i - N / (D * D) * D_D_D_Q_i);
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

            result = 1 / (1 + (F_21 / F_11) * (F_21 / F_11)) * (1 / F_11 * DF_21_DQ_i - F_21 / (F_11 * F_11) * DF_11_DQ_i);

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
        MatrixXd Jacobian;

        for (int i = 1; i <= GetSize(); i++)
        {
            Jacobian(0, i) = DerivativeOf_X_Direction_wrt_Q_i_DOF(i);
            Jacobian(1, i) = DerivativeOf_Y_Direction_wrt_Q_i_DOF(i);
            Jacobian(2, i) = DerivativeOf_Z_Direction_wrt_Q_i_DOF(i);

            Jacobian(3, i) = DerivativeOf_Psi_Direction_wrt_Q_i_DOF(i);
            Jacobian(4, i) = DerivativeOf_Theta_Direction_wrt_Q_i_DOF(i);
            Jacobian(5, i) = DerivativeOf_Phi_Direction_wrt_Q_i_DOF(i);
        }

        return Jacobian;
    }

};


int main()
{
    Matrix4x4d a;
    a <<
    1, 2, 3, 4,
    5, 6, 7, 8,
    9, 10, 11, 12,
    13, 14, 15, 16;
    std::cout << a << std::endl << std::endl;


    D_H_Parameterization Joint1 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, EIGEN_PI, 0.335, 0.075, 0 );

    D_H_Parameterization Joint2 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, 0, 0, 0.365, EIGEN_PI );

    D_H_Parameterization Joint3 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, EIGEN_PI, 0, 0.090, 0 );

    D_H_Parameterization Joint4 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute,-EIGEN_PI, 0.405, 0, 0 );

    D_H_Parameterization Joint5 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, EIGEN_PI, 0, 0, 0 );

    D_H_Parameterization Joint6 = D_H_Parameterization(
            D_H_Parameterization::Type::Revolute, 0, 0.080,0, 0 );

    std::cout <<
    Joint1.GetAsHomogeniousMatrix() << std::endl <<
    Joint2.GetAsHomogeniousMatrix() << std::endl <<
    Joint3.GetAsHomogeniousMatrix() << std::endl <<
    Joint4.GetAsHomogeniousMatrix() << std::endl <<
    Joint5.GetAsHomogeniousMatrix() << std::endl <<
    Joint6.GetAsHomogeniousMatrix() << std::endl;


    HierarchyOfDHParameterization Hierarchy;

    Hierarchy.Add_D_H(Joint1).Add_D_H(Joint2).Add_D_H(Joint3).Add_D_H(Joint4).Add_D_H(Joint5).Add_D_H(Joint6);

    std::cout << Hierarchy.MatrixProductInterval( 0, Hierarchy.GetSize() ) << std::endl;

    return 0;
}
