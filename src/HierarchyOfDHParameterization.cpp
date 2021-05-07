//
//

#include "../include/IKProject/HierarchyOfDHParameterization.h"
#include "../include/IKProject/Utils.h"


HierarchyOfDHParameterization& HierarchyOfDHParameterization::Add_D_H(D_H_Parameterization& D_H)
{
    D_Hs.push_back(D_H);
    return *this;
}

int HierarchyOfDHParameterization::GetHierarchyLength()
{
    return D_Hs.size();
}

bool HierarchyOfDHParameterization::IsDOF_AtIndex(int index)
{
    assert( index >= 0 && index < GetHierarchyLength() );

    if ( D_Hs[ index ].GetType() != D_H_Parameterization::Type::NO_DOF )
        return true;

    return false;
}

int HierarchyOfDHParameterization::GetNumberDOF()
{
    int numberDOF = 0;

    for( unsigned int i = 0; i < D_Hs.size(); i++ ) {
        if( IsDOF_AtIndex( i ) )
            numberDOF++;
    }

    return numberDOF;
}


//Q_i is the Joint index of some DOF in Hierarchy
double HierarchyOfDHParameterization::DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(int J, int K, int Q_i)
{
    double result =
            MatrixProductInterval( 0, Q_i - 1 ).row( J ) *
            D_Hs[ Q_i ].GetDerivativeAtCurrentDHValueAsMatrix() *
            MatrixProductInterval( Q_i + 1, GetHierarchyLength() - 1 ).col(K );

    return result;
}

//Q_i is the Joint index of some DOF in Hierarchy
double HierarchyOfDHParameterization::DerivativeOf_X_Direction_wrt_Q_i_DOF(int Q_i)
{
    double result =
            DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(1 - 1, 4 - 1, Q_i);

    return result;
}

//Q_i is the Joint index of some DOF in Hierarchy
double HierarchyOfDHParameterization::DerivativeOf_Y_Direction_wrt_Q_i_DOF(int Q_i)
{
    double result =
            DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(2 - 1, 4 - 1, Q_i);

    return result;
}

//Q_i is the Joint index of some DOF in Hierarchy
double HierarchyOfDHParameterization::DerivativeOf_Z_Direction_wrt_Q_i_DOF(int Q_i)
{
    double result =
            DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(3 - 1, 4 - 1, Q_i);

    return result;
}

//Q_i is the Joint index of some DOF in Hierarchy
double HierarchyOfDHParameterization::DerivativeOf_Psi_Direction_wrt_Q_i_DOF(int Q_i)
{
    double result;

    if (D_Hs[Q_i].GetType() == D_H_Parameterization::Type::Prismatic)
        result = 0;

    if (D_Hs[Q_i].GetType() == D_H_Parameterization::Type::Revolute)
    {
        Matrix4x4d product = MatrixProductInterval(0, GetHierarchyLength() - 1);

        double F_32 = product(3 - 1, 2 - 1);
        double F_33 = product(3 - 1, 3 - 1);

        double DF_32_DQ_i = DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(3 - 1, 2 - 1, Q_i);
        double DF_33_DQ_i = DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(3 - 1, 3 - 1, Q_i);

        result = 1.0 / (1.0 + (F_32 / F_33) * (F_32 / F_33) ) * (1.0 / F_33 * DF_32_DQ_i - F_32 / (F_33 * F_33) * DF_33_DQ_i);
    }

    return result;
}

//Q_i is the Joint index of some DOF in Hierarchy
double HierarchyOfDHParameterization::DerivativeOf_Theta_Direction_wrt_Q_i_DOF(int Q_i)
{
    double result;

    if (D_Hs[Q_i].GetType() == D_H_Parameterization::Type::Prismatic)
        result = 0;

    if (D_Hs[Q_i].GetType() == D_H_Parameterization::Type::Revolute)
    {
        Matrix4x4d product = MatrixProductInterval(0, GetHierarchyLength() - 1);

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

//Q_i is the Joint index of some DOF in Hierarchy
double HierarchyOfDHParameterization::DerivativeOf_Phi_Direction_wrt_Q_i_DOF(int Q_i)
{
    double result;

    if (D_Hs[Q_i].GetType() == D_H_Parameterization::Type::Prismatic)
        result = 0;

    if (D_Hs[Q_i].GetType() == D_H_Parameterization::Type::Revolute)
    {
        Matrix4x4d product = MatrixProductInterval(0, GetHierarchyLength() - 1);

        double F_21 = product(2 - 1, 1 - 1);
        double F_11 = product(1 - 1, 1 - 1);

        double DF_21_DQ_i = DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(2 - 1, 1 - 1, Q_i);
        double DF_11_DQ_i = DerivativeOf_F_At_J_and_K_position_wrt_Q_i_DOF(1 - 1, 1 - 1, Q_i);

        result = 1 / ( 1 + (F_21 / F_11) * (F_21 / F_11) ) * ( 1 / F_11 * DF_21_DQ_i - F_21 / ( F_11 * F_11 ) * DF_11_DQ_i );

    }

    return result;
}


Matrix4x4d HierarchyOfDHParameterization::MatrixProductInterval(int from, int to)
{
    if (to < from)
        return Matrix4x4d::Identity();

    Matrix4x4d result = Matrix4x4d::Identity();

    for (int i = from; i <= to; i++)
        result = result * D_Hs[i].GetAsHomogeniousMatrix();

    return result;
}

MatrixXd HierarchyOfDHParameterization::GetJacobian()
{
    MatrixXd Jacobian(6, GetNumberDOF() );

    for (int i = 1; i <= GetNumberDOF(); i++)
    {
        int JointIndexForJointNumber = GetJointIndexInHierarchyForJointNumber(i);

        Jacobian(0, i - 1 ) = DerivativeOf_X_Direction_wrt_Q_i_DOF( JointIndexForJointNumber );
        Jacobian(1, i - 1 ) = DerivativeOf_Y_Direction_wrt_Q_i_DOF( JointIndexForJointNumber );
        Jacobian(2, i - 1 ) = DerivativeOf_Z_Direction_wrt_Q_i_DOF( JointIndexForJointNumber );

        Jacobian(3, i - 1 ) = DerivativeOf_Psi_Direction_wrt_Q_i_DOF( JointIndexForJointNumber );
        Jacobian(4, i - 1 ) = DerivativeOf_Theta_Direction_wrt_Q_i_DOF( JointIndexForJointNumber );
        Jacobian(5, i - 1 ) = DerivativeOf_Phi_Direction_wrt_Q_i_DOF( JointIndexForJointNumber );

    }

    return Jacobian;
}


MatrixXd  HierarchyOfDHParameterization::GetNumericalJacobian(double delta)
{
    MatrixXd Jacobian(6, GetNumberDOF() );

    VectorXd Default_Q_States = GetQ();

    for (int i = 1; i <= GetNumberDOF(); i++)
    {
        //apply +delta to joint "i" and computer the hierarchy
        SetQ( Default_Q_States );
        SetQforJoint( i, Default_Q_States( i - 1 ) + delta );
        Matrix4x4d HomogeniousMatrixPlusDelta = MatrixProductInterval(0, GetHierarchyLength() - 1 );

        //apply -delta to joint "i" and computer the hierarchy
        SetQ( Default_Q_States );
        SetQforJoint( i, Default_Q_States( i - 1 ) - delta );
        //SetQforJoint( i, Default_Q_States( i - 1 ) );
        Matrix4x4d HomogeniousMatrixMinusDelta = MatrixProductInterval(0, GetHierarchyLength() - 1 );

        //extract translation and rotation for hierarchy with +delta
        Vector3d Translation_PlusDelta = GetTraslationFromHomogeniousMatrix( HomogeniousMatrixPlusDelta );
        Vector3d Rotations_PlusDelta = GetEulerRotationsFromHomogeniousMatrix( HomogeniousMatrixPlusDelta );

        //extract translation and rotation for hierarchy with -delta
        Vector3d Translation_MinusDelta = GetTraslationFromHomogeniousMatrix( HomogeniousMatrixMinusDelta );
        Vector3d Rotation_MinusDelta = GetEulerRotationsFromHomogeniousMatrix( HomogeniousMatrixMinusDelta );

        Vector3d DeltaTranslation = Translation_PlusDelta - Translation_MinusDelta;
        Vector3d DeltaRotation = Rotations_PlusDelta - Rotation_MinusDelta;


        Jacobian( 0, i - 1 ) = DeltaTranslation( 0 ) / ( 2 * delta );
        Jacobian( 1, i - 1 ) = DeltaTranslation( 1 ) / ( 2 * delta );
        Jacobian( 2, i - 1 ) = DeltaTranslation( 2 ) / ( 2 * delta );

        Jacobian( 3, i - 1 ) = DeltaRotation( 0 ) / ( 2 * delta );
        Jacobian( 4, i - 1 ) = DeltaRotation( 1 ) / ( 2 * delta );
        Jacobian( 5, i - 1 ) = DeltaRotation( 2 ) / ( 2 * delta );


        /*
        Jacobian( 0, i - 1 ) = DeltaTranslation( 0 ) / ( delta );
        Jacobian( 1, i - 1 ) = DeltaTranslation( 1 ) / ( delta );
        Jacobian( 2, i - 1 ) = DeltaTranslation( 2 ) / ( delta );

        Jacobian( 3, i - 1 ) = DeltaRotation( 0 ) / ( delta );
        Jacobian( 4, i - 1 ) = DeltaRotation( 1 ) / ( delta );
        Jacobian( 5, i - 1 ) = DeltaRotation( 2 ) / ( delta );
        */
    }

    SetQ( Default_Q_States );

    return Jacobian;

}

//JointNumber starts from 1
int HierarchyOfDHParameterization::GetJointIndexInHierarchyForJointNumber(int JointNumber)
{
    assert( JointNumber >= 1 && JointNumber <= GetNumberDOF() );

    int currentJoint = 0;

    for(int i = 0; i < GetHierarchyLength(); i++)
    {
        if( IsDOF_AtIndex( i ) )
            currentJoint++;

        if( currentJoint == JointNumber )
            return i;
    }

    assert(!"JointNumber too big!");
}

// joint number starts with 1
double HierarchyOfDHParameterization::GetQforJoint(int JointNumber)
{
    assert( JointNumber >= 1 && JointNumber <= GetNumberDOF() );

    return D_Hs[ GetJointIndexInHierarchyForJointNumber( JointNumber ) ].GetDOF();
}

// joint number starts with 1
void HierarchyOfDHParameterization::SetQforJoint( int  JointNumber, double value )
{
    assert( JointNumber >= 1 && JointNumber <= GetNumberDOF() );

    D_Hs[ GetJointIndexInHierarchyForJointNumber( JointNumber ) ].SetDOF( value );
}

void HierarchyOfDHParameterization::PrintJointsType()
{
    std::cout << "\n-----\nPrint current Joints Type: \n";

    for(int i = 0; i < GetHierarchyLength(); i++)
        std::cout <<(int)D_Hs[i].GetType() << " ";
    std::cout << "\n-----" << std::endl;
}

VectorXd HierarchyOfDHParameterization::GetQ()
{
    VectorXd Q(GetNumberDOF() );

    // joint number starts with 1
    for(int i = 0; i < GetNumberDOF(); i++)
        Q[ i ] = D_Hs[ GetJointIndexInHierarchyForJointNumber(i + 1 ) ].GetDOF();

    return Q;
}

void HierarchyOfDHParameterization::SetQ( VectorXd Q )
{
    assert( Q.size() == GetNumberDOF() );

    // joint number starts with 1
    for(int i = 0; i < GetNumberDOF(); i++)
        D_Hs[ GetJointIndexInHierarchyForJointNumber(i + 1 ) ].SetDOF( Q[ i ] );
}