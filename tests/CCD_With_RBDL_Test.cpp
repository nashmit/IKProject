#include "../include/ccd.h"

#include <map>

//index must be between 1 and 6 for our robot
Vector3d GetAxesFor( Model& model, unsigned index )
{

    for( unsigned i = 0 ; i < model.mJoints[ index ].mDoFCount ; i++ )
    {
        Vector3d Axes;
        Axes << model.mJoints[ index ].mJointAxes[i][0], model.mJoints[ index ].mJointAxes[i][1], model.mJoints[ index ].mJointAxes[i][2];

        return Axes;
    }

    assert(!"you should not get this message!");
}

void ApplyRotation(
        Model& model, unsigned pivot_id, unsigned NodeToRotate_id, Vector3d Target, Vector6d& Qs,
        int nodeIndex, unsigned parent_id, Vector3d PivotAxes )
{
    Vector3d pivot_worldSpace_position = CalcBodyToBaseCoordinates(model,  Qs, pivot_id,Vector3d(0,0,0),true);
    Eigen::Matrix3d pivot_wordSpace_orientation = CalcBodyWorldOrientation (model,  Qs, pivot_id,true);

    Vector3d NodeToRotate_id_worldSpace_position = CalcBodyToBaseCoordinates( model, Qs, NodeToRotate_id, Vector3d(0,0,0),true);
    //Eigen::Matrix3d NodeToRotate_id_wordSpace_orientation = CalcBodyWorldOrientation (model,  Qs, NodeToRotate_id,true);

    Quaterniond pivot_wordSpace_orientation_asQuaternion = Quaterniond().FromTwoVectors(
            NodeToRotate_id_worldSpace_position - pivot_worldSpace_position,
            Target - pivot_worldSpace_position ) * Quaterniond( pivot_wordSpace_orientation );



    //EnforceHinge
    Eigen::Matrix3d parent_wordSpace_orientation = CalcBodyWorldOrientation (model,  Qs, parent_id,true);

    Qs[ nodeIndex ] = AngleAxisd(Quaterniond().FromTwoVectors(
            pivot_wordSpace_orientation_asQuaternion * PivotAxes,
            parent_wordSpace_orientation * PivotAxes ) * pivot_wordSpace_orientation_asQuaternion ).angle();


}



//void EnforceJointLimits()



int main()
{

    Model model;
    if (!Addons::LuaModelReadFromFile ("LUA/kuka.lua", &model, false)) {
        std::cerr << "Error loading lua file" << std::endl;
        abort();
    }

    std::map<int,int> index_id;


    // Vector to store the start
    Vector6d q_start;
    //Vector6d q_final;
    //VectorXd q(6);

    q_start << 0, 0, 0, 0, 0, 0;
    //q_final << EIGEN_PI/2, 0, EIGEN_PI/2, -EIGEN_PI/2, EIGEN_PI, 0;

    // Get the IDs given by the names in the lua file
    //int Base_id = model.GetBodyId("Base");
    int A1_id = model.GetBodyId("A1");
    int A2_id = model.GetBodyId("A2");
    int A3_id = model.GetBodyId("A3");
    int A4_id = model.GetBodyId("A4");
    int A5_id = model.GetBodyId("A5");
    int A6_id = model.GetBodyId("A6");
    //int EE_id = model.GetBodyId("TCP");

    //mapping
    index_id[1]=A1_id;
    index_id[2]=A2_id;
    index_id[3]=A3_id;
    index_id[4]=A4_id;
    index_id[5]=A5_id;
    index_id[6]=A6_id;


    Vector3d Target = CalcBodyToBaseCoordinates(model, q_start, A4_id,Vector3d(0,0,0),true);

    int nrIterationMax = 100;
    int nrCurrentIteration = 0;
    double Epsilon = 0.01;

    while(1)
    {
        nrCurrentIteration++;

        for (int i = 5; i > 0; i--) {
            ApplyRotation(model, index_id[i - 1], index_id[i], Target, q_start, i, index_id[i - 2], GetAxesFor(model, i) );
        }

        Vector3d EE_worldSpace_position = CalcBodyToBaseCoordinates(model,  q_start, A6_id,Vector3d(0,0,0),true);


        //std::cout << "----\n" << EE_worldSpace_position << "----\n" << q_start << std::endl;
        std::cout << q_start << std::endl << "----------" << std::endl;

        std::cout << "Error iteration nr: " << nrCurrentIteration << "\n" << (EE_worldSpace_position - Target).squaredNorm() << "----\n";

        if( (EE_worldSpace_position - Target).squaredNorm() < Epsilon )
            break;

        if( nrCurrentIteration > nrIterationMax )
            break;
    }


    return 0;
}