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
        int pivotIndex, unsigned parent_id, Vector3d PivotAxes, Vector3d ParentAxes, int parentIndex )
{
    //Vector3d pivot_worldSpace_position = CalcBodyToBaseCoordinates(model,  Qs, pivot_id,Vector3d(0,0,0),true);
    //local position in pivot frame
    Vector3d TargetInPivotCoordinatesSpace = CalcBaseToBodyCoordinates(model, Qs, pivot_id, Target, true );

    //Eigen::Matrix3d pivot_wordSpace_orientation = CalcBodyWorldOrientation (model,  Qs, pivot_id,true);
    //local rotation
    Quaterniond pivot_local_orientation(AngleAxisd( Qs[ pivotIndex ], PivotAxes ) );

    //Vector3d NodeToRotate_id_worldSpace_position = CalcBodyToBaseCoordinates( model, Qs, NodeToRotate_id, Vector3d(0,0,0),true);
    Vector3d NodeToRotateInGlobalSpace = CalcBodyToBaseCoordinates(model, Qs, NodeToRotate_id, Vector3d::Zero(), true);
    Vector3d NodeToRotateInPivotCoordinatesSpace = CalcBaseToBodyCoordinates(model, Qs, pivot_id, NodeToRotateInGlobalSpace, true);


    //Quaterniond pivot_wordSpace_orientation_asQuaternion = Quaterniond().FromTwoVectors(
    //        NodeToRotate_id_worldSpace_position - pivot_worldSpace_position,
    //        Target - pivot_worldSpace_position ) * Quaterniond( pivot_wordSpace_orientation );
    pivot_local_orientation = Quaterniond().FromTwoVectors(
            NodeToRotateInPivotCoordinatesSpace,
            TargetInPivotCoordinatesSpace) * pivot_local_orientation;


    //EnforceHinge
    //Eigen::Matrix3d parent_wordSpace_orientation = CalcBodyWorldOrientation (model,  Qs, parent_id,true);
    Quaterniond parent_local_orientation( AngleAxisd ( Qs[ parentIndex ], ParentAxes ) );


    //double angle = AngleAxisd(Quaterniond().FromTwoVectors(
    //        pivot_wordSpace_orientation_asQuaternion * PivotAxes,
    //        parent_wordSpace_orientation * PivotAxes ) * pivot_wordSpace_orientation_asQuaternion ).angle();
    pivot_local_orientation = Quaterniond().FromTwoVectors(
            pivot_local_orientation * PivotAxes,
            parent_local_orientation * PivotAxes
            ) * pivot_local_orientation;
    double angle = AngleAxisd( pivot_local_orientation ).angle();

    Qs[ pivotIndex ] = angle;
    return;

    Vector3d EE_worldSpace_position = CalcBodyToBaseCoordinates(model,  Qs, NodeToRotate_id,Vector3d(0,0,0),true);
    double ErrorBefore = (EE_worldSpace_position - Target).squaredNorm();

    double aux = Qs[ pivotIndex ];
    Qs[ pivotIndex ] = angle;

    EE_worldSpace_position = CalcBodyToBaseCoordinates(model,  Qs, NodeToRotate_id,Vector3d(0,0,0),true);
    double ErrorAfter = (EE_worldSpace_position - Target).squaredNorm();

    if(ErrorBefore<ErrorAfter)
        Qs[ pivotIndex ] = aux;

    //std::cout << "Error : " << "\n" << (EE_worldSpace_position - Target).squaredNorm()<< "\n----\n";


}



//void EnforceJointLimits()



int main()
{

    Model model;
    if (!Addons::LuaModelReadFromFile ("LUA/kuka.lua", &model, false)) {
        std::cerr << "Error loading lua file" << std::endl;
        abort();
    }

    std::ofstream outfile("animationCCD_debug.csv");

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


    //Vector3d Target = CalcBodyToBaseCoordinates(model, q_start, A4_id,Vector3d(0,0,0),true);
    Vector3d Target;
    //Target << -0.5, -0.145, 0.2;
    //Target << 0, 0.2, 0.2;
    //Target << -0.1, -0.3, 0.3;
    Target << -0.3, -0.2, 0.3;


    int nrIterationMax = 100;
    int nrCurrentIteration = 0;
    double Epsilon = 0.02;

    outfile << 0 << ", " << q_start[0] << ", " << q_start[1] << ", " << q_start[2] << ", " << q_start[3] << ", " << q_start[4] << ", " << q_start[5] << ", " << "\n";

    while(1)
    {
        nrCurrentIteration++;

        Vector3d EE_worldSpace_position = CalcBodyToBaseCoordinates(model,  q_start, A6_id,Vector3d(0,0,0),true);

        for (int i = 4; i > 0; i--) {
            ApplyRotation(
                    model,
                    index_id[ i - 0 ],
                    A6_id,
                    Target,
                    q_start,
                    i - 0,
                    index_id[ i - 2 ],
                    GetAxesFor(model, i + 1 ),
                    GetAxesFor(model, i + 0 ),
                    i - 1 );
        }

        outfile << nrCurrentIteration << ", " << q_start[0] << ", " << q_start[1] << ", " << q_start[2] << ", " << q_start[3] << ", " << q_start[4] << ", " << q_start[5] << ", " << "\n";

        //std::cout << "----\n" << EE_worldSpace_position << "----\n" << q_start << std::endl;
        //std::cout << q_start << std::endl << "----------" << std::endl;

        std::cout << "Error iteration nr: " << nrCurrentIteration << "\n" << (EE_worldSpace_position - Target).squaredNorm() << "----\n";

        if( (EE_worldSpace_position - Target).squaredNorm() < Epsilon )
            break;

        if( nrCurrentIteration > nrIterationMax )
            break;
    }

    outfile.close();


    return 0;
}