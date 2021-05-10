#include "ccd.h"

void ccd::Add(Node node)
{
    Nodes.push_back( node );
}

Quaterniond ccd::RotateNodeAroundNodeTo(Node pivot, Node NoteToRotate, Node Direction)
{
    Vector3d V1 = ( NoteToRotate.transform.Translation - pivot.transform.Translation );
    Vector3d V2 = ( Direction.transform.Translation - pivot.transform.Translation );

    Quaterniond rotation;
    return rotation.FromTwoVectors(V1, V2) * pivot.transform.Rotation;

}

Quaterniond ccd::EnforcingHinge(Node parent, Node pivot)
{
    Quaterniond  rotation;
    return  rotation.FromTwoVectors(
            pivot.transform.Rotation * pivot.Axes,
            parent.transform.Rotation * pivot.Axes ) * pivot.transform.Rotation;
}

bool ccd::Iterate( double Epsilon, int iterationNumber )
{
    int nr = 0;

    while(1) {

        nr++;

        for (int i = Nodes.size() - 2; i >= 1; i--) {

            Nodes[i].transform.Rotation = RotateNodeAroundNodeTo(Nodes[i], Nodes[i + 1], Target);
            Nodes[i].transform.Rotation = EnforcingHinge(Nodes[i - 1], Nodes[i]);
        }

        if(  (Nodes.back().transform.Translation - Target.transform.Translation).squaredNorm() < Epsilon )
            return true;

        if(nr>iterationNumber)
            return false;
    }
}

Frame ccd::UpdateGlobalCoordinates( int nodeIndex )
{
    Frame localToGlobal; // identity

    for( int i = 0 ; i <= nodeIndex ; i++ )
    {
        localToGlobal = Nodes[ i ].transform.ApplyTransform( localToGlobal );
    }

    return localToGlobal;
}

VectorXd ccd::GetQs()
{
    // don't forget to add an extra node for root since the root doesn't rotate in the algorithm!
    VectorXd Qs( Nodes.size() - 2 );

    for( int  i = 0; i < Qs.size() ; i++ )
    {
        AngleAxisd rotation( Nodes[ i ].transform.Rotation );
        Qs[i] = rotation.angle();
    }

    return Qs;
}

void ccd::SetTarget(Node _Target)
{
    Target = _Target;
}