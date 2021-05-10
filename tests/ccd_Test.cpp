#include "../include/ccd.h"

int main()
{
    ccd ccdAlgorithm;

    //root
    ccdAlgorithm.Add( Node( ccdTransform( Vector3d().setZero(), Quaterniond().Identity() ), Vector3d().setZero() ) );

    ccdAlgorithm.Add( Node( ccdTransform( Vector3d(0,0,0.203), Quaterniond().Identity() ), Vector3d(0,0,-1) ) );
    ccdAlgorithm.Add( Node( ccdTransform( Vector3d(0.075, 0, 0.132), Quaterniond().Identity() ), Vector3d(0,1,0) ) );
    ccdAlgorithm.Add( Node( ccdTransform( Vector3d(0.365, 0, 0), Quaterniond().Identity() ), Vector3d(0,1,0) ) );
    ccdAlgorithm.Add( Node( ccdTransform( Vector3d(0.208, 0, 0.09), Quaterniond().Identity() ), Vector3d(-1,0,0) ) );
    ccdAlgorithm.Add( Node( ccdTransform( Vector3d(0.197,0,0), Quaterniond().Identity() ), Vector3d(0,1,0) ) );
    ccdAlgorithm.Add( Node( ccdTransform( Vector3d(0.08,0,0), Quaterniond().Identity() ), Vector3d(1,0,0) ) );

    ccdAlgorithm.SetTarget(
            Node(
                    ccdTransform(
                            Vector3d(0.197,0,0),
                            Quaterniond().Identity() ),
                            Vector3d(0,1,0)
                            )
                            );

    bool foundSometing = ccdAlgorithm.Iterate();

    if(!foundSometing)
    {
        std::cout << "No solution!" << std::endl;
        return 1;
    }

    std::cout << ccdAlgorithm.GetQs();

    return 0;
}