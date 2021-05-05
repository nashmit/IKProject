#ifndef IKPROJECT_TRIHEDRAL_H
#define IKPROJECT_TRIHEDRAL_H

#include "config.h"
#include <iostream>

class Trihedral {

protected:

    //AxesX
    Vector4d Forward;

    //AxesY
    Vector4d Left;

    //AxesZ = AxesX X AxesY
    Vector4d Up;

public:

    Trihedral();

    Trihedral( const Vector4d& X, const Vector4d& Y, const Vector4d& Z);

    Vector4d GetX_Vector();
    Vector4d GetY_Vector();
    Vector4d GetZ_Vector();

    friend Trihedral operator* ( Matrix4x4d transform, const Trihedral& In );
    friend std::ostream& operator<<(std::ostream& out, const Trihedral& o);
};


#endif //IKPROJECT_TRIHEDRAL_H
