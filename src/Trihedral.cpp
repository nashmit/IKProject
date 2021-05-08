#include "../include/IKProject/Trihedral.h"

Trihedral::Trihedral()
{
    Forward = Vector4d( 1, 0, 0, 1);
    Left = Vector4d( 0, -1, 0, 1);
    Up =Vector4d( 0, 0, 1, 1);
}

Trihedral::Trihedral( const Vector4d& X, const Vector4d& Y, const Vector4d& Z):
    Forward(X), Left(Y), Up(Z)
{}

Vector4d Trihedral::GetX_Vector()
{
    return Forward;
}

Vector4d Trihedral::GetY_Vector()
{
    return Left;
}

Vector4d Trihedral::GetZ_Vector()
{
    return Up;
}

Trihedral operator* ( Matrix4x4d Transform, const Trihedral& In )
{
    return Trihedral( Transform * In.Forward, Transform * In.Left, Transform * In.Up );
}

std::ostream& operator<< ( std::ostream& out, const Trihedral& o )
{
    out
    << "\n\nForward axes (becomes): \n" << o.Forward
    << "\n\nLeft axes (becomes): \n" << o.Left
    << "\n\nUp axes (becomes): \n" << o.Up << std::endl;

    return out;
}