#include <iostream>

#include "point.hpp"

// add default constructor and subtraction!
Point3D operator+(Point3D lhs, const Point3D& rhs)
{
  lhs += rhs;
  return lhs;
}

Point3D operator*(Point3D lhs, double rhs)
{
  return {lhs.x*rhs, lhs.y*rhs, lhs.z*rhs};
}
Point3D operator*(double lhs, Point3D rhs)
{
  return rhs*lhs;
}
std::ostream& operator<<(std::ostream& os, const Point3D & point)
{
  os << "(" << point.x << ", " << point.y << ", " << point.z << ")";
  return os;
}

double euclideanDistance(Point3D p1, Point3D p2)
{
  return sqrt(pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2) + pow(p1.z-p2.z, 2));
}
