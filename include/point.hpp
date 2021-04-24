#ifndef ROBOTICS_PRACTICAL_POINT3D_HPP_
#define ROBOTICS_PRACTICAL_POINT3D_HPP_

#include <cmath> // for sqrt() and pow()
#include <vector>
#include <exception> // for std::terminate

class Point3D
{
private:
  double x,y,z;
public:
  Point3D(double _x, double _y, double _z):x(_x), y(_y), z(_z){}

  double getXYZ(int coord)
  {
    switch(coord)
    {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
    }
    std::terminate();
  }

  // Using default copy and move constructors and assignment operators
  Point3D(Point3D const&) = default;
  Point3D& operator=(const Point3D&) = default;
  Point3D (Point3D&&) = default;
  Point3D& operator=(Point3D&&) = default;

  Point3D& operator+=(const Point3D& rhs)
  {
    this->x += rhs.x;
    this->y += rhs.y;
    this->z += rhs.z;
    return *this;
  }
  friend Point3D operator+(Point3D lhs, const Point3D& rhs);
  friend Point3D operator*(Point3D lhs, double rhs);
  friend Point3D operator*(double lhs, Point3D);
  friend std::ostream& operator<<(std::ostream& os, const Point3D & point);

  friend double euclideanDistance(Point3D p1, Point3D p2);

};

#endif // ROBOTICS_PRACTICAL_POINT3D_HPP_
