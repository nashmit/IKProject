#ifndef ROBOTICS_PRACTICAL_PLANE_HPP_
#define ROBOTICS_PRACTICAL_PLANE_HPP_
#include <Eigen/Geometry>

#include "point.hpp"

using namespace Eigen;

class Plane{
private:
  Vector3d origin;
  Vector3d vec1;
  Vector3d vec2;
  Vector3d normal;
public:
  Plane(Vector3d _origin, Vector3d _vec1, Vector3d _vec2): origin(_origin),
                                                   vec1(_vec1), vec2(_vec2)
  {
    normal = vec1.cross(vec2);
    std::cout << "Plane's normal vector: " << normal << std::endl;
  }

  Vector3d projectPointOnPlane(Vector3d point)
  {
    return point - (normal.dot(point) - normal.dot(origin)) * normal;
  }
};

#endif // ROBOTICS_PRACTICAL_PLANE_HPP_
