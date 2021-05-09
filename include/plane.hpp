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

  Plane(Point3D _origin, Point3D _vec1, Point3D _vec2)
  {
    origin = Vector3d(_origin.getXYZ(0), _origin.getXYZ(1), _origin.getXYZ(2));
    vec1 = Vector3d(_vec1.getXYZ(0), _vec1.getXYZ(1), _vec1.getXYZ(2));
    vec2 = Vector3d(_vec2.getXYZ(0), _vec2.getXYZ(1), _vec2.getXYZ(2));
    normal = vec1.cross(vec2);
  }

  Vector3d projectPointOnPlane(Vector3d point)
  {
    return point - (normal.dot(point) - normal.dot(origin)) * normal;
  }
  Point3D projectPointOnPlane(Point3D point)
  {
    auto pointAsVec = Vector3d(point.getXYZ(0), point.getXYZ(1), point.getXYZ(2));
    auto projection = projectPointOnPlane(pointAsVec);
    return {projection[0], projection[1], projection[2]};
  }

};

#endif // ROBOTICS_PRACTICAL_PLANE_HPP_
