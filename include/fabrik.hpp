#ifndef ROBOTICS_PRACTICAL_FABRIK_IK_HPP_
#define ROBOTICS_PRACTICAL_FABRIK_IK_HPP_

#include <vector>
#include "axisLimit.hpp"

std::vector<double> simpleVersion(std::vector<Point3D> & positions,
   const std::vector<Point3D> & startPositions, Point3D target,
   double epsilon, std::string rotationAxes);

std::vector<double> constraintVersion(std::vector<Point3D> & positions,
   const std::vector<Point3D> & startPositions, Point3D target,
   double epsilon, std::string rotationAxes, const std::vector<AxisLimit>& limits);

#endif // ROBOTICS_PRACTICAL_FABRIK_IK_HPP_
