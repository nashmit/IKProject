#include <vector>
#include <iostream>

#include "point.hpp"
#include "fabrik.hpp"

int main()
{
  Point3D link1 = {0.5, 1, 4};
  Point3D link2 = {5, 4, 4};
  Point3D link3 = {10, 8, 5};
  Point3D endEffector = {12, 4, 0};
  std::vector<Point3D> positions = {link1, link2, link3, endEffector};

  Point3D target = {-4, -5, -11.5};
  //Point3D target = {-100, -2000, -1000};
  std::cout << "End effector starts at position" << endEffector << std::endl;
  std::cout << "Goal is: " << target << std::endl;
  std::cout << "Algorithm start:" << std::endl;
  auto angles = simpleVersion(positions, target, 0.001);
  std::cout << "Algorithm stop. EE Position:" << positions.back() << std::endl;
  std::cout << "Angles:" << std::endl;
  for (auto angle: angles)
  {
    std::cout << angle << "°" << ",";
  }
  return 0;
}
