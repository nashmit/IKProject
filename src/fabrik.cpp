// Implements the FABRIK algorithm
#include <iostream>
#include <vector>
#include <numeric> // for std::accumulate()
#include <cmath> // for acos()
#include <string>
#include <array>
#include <Eigen/Geometry>

#include "point.hpp"
#include "fabrik.hpp"

using namespace Eigen;

Point3D rotatePoint(Point3D point, char axis, double angle)
{
  Matrix3d m;
  Transform<double, 3, Affine> t;

  Vector3d vec(point.getXYZ(0),
               point.getXYZ(1),
               point.getXYZ(2));
  Vector3d rotationAxis = axis == 'X'?
                          Vector3d(1, 0, 0) :
                          (axis == 'Y'?
                          Vector3d(0, 1, 0) :
                          Vector3d(0, 0, 1));

  m = AngleAxisd(angle, rotationAxis);
  t = m;
  auto vec2 = t.linear() * vec;

  std::cout << "Rotation from " << point << " by " << angle*180/M_PI << "° around axis "
  << axis << " gives vector" << Point3D(vec2[0], vec2[1], vec2[2]) << std::endl;
  return {vec2[0], vec2[1], vec2[2]};
}

std::vector<double> convertPositionsToAngles(std::vector<Point3D> positions,
                                             std::vector<Point3D> startPositions,
                                             std::string rotationAxes)
{
  std::vector<double> result (positions.size() - 1, 0.);
  std::vector<Point3D> origStartPositions = startPositions;
  /* Calculating angles, using the law of cosines
     Link in original position:  x--------x

     Link in target postion (Calculated by FABRIK)
            x
          /
        /
      x
    We move the the target link, such its edge (which is closer to the base)
    is at the position of the egde (which is closer to the base) of the link at
    the original position to create a triangle.
          x   <-- newEdge
        /  \
      /     \
    x--------x
  */
  for (int i = 0; i < positions.size() - 1; i++)
  {
    if (i != 0)
    {
        auto test = (-1)* origStartPositions[i] + origStartPositions[i+1];
        std::cout << "\nRotation vector " << i << std::endl;
        std::cout << "Changed left point from " << startPositions[i];
        startPositions[i] = positions[i];
        std::cout << " to "<< startPositions[i] << std::endl;

        Point3D rotatedPoint = test;
        for(int j = 0; j < i; j++)
        {
          rotatedPoint = rotatePoint(rotatedPoint, rotationAxes[j], result[j]/180*M_PI);
        }
        std::cout << "Changed right vector from " << startPositions[i+1];
        startPositions[i+1] = rotatedPoint + startPositions[i];
        std::cout << " to " << startPositions[i+1] << std::endl;


    }
    /*auto newEdge = startPositions[i+1] +
                   positions[i] + (-1) * startPositions[i];
    auto a = euclideanDistance(positions[i], newEdge);
    auto b = euclideanDistance(positions[i], positions[i+1]);
    auto c = euclideanDistance(newEdge, positions[i+1] );*/

    // For refactoring: a and b are the same length!
    auto a = euclideanDistance(startPositions[i], startPositions[i+1]);
    auto b = euclideanDistance(positions[i], positions[i+1]);
    auto c = euclideanDistance(startPositions[i+1], positions[i+1]);

    std::cout << "a = " << a << ", b = " << b << ", c = " << c << std::endl;
    auto x = (pow(a, 2) + pow(b, 2) - pow(c, 2))/(2*a*b);
    auto angle = acos(x) * 180 /M_PI;
    std:: cout << "x = " << x << ", => angle: " << angle << std::endl;
    result[i] = angle;
  }
  std::cout << "Original Positions:" << std::endl;
  for(auto position : origStartPositions)
  {
    std::cout << position << std::endl;
  }
  return result;
}

std::vector<double> simpleVersion(std::vector<Point3D> & positions,
     Point3D target, double epsilon, std::string rotationAxes)
{
  std::vector<Point3D> startPositions = positions;
  // get the link lenghts
  std::vector<double> lengths (positions.size() - 1, 0.);
  for(int i = 0; i < positions.size() - 1; i++)
  {
    lengths[i] = euclideanDistance(positions[i], positions[i+1]);
  }
  // check if target is within reach
  auto distanceRootTarget = euclideanDistance(positions[0], target);
  if (distanceRootTarget > std::accumulate(lengths.begin(), lengths.end(), 0.))
  {
    // Target is not within reach
    std::cout << "Target is not within reach!" << std::endl;
    for(int i = 0; i < positions.size() - 1; i++)
    {
      // Get the distance between target and joint position
      auto distanceJointTarget = euclideanDistance(positions[i], target);
      auto lambda = lengths[i] / distanceJointTarget;
      // The new joint positions
      positions[i+1] = (1-lambda)*positions[i] + lambda*target;
    }

  } else {
    // target is within reach. We need to remember the root position, i.e. the position
    // that the system is fixed in, for step 2).
    auto root = positions[0];

    // stop when EE is close enough to the target position
    while(euclideanDistance(positions.back(), target) > epsilon)
    {
      // 1) FORWARD REACHING
      // set the EE position as target
      positions[positions.size() - 1] = target;
      for (int i = positions.size() - 2; i >= 0; i--)
      {
        // Calculate the distance between the new position i + 1 and the current position i...
        auto distancePP = euclideanDistance(positions[i+1], positions[i]);
        auto lambda = lengths[i] / distancePP;
        // ... to get the new position for point i!
        positions[i] = (1 - lambda) * positions[i+1] + lambda * positions[i];
      }

      // 2) BACKWARD REACHING
      // move root back to its previous position
      positions[0] = root;
      for(int i = 0; i < positions.size() - 1; i++)
      {
        // Same algorithm as in Step 1), but we go from root to EE
        auto distancePP = euclideanDistance(positions[i+1], positions[i]);
        auto lambda = lengths[i] / distancePP;
        positions[i+1] = (1 - lambda) * positions[i] + lambda * positions[i+1];
      }
    }
    std::cout << "Target within reach, result:" << std::endl;
    for (auto position: positions)
    {
      std::cout << position << std::endl;
    }
  }
  return convertPositionsToAngles(positions, startPositions, rotationAxes);
}
