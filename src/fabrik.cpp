// Implements the FABRIK algorithm
#include <iostream>
#include <vector>
#include <numeric> // for std::accumulate()
#include <cmath> // for atan2()
#include <string>
#include <array>

#include "point.hpp"
#include "fabrik.hpp"

std::vector<double> convertPositionsToAngles(std::vector<Point3D> positions,
                                             std::vector<Point3D> startPositions,
                                             int rotationAxis)
{
  std::vector<double> result (positions.size(), 0.);
  std::array<int, 2> coord;

  switch (rotationAxis)
  {
    case(0): // rotation around x-axis
      coord = {2, 1};
      break;
    case(1): // rotation around y-axis
      coord = {2, 0};
      break;
    case(2): // rotation around z-axis
      coord = {1, 0};
      break;
    default:
      std::cout << "Invalid rotation axis (use 0 for x-axis, 1 y-axis, 2 for z-axis)" << std::endl;
      return std::vector<double>(0);
  }

  for (int i = 0; i < positions.size(); i++)
  {
    auto angle = atan2(positions[i].getCoordinate(coord[0]) -
                       startPositions[i].getCoordinate(coord[0]),
                       positions[i].getCoordinate(coord[1]) -
                       startPositions[i].getCoordinate(coord[1])) * 180 /M_PI;
    result[i] = angle;
  }
  return result;
}

std::vector<double> simpleVersion(std::vector<Point3D> & positions, Point3D target, double epsilon)
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
  }
  return convertPositionsToAngles(positions, startPositions, 0);
}
