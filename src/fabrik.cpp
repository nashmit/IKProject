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

/*
   BUG: Angles > 180° aren't handlend correctly (probably a logic error in the 
   calculateAnglesGivenPositions). Not fixed yet, because max. KUKA angle
   is 170°.
*/
using namespace Eigen;

/*
 * Rotates the displacement vector from the origin (P-O) of a
 * given point P by an angle (in radians) around a given axis
 * (= unit vector of one of the three coordinate system axes,
 *  specify with 'X', 'Y', or 'Z'). Uses the Eigen library.
*/
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
  return {vec2[0], vec2[1], vec2[2]};
}

/*
 * For all links, this function calculates the angles (in degrees) between link i
 * in the initial configuration of a system (specified by a vector of points that
 * contains the position of the start and end point of all links) and link i in
 * the desired end position.
 * rotationAxes is a string which specifies the rotation axes ('X', 'Y', or
 * 'Z') for each link, e.g. XXYZ.
 *
*/
std::vector<double>
calculateAnglesGivenPositions(std::vector<Point3D> startPositions,
                              std::vector<Point3D> endPositions,
                              std::string rotationAxes)
{
  std::vector<double> result (endPositions.size() - 1, 0.);
  std::vector<Point3D> initialStartPositions = startPositions;

  for (int i = 0; i < endPositions.size() - 1; i++)
  {
    /*
     * In the FABRIK algorithm the links are rotated one by one (first link 0,
     * then link 1, ...). When link i is rotated by a certain angle, all sub-
     * sequent links i+n get rotated by that angle as well
     * (assuming a chain robot).
     * We simulate this in the following if-branch.
     */
    if (i != 0)
    {
        // Move the start point of the link to the end point of the
        // previous link (which is already at its end position).
        startPositions[i] = endPositions[i];

        // Apply all rotations of the previous links to link i.
        Point3D rotatedPoint = (-1)* initialStartPositions[i] +
                                     initialStartPositions[i+1];
        for(int j = 0; j < i; j++)
        {
          rotatedPoint = rotatePoint(rotatedPoint, rotationAxes[j],
                                     result[j]/180*M_PI);
        }
        startPositions[i+1] = rotatedPoint + startPositions[i];
    }

    // Use the cosine theorem, to calculate the angle (in deg) that we have to
    // rotate link i by in order to have it reach the end position.
    auto a = euclideanDistance(startPositions[i], startPositions[i+1]);
    auto b = euclideanDistance(endPositions[i],endPositions[i+1]);
    auto c = euclideanDistance(startPositions[i+1],endPositions[i+1]);
    auto x = (pow(a, 2) + pow(b, 2) - pow(c, 2))/(2*a*b);
    auto angle = acos(x) * 180 /M_PI;

    if (std::isnan(angle)) angle = 0;
    result[i] = angle;
  }
  return result;
}

/*
 * Implements the FABRIK algorithm for a single end-effector and does not apply
 * apply any constraints for the joints.
*/
std::vector<double> simpleVersion(std::vector<Point3D> & positions,
    const std::vector<Point3D> & startPositions, Point3D target,
     double epsilon, std::string rotationAxes)
{
  //std::vector<Point3D> startPositions = positions;
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
    std::cout << "Target " << target << " is not within reach!" << std::endl;
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
    std::cout << "Target " << target << " is within reach!\nResult:" << std::endl;
    for (auto position: positions)
    {
      std::cout << position << std::endl;
    }
  }
  return calculateAnglesGivenPositions(startPositions, positions, rotationAxes);
}
