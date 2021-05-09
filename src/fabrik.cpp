// Implements the FABRIK algorithm
#include <iostream>
#include <vector>
#include <numeric> // for std::accumulate()
#include <cmath> // for acos() and fabs()
#include <string>
#include <array>
#include <Eigen/Geometry>

#include "point.hpp"
#include "fabrik.hpp"
#include "axisLimit.hpp"
#include "plane.hpp"

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
                                     result[j]/180.*M_PI);
        }
        startPositions[i+1] = rotatedPoint + startPositions[i];
    }

    // Using dot product (instead of cosine theorem), because we need to take into
    // account the sign of the rotation (otherwise angles >= 180 won't work, since
    // angles in a triangle cannot be bigger than 180°).
    // Idea taken from here:
    // https://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane
    Vector3d normalOfRotationPlane = rotationAxes[i] == 'X'?
                            Vector3d(1, 0, 0) :
                            (rotationAxes[i] == 'Y'?
                            Vector3d(0, 1, 0) :
                            Vector3d(0, 0, 1));
    auto start = startPositions[i+1] - startPositions[i];
    auto end   = endPositions[i+1]   - endPositions[i];
    Vector3d startVector(start.getXYZ(0), start.getXYZ(1), start.getXYZ(2));
    Vector3d endVector(end.getXYZ(0), end.getXYZ(1), end.getXYZ(2));
    double angle  = acos(startVector.normalized().dot(endVector.normalized()));
    auto crossProduct = startVector.cross(endVector);
    if (normalOfRotationPlane.dot(crossProduct) < 0)
    {
      angle = -angle;
    }
    if (std::isnan(angle)) angle = 0;
    result[i] = angle * 180/M_PI;
  }
  return result;
}

/*
 * Implements the FABRIK algorithm for a single end-effector while taking
 *  into account the rotational constraints of each joint
*/
std::vector<double> constraintVersion(std::vector<Point3D> & positions,
    const std::vector<Point3D> & startPositions, Point3D target,
     double epsilon, std::string rotationAxes, const std::vector<AxisLimit>& limits)
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
  auto root = positions[0];

  // Create plane using the root and target point, as well as target orientation
  // (hard-coded here)
  auto planeRT = Plane(root, target-root, Point3D(0,1,0));
  // set the EE position as Target
  positions[positions.size() - 1] = target;
  for (int i = positions.size() - 2; i >= 0; i--)
  {
    auto allowedMotion = rotationAxes[i] == 'X'?
                         Plane(positions[i], Point3D(0, 1, 0), Point3D(0, 0, 1)) :
                         (rotationAxes[i] == 'Y'?
                         Plane(positions[i], Point3D(1, 0, 0), Point3D(0, 0, 1)) :
                         Plane(positions[i], Point3D(1, 0, 0), Point3D(0, 1, 0)));
    // project the point onto planeRT
    Point3D positionProjection = planeRT.projectPointOnPlane(positions[i]);
    auto newDirection = positionProjection - target;
    auto newPosition = target + lengths[i] *
                       (1./euclideanDistance(newDirection, Point3D(0, 0, 0)))
                       * newDirection;
    auto angle = calculateAnglesGivenPositions({startPositions[i], startPositions[i+1]},
                                               {newPosition, positions[i+1]},
                                               std::string(1, rotationAxes[i]))[0];
    if (angle < 0 && angle < limits[i].min)
    {
      std::cout << "Negative angle is too small. Rotating to minimum instead" << std::endl;
      positions[i] = rotatePoint(startPositions[i+1] - startPositions[i], rotationAxes[i], limits[i].min * M_PI / 180.);
      std::cout << "Angle before: " << angle << ", minimum: " << limits[i].min <<
      ", angle after new rotation: " << calculateAnglesGivenPositions({startPositions[i], startPositions[i+1]},
                                                 {positions[i], positions[i+1]},
                                                 std::string(1, rotationAxes[i]))[0]
                                                 << ", expected: " << limits[i].min << std::endl;
    } else if (angle > 0 && angle > limits[i].max){
      std::cout << "Positive angle is too big! Rotating to maximum instead" << std::endl;
      positions[i] = rotatePoint(startPositions[i+1] - startPositions[i], rotationAxes[i], limits[i].max * M_PI / 180.);
      std::cout << "Angle before: " << angle << ", maximum: " << limits[i].max <<
      ", angle after new rotation: " << calculateAnglesGivenPositions({startPositions[i], startPositions[i+1]},
                                                 {positions[i], positions[i+1]},
                                                 std::string(1, rotationAxes[i]))[0]
                                                 << ", expected: " << limits[i].max << std::endl;
    } else {
      positions[i] = newPosition;
    }

  }
  // stop when EE is close enough to the target position
  /*while(euclideanDistance(positions.back(), target) > epsilon)
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
  }*/
  return calculateAnglesGivenPositions(startPositions, positions, rotationAxes);
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
    //while(euclideanDistance(positions.back(), target) > epsilon)
    while ((std::fabs(positions.back().getXYZ(0) - target.getXYZ(0)) > epsilon) ||
           (std::fabs(positions.back().getXYZ(1) - target.getXYZ(1)) > epsilon) ||
           (std::fabs(positions.back().getXYZ(2) - target.getXYZ(2)) > epsilon))
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
