
#ifndef ROBOTICS_PRACTICAL_AXIS_LIMIT_HPP_
#define ROBOTICS_PRACTICAL_AXIS_LIMIT_HPP_

/*
 *  Encapsulates the axis limits of a 1DoF hinge joint.
*/
struct AxisLimit{
  double min;
  double max;
  AxisLimit(double _min, double _max): min(_min), max(_max){}
};

#endif // ROBOTICS_PRACTICAL_AXIS_LIMIT_HPP_
