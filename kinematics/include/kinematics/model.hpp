#ifndef MODEL_HPP
#define MODE_HPP

#include <kinematics/pose.hpp>

class Model
{
public:
  Model(){};
  virtual ~Model(){};
 
  virtual Pose calculate(double v0, double w, double t_sec) = 0;
  //virtual Pose calculate(double v0, double w, double t_sec, Pose last = Pose()) = 0;
};
#endif
