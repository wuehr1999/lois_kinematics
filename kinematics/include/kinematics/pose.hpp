#ifndef POSE_HPP
#define POSE_HPP

#include <string>
#include <math.h>

class Pose
{
public:

 Pose();
 double x, y, z;
 double yaw, pitch, roll;

 std::string str();

 Pose operator+(const Pose& p);
 Pose& operator+=(const Pose& p);

 double dist2D(const Pose& p);
 double angle(const Pose& p);
 double dX(const Pose& p);
 double dY(const Pose& p);
 void rotatePitch(double r);
 void rotateRoll(double r);
 void rotateYaw(double r);
};
#endif
