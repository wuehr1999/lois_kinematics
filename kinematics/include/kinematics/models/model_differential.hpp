#ifndef MODEL_HPP
#define MODEL_HPP

#include <kinematics/model.hpp>

class Model_Differential: public Model
{
public:

  Model_Differential(double wheelWidth_m) : Model()
  {
    this->wheelWidth_m = wheelWidth_m;
  }

  Pose calculate(double v0, double w, double t_sec)
  {
    Pose p;
    if(0.0 != w)
    {
      double deltaV = (w * wheelWidth_m) / 2.0;
      double vL = v0 + deltaV;
      double vR = v0 - deltaV;

      double R = 0;
      R = 0.5 * wheelWidth_m * (vL + vR) / (vR -vL);
      double ICCx = -R * sin(0.0);
      double ICCy = R * cos(0.0);
      p.x = cos(w * t_sec) * -ICCx - sin(w * t_sec) * -ICCy + ICCx;
      p.x *= -1;
      p.y = sin(w * t_sec) * -ICCx + cos(w * t_sec) * -ICCy + ICCy;
      p.y *= -1;
      p.yaw = w * t_sec;
    }
    else
    {
      p.x = v0 * t_sec;
      p.y = 0;
      p.yaw = 0;
    }
    return p;
  }
  
  Pose calculateByDiff(double vL, double vR, double t_sec)
  {
    std::pair<double, double> vw = vwFromDiff(vL, vR);
    return calculate(vw.first, vw.second, t_sec);
  }

  std::pair<double, double> vwFromDiff(double vL, double vR)
  {
    double w = (vR - vL) / wheelWidth_m;
    double v = (vR + vL) * 0.5;
    return std::pair<double, double>(v, w);
  }
/*  Pose calculate(double v0, double w, double t_sec, Pose last)
  {
//    printf("%f, %f, %f, %s\n", v0, w, t_sec, last.str().c_str());
    Pose p;
    if(0.0 != w)
    {
      double deltaV = (w * wheelWidth_m) / 2.0;
      double vL = v0 + deltaV;
      double vR = v0 - deltaV;

      double R = 0;
      if(vL == vR)
      {
        R = 0.5 * wheelWidth_m;
      }
      else
      {
        R = 0.5 * wheelWidth_m * (vL + vR) / (vR -vL);
      }
      double ICCx = last.x - R * sin(last.yaw);
      double ICCy = last.y + R * cos(last.yaw);
      p.x = cos(w * t_sec) * (last.x - ICCx) - sin(w * t_sec) * (last.y - ICCy) + ICCx;
      p.x *= -1;
      p.y = sin(w * t_sec) * (last.x -ICCx) + cos(w * t_sec) * (last.y - ICCy) + ICCy;
      p.y *= -1;
      p.yaw = last.yaw + w * t_sec;
    }
    else
    {
      p.x = last.x + v0 * t_sec;
      p.y = last.y + 0;
      p.yaw = last.yaw + 0;
    }
    return p;
  }
*/ 
private:
  double wheelWidth_m;
};
#endif
