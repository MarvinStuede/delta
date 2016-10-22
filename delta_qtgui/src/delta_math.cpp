#include "delta_math.hpp"

using namespace Eigen;
using namespace std;


void delta_math::calcCubicPoint(float te, float t_i, float p_start, float p_end, float &p_i, float &v_i){
  //Punkt auf kubischem Polynom berechnen
   float a0, a2, a3;
   a0 = p_start;
   a2 = -3*(p_start- p_end)/(te*te);
   a3 = 2*(p_start - p_end)/(te * te *te);
   p_i = a0 + a2*t_i*t_i + a3*t_i*t_i*t_i;
   v_i = 2*a2*t_i + 3*a3*t_i*t_i;
}

void delta_math::calcCircleFromThreePoints(const Vector3d &p1, const Vector3d &p2,const Vector3d &p3,Vector3d &circCenter,Vector3d &circAxis,double &circRadius){
  //Berechne Kreis anhand dreier Punkte, die auf dem Kreis liegen
  // triangle "edges"


  const Vector3d t = p2-p1;
  const Vector3d u = p3-p1;
  const Vector3d v = p3-p2;

  // triangle normal
  const Vector3d w = t.cross(u);
  const double wsl = w.squaredNorm();

  // helpers
  const double iwsl2 = 1.0 / (2.0*wsl);
  const double tt = t.adjoint()*t;
  const double uu = u.adjoint()*u;
  const double vv = v.adjoint()*v;

  // result circle
  circCenter = p1 + (u*tt*(u.adjoint()*v) - t*uu*(t.adjoint()*v)) * iwsl2;
  circRadius = sqrt(tt * uu * vv * iwsl2*0.5);
  circAxis   = w/w.norm();


}
void delta_math::rotatePointAroundCircleAxis(const Vector3d &p,std::vector<float> &q, double theta,const Vector3d &r)
{
  //Punkt p um Winkel theta um Achse r rotieren, ergibt q

   double costheta,sintheta;

   costheta = cos(theta);
   sintheta = sin(theta);

   q[0] = (costheta + (1 - costheta) * r[0] * r[0]) * p[0];
   q[0] += ((1 - costheta) * r[0] * r[1] - r[2] * sintheta) * p[1];
   q[0] += ((1 - costheta) * r[0] * r[2] + r[1] * sintheta) * p[2];

   q[1] = ((1 - costheta) * r[0] * r[1] + r[2] * sintheta) * p[0];
   q[1] += (costheta + (1 - costheta) * r[1] * r[1]) * p[1];
   q[1] += ((1 - costheta) * r[1] * r[2] - r[0] * sintheta) * p[2];

   q[2] = ((1 - costheta) * r[0] * r[2] - r[1] * sintheta) * p[0];
   q[2] += ((1 - costheta) * r[1] * r[2] + r[0] * sintheta) * p[1];
   q[2] += (costheta + (1 - costheta) * r[2] * r[2]) * p[2];

}
