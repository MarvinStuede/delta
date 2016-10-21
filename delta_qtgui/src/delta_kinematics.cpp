#include "math.h"
//#include <stdafx.h>
#include <delta_kinematics.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Kinematics::Kinematics(float e_radius, float f_radius, float forearm, float biceps){

    sqrt3 = sqrt(3.0);
    pi = 3.141592653;
    sin120 = sqrt3/2.0;
    cos120 = -0.5;
    tan60 = sqrt3;
    sin30 = 0.5;
    tan30 = 1/sqrt3;
   e = e_radius;
   f = f_radius;
   re = forearm;
   rf = biceps;

}
using namespace std;
int Kinematics::delta_calcForward(const vector<float> &theta, vector<float> &x) {
  //Direkte Kinematik
     float t = (f-e);//*tan30/2;
     float dtr = -pi/(float)180.0;

     float theta0 = theta[0] * dtr;
     float theta1 = theta[1] * dtr;
     float theta2 = theta[2] * dtr;

     float y1 = -(t + rf*cos(theta0));
     float z1 = -rf*sin(theta0);

     float y2 = (t + rf*cos(theta1))*sin30;
     float x2 = y2*tan60;
     float z2 = -rf*sin(theta1);

     float y3 = (t + rf*cos(theta2))*sin30;
     float x3 = -y3*tan60;
     float z3 = -rf*sin(theta2);

     float dnm = (y2-y1)*x3-(y3-y1)*x2;

     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;

     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);

     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // non-existing point

     x[2] = -(float)0.5*(b+sqrt(d))/a;
     x[0] = (a1*x[2] + b1)/dnm;
     x[1] = (a2*x[2] + b2)/dnm;

     return 0;
}
int Kinematics::delta_calcInverse(const vector<float> &x,  vector<float> &theta){
     theta[0] = theta[1] = theta[2] = 0;
     float x0 = x[0];
     float y0 = x[1];
     float z0 = x[2];
     int status = delta_calcAngleYZ(x0, y0, z0, theta[0]);
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta[1] );  // rotate coords to +120 deg
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta[2]);  // rotate coords to -120 deg
     return status;
}
int Kinematics::delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
  //Inverse Kinematik
  float theta1, theta2;
  theta1 = theta2 = 0;

  //a*sin(theta)-b*cos(theta)=c
  float a = 2 * z0*rf;
  float b = 2 * rf*(y0 + f - e);
  float c = z0*z0 + rf*rf + x0*x0 + (y0 + f - e)*(y0 + f - e) - re*re;

  //Fallunterscheidungen (Faelle nach Formelsammlung "Industrieroboter in der Montagetechnik" von Prof. Raatz
  //Fall 4; a*sin(theta)-b*cos(theta)=c mit a!=0, b!=0, c!=0
  if (a != 0 && b != 0 && c != 0) {
    if (c*c < (a*a + b*b)) {
      theta1 = atan2(c, sqrt(a*a + b*b - c*c)) + atan2(b, a);
      theta2 = atan2(c, -sqrt(a*a + b*b - c*c)) + atan2(b, a);
    }
    else {
      theta1 = NAN;
      theta2 = NAN;
    }
  }
  //Fall 2: cos(theta)=c/-b mit a=0,b!=0,c!=0 oder c=0
  else if (a == 0 && b != 0) {
    float a1 = c / -b;
    if ((a1*a1) < 1) {
      theta1 = atan2(sqrt(1 - a1*a1), a1);
      theta2 = -theta1;
    }
    else {
      theta1 = NAN;
      theta2 = NAN;
    }
  }
  //Fall 1: sin(theta)=c/a mit a!= 0, b=0, c!=0 oder c=0
  else if (b == 0 && a != 0 && c != 0) {
    float a1 = c / a;
    if ((a1*a1) < 1) {
      theta1 = atan2(a1, sqrt(1 - a1*a1));
      theta2 = M_PI - theta1;
    }
    else {
      theta1 = NAN;
      theta2 = NAN;
    }
  }
  //Fall 3: a*sin(theta)-b*cos(theta)=0 mit a!=0, b!=0, c=0
  else if (c == 0 && a != 0 && b != 0) {
    theta1 = atan2(b, a);
    theta2 = theta1 + M_PI;
  }
  if (isnan(theta1) || isnan(theta2)) return 1;
  else {
    //theta1 und theta2 auf Intervall [-Pi, Pi] bringen
    if (theta1>0)
      theta1 = fmod(theta1 + M_PI, 2.0*M_PI) - M_PI;
    else
      theta1 = fmod(theta1 - M_PI, 2.0*M_PI) + M_PI;
    if (theta2>0)
      theta2 = fmod(theta2 + M_PI, 2.0*M_PI) - M_PI;
    else
      theta2 = fmod(theta2 - M_PI, 2.0*M_PI) + M_PI;

    if (-M_PI / 2 <= theta2 && theta2 <= M_PI / 2) theta = theta2;
    else theta = theta1;

    return 0;
  }



}

int Kinematics::delta_calcParallelAngle(const vector<float> &x,const vector<float> &theta, vector<float> &psi,vector<float> &phi) {
  //Berechnet die Winkel der passiven Gelenke
  //phi = [phi(1) phi(2) phi(3)] :  Winkel zwischen Ober- und Unterarm
  //psi = [psi(1) psi(2) psi(3)] : Winkel innerhalb des Parallelogramms der Teilketten
  //
  vector<float> rot(3,0);
  rot[1]=2*M_PI/3;
  rot[2]=4*M_PI/3;

  for(int i=0;i<3;i++){
    float u1 = x[0] + cos(rot[i])*(e-f-rf*cos(theta[i]));
    float u2 = x[1] + sin(rot[i])*(e - f - rf*cos(theta[i]));
    float u3 = x[2] - rf*sin(theta[i]);
    //Berechnung des Vektors v, ausgehend von Koppelpunkt zwischen Ober- und
    //Unterarm in Richtung des Parallelograms
    float v1 = -sin(rot[i]);
    float v2 = cos(rot[i]);
    float v3 = 0;
    //Berechnung des Vektors w, ausgehend von Motorwelle in Richtung des
    //Oberarms (von Motor weg zeigend)
    float w1 = cos(rot[i]) * rf * cos(theta[i]);
    float w2 = sin(rot[i]) * rf * cos(theta[i]);
    float w3 = rf * sin(theta[i]);
    float crossP1 = u2*v3 - u3*v2;
    float crossP2 = u3*v1 - u1*v3;
    float crossP3 = u1*v2 - u2*v1;

    psi[i] = M_PI/2 - atan2(sqrt(crossP1*crossP1 + crossP2*crossP2 + crossP3*crossP3), u1*v1+u2*v2+u3*v3);

    crossP1 = w2*u3 - w3*u2;
    crossP2 = w3*u1 - w1*u3;
    crossP3 = w1*u2 - w2*u1;

    phi[i] = - atan2(sqrt(crossP1*crossP1 + crossP2*crossP2 + crossP3*crossP3), u1*v1+u2*v2+u3*v3);
  }

}

using namespace Eigen;
void Kinematics::delta_DirJacMatrix(const vector<float> &phi, const vector<float> &psi, const vector<float> &theta, Matrix3f & OUTPUT){
  //Direkte Jacobimatrix
  float T1 = 0;
  float T2 = 2*M_PI/3;
  float T3 = 4*M_PI/3;
  float T4 = psi[0];
  float T5 = psi[1];
  float T6 = psi[2];
  float T7 = cos(T4);
  float T8 = theta[0] + phi[0];
  float T9 = cos(T8);
  float T10 = T9 * T7;
  float T11 = cos(T6);
  float T12 = phi[2] + theta[2];
  float T13 = cos(T12);
  float T14 = T11 * T13;
  float T15 = cos(T5);
  float T16 = phi[1] + theta[1];
  float T17 = cos(T16);
  float T18 = T15 * T17;
  float T19 = sin(T4);
  float T20 = sin(T5);
  float T21 = sin(T6);
  float T22 = cos(T1);
  float T23 = cos(T2);
  float T24 = cos(T3);
  float T25 = sin(T1);
  float T26 = sin(T2);
  float T27 = sin(T3);
  float T28 = sin(T8);
  float T29 = sin(T16);
  float T30 = sin(T12);
  OUTPUT(0,0) = T10 * T22 - T19 * T25;
  OUTPUT(0,1) = T10 * T25 + T19 * T22;
  OUTPUT(0,2) = T28 * T7;
  OUTPUT(1,0) = T18 * T23 - T20 * T26;
  OUTPUT(1,1) = T18 * T26 + T20 * T23;
  OUTPUT(1,2) = T29 * T15;
  OUTPUT(2,0) = T14 * T24 - T21 * T27;
  OUTPUT(2,1) = T14 * T27 + T21 * T24;
  OUTPUT(2,2) = T30 * T11;

}
void Kinematics::delta_InvJacMatrix(const vector<float> &phi, const vector<float> &psi,Matrix3f & OUTPUT){
  //Inverse Jacobimatrix
  float T1 = rf/1000;
  float T2 = cos(psi[0]);
  float T3 = sin(phi[0]);
  float T4 = cos(psi[1]);
  float T5 = sin(phi[1]);
  float T6 = cos(psi[2]);
  float T7 = sin(phi[2]);
  OUTPUT(0,0) = T1 * T2 * T3;
  OUTPUT(1,1) = T1 * T4 * T5;
  OUTPUT(2,2) = T1 * T6 * T7;

}
int Kinematics::delta_calcJointVel(const vector<float> &x,vector<float> &dx,vector<float> &theta, vector<float> &dtheta){
//Berechnet die Gelenkgeschwindigkeiten anhand der Endeffektorgeschwindigkeit und der direkten und inversen Jacobimatrix
  //dq = B⁻¹ * A * dxe
  vector<float> phi(3,0);
  vector<float> psi(3,0);
  Vector3f dx_e(dx[0],dx[1],dx[2]);

  delta_calcParallelAngle(x, theta, psi, phi);

  Matrix3f A = Matrix3f::Zero(3,3);
  Matrix3f B = Matrix3f::Zero(3,3);
  delta_DirJacMatrix(phi,psi,theta,A);
  delta_InvJacMatrix(phi,psi,B);
  Matrix3f Inv_Jacobian = B.inverse() * A;
  Vector3f dtheta_e = Inv_Jacobian * dx_e;
  for (int k=0;k<3;k++) dtheta[k] = dtheta_e(k)/1000*180/M_PI;

}
void Kinematics::deg2rad(vector<float> &x){
  int n=x.size();
  for(int i=0;i<n;i++){
    x[i] *= M_PI/180;
  }
}
void Kinematics::rad2deg(vector<float> &x){
  int n=x.size();
  for(int i=0;i<n;i++){
    x[i] *= 180/M_PI;
  }
}
