#include "math.h"
//#include <stdafx.h>
#include <Kinematics.h>
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
int Kinematics::delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
  //Direkte Kinematik (nicht auf Funktionalitaet ueberprueft)
     float t = (f-e)*tan30/2;
     float dtr = -pi/(float)180.0;

     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;

     float y1 = -(t + rf*cos(theta1));
     float z1 = -rf*sin(theta1);

     float y2 = (t + rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -rf*sin(theta2);

     float y3 = (t + rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -rf*sin(theta3);

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

     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;
     return 0;
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
int Kinematics::delta_calcParallelAngle(float x0, float y0, float z0, float theta1, float theta2, float theta3, float &phi1, float &phi2, float &phi3) {
  //Berechnung der Winkel innerhalb der Parallelogramme der Unterarme des Roboters
  //Erfolgt ueber |u x v|=|u|*|v|*sin(theta) mit u: Vektor der den Unterarm beschreibt, v: Vektor der senkrecht auf a steht und aus Oberarm raus zeigt (Laenge 1)
  int status = delta_calcAngleU1U2(x0, y0, z0, theta1, phi1, 0);
  if (status == 0) status = delta_calcAngleU1U2(x0, y0, z0, theta2, phi2, 2 * M_PI / 3);  // rotate coords to +120 deg
  if (status == 0) status = delta_calcAngleU1U2(x0, y0, z0, theta3, phi3, -2 * M_PI / 3);   // rotate coords to -120 deg
  return status;

}

int Kinematics::delta_calcAngleU1U2(float x0, float y0, float z0, float theta, float &phi, float rot) {
  //Hilfsfunktion um Winkel zu berechnen
  float u1 = x0+sin(rot)*(e-f+rf*cos(theta));
  float u2 = y0 - cos(rot)*(e - f - rf*cos(theta));
  float u3 = z0 + rf*sin(theta);
  float v1 = cos(rot);
  float v2 = sin(rot);
  float v3 = 0;
  float crossP1 = u2*v3 - u3*v2;
  float crossP2 = u3*v1 - u1*v3;
  float crossP3 = u1*v2 - u2*v1;
  phi = M_PI/2 - asin(sqrt(crossP1*crossP1 + crossP2*crossP2 + crossP3*crossP3) / (sqrt(u1*u1+u2*u2+u3*u3)*sqrt(v1*v1 + v2*v2 + v3*v3)));
  return 0;
}

int Kinematics::delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3){
     theta1 = theta2 = theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, theta1);
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
     if(theta1< -100*M_PI/180 ||theta2< -100*M_PI/180 ||theta3< -100*M_PI/180){
       int ee=0;
     }
     return status;
}
int Kinematics::delta_Jacobian(float* phi, float* psi, float* theta,float* Jac[]){

      float t1 = cos(psi[2]);
      float t2 = phi[2] + theta[2];
      float t3 = cos(t2);
      float t4 = t1 * t3;
      float t5 = sin(-2*M_PI/3);
      float t6 = phi[1] + theta[1];
      float t7 = sin(t6);
      float t9 = cos(psi[1]);
      float t12 = t1 * t9;
      float t13 = cos(t6);
      float t14 = sin(2*M_PI/3);
      float t15 = t13 * t14;
      float t16 = sin(t2);
      float t19 = sin(psi[1]);
      float t21 = cos(2*M_PI/3);
      float t22 = t21 * t16;
      float t24 = cos(-2*M_PI/3);
      float t25 = sin(psi[2]);
      float t26 = t24 * t25;
      float t27 = t7 * t9;
      float t30 = theta[0] + phi[0];
      float t31 = cos(t30);
      float t34 = cos(psi[0]);
      float t35 = sin(0);
      float t36 = t34 * t35;
      float t37 = t36 * t27;
      float t39 = t24 * t34;
      float t41 = sin(t30);
      float t42 = t9 * t41;
      float t43 = t42 * t15;
      float t47 = cos(0);
      float t48 = t34 * t47;
      float t49 = t48 * t27;
      float t51 = t5 * t34;
      float t53 = t13 * t21;
      float t54 = t42 * t53;
      float t56 = t1 * t31;
      float t58 = t9 * t13;
      float t59 = t58 * t22;
      float t63 = t58 * t14 * t16;
      float t65 = t4 * t24;
      float t66 = t34 * t41;
      float t67 = t19 * t21;
      float t68 = t66 * t67;
      float t70 = sin(psi[0]);
      float t71 = t70 * t47;
      float t72 = t71 * t27;
      float t74 = t4 * t5;
      float t75 = t14 * t19;
      float t76 = t66 * t75;
      float t78 = t35 * t70;
      float t79 = t78 * t27;
      float t81 = t56 * t34;
      float t82 = t35 * t14;
      float t83 = t19 * t16;
      float t86 = t47 * t19;
      float t89 = t4 * t24 * t31 * t37 - t4 * t5 * t31 * t49 + t81 * t86 * t22 - t56 * t36 * t59 - t4 * t39 * t43 + t4 * t51 * t54 + t56 * t48 * t63 + t81 * t82 * t83 - t65 * t68 + t65 * t72 - t74 * t76 + t74 * t79;
      float t90 = t1 * t35;
      float t91 = t90 * t70;
      float t93 = t1 * t70;
      float t94 = t93 * t47;
      float t98 = t26 * t34;
      float t100 = t25 * t5;
      float t103 = t100 * t34;
      float t121 = t47 * t7 * t9;
      float t123 = t26 * t35 * t70 * t7 * t9 + t103 * t41 * t19 * t21 - t98 * t41 * t14 * t19 - t100 * t70 * t121 - t100 * t31 * t37 - t91 * t67 * t16 + t94 * t75 * t16 - t26 * t31 * t49 + t103 * t43 + t98 * t54 - t94 * t59 - t91 * t63;
      float t125 = 1 / (t89 + t123);
      float t128 = sin(phi[0]);
      float t129 = l_all(1) * t34 * t128;
      float t141 = sin(phi[1]);
      float t142 = l_all(1) * t9 * t141;
      float t144 = t31 * t34;
      float t148 = t34 * t9;
      float t149 = t41 * t13;
      float t155 = sin(phi[2]);
      float t156 = l_all(1) * t1 * t155;
      float t185 = t58 * t14;
      float t187 = t58 * t21;
      float t202 = t144 * t35;
      float t204 = t144 * t47;
      Jac[0][0] = -(t4 * t5 * t7 * t9 - t1 * t19 * t22 - t12 * t15 * t16 + t26 * t27) * t125 * t129;
      Jac[0][1] = (-t56 * t36 * t16 - t93 * t47 * t16 + t4 * t51 * t41 + t26 * t66) * t125 * t142;
      Jac[0][2] = (t144 * t35 * t7 * t9 - t148 * t149 * t14 - t68 + t72) * t125 * t156;
      Jac[1][0] = (t4 * t24 * t7 * t9 + t1 * t14 * t83 - t12 * t53 * t16 - t100 * t27) * t125 * t129;
      Jac[1][1] = -(-t56 * t48 * t16 + t90 * t70 * t16 + t4 * t39 * t41 - t100 * t66) * t125 * t142;
      Jac[1][2] = -(-t148 * t149 * t21 + t144 * t121 + t76 - t79) * t125 * t156;
      Jac[2][0] = -(t4 * t5 * t14 * t19 + t4 * t24 * t19 * t21 - t100 * t185 - t100 * t67 + t65 * t185 - t26 * t187 - t74 * t187 + t26 * t75) * t125 * t129;
      Jac[2][1] = (t4 * t24 * t70 * t47 + t4 * t5 * t35 * t70 - t100 * t202 - t100 * t71 + t65 * t202 - t26 * t204 - t74 * t204 + t26 * t78) * t125 * t142;
      Jac[2][2] = -(-t144 * t82 * t19 - t144 * t86 * t21 - t204 * t185 + t78 * t185 + t202 * t187 + t71 * t187 + t78 * t67 - t71 * t75) * t125 * t156;
}
