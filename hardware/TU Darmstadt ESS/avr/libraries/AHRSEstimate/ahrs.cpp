//=====================================================================================================
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Header files

#include "ahrs.h"
#include <Arduino.h>
#include <math.h>

//----------------------------------------------------------------------------------------------------
// Definitions

#define Kp 2.0f     // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f   // integral gain governs rate of convergence of gyroscope biases

//====================================================================================================
// Function
//====================================================================================================

#define DEBUG 0
#if DEBUG
# define PRINTF(...) printf(__VA_ARGS__)
#else
# define PRINTF(...)
#endif

orientation_t*
AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float halfT) {
  static orientation_t o = {1,0,0,0,0,0,0};
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;

  // auxiliary variables to reduce number of repeated operations
  float q0q0 = o.q0*o.q0;
  float q0q1 = o.q0*o.q1;
  float q0q2 = o.q0*o.q2;
  float q0q3 = o.q0*o.q3;
  float q1q1 = o.q1*o.q1;
  float q1q2 = o.q1*o.q2;
  float q1q3 = o.q1*o.q3;
  float q2q2 = o.q2*o.q2;   
  float q2q3 = o.q2*o.q3;
  float q3q3 = o.q3*o.q3;          

  // normalise the measurements
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;
  PRINTF("a: %d %d %d %d\n",(int) (ax*1000), (int) (ay*1000), (int) (az*1000), (int) norm);
  norm = sqrt(mx*mx + my*my + mz*mz);
  mx = mx / norm;
  my = my / norm;
  mz = mz / norm;
  PRINTF("m: %d %d %d %d\n",(int) (mx*1000), (int) (my*1000), (int) (mz*1000), (int) norm);

  // compute reference direction of flux
  hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;

  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
  PRINTF("e: %d %d %d %d\n",(int) (ex*1000), (int) (ey*1000), (int) (ez*1000), (int) norm);
  
  // integral error scaled integral gain
  o.exInt = o.exInt + ex*Ki;
  o.eyInt = o.eyInt + ey*Ki;
  o.ezInt = o.ezInt + ez*Ki;
  
  // adjusted gyroscope measurements
  PRINTF("g: %d %d %d %d\n",(int) gx*1000, (int) gy*1000, (int) gz*1000, (int) norm);
  gx = gx + Kp*ex + o.exInt;
  gy = gy + Kp*ey + o.eyInt;
  gz = gz + Kp*ez + o.ezInt;
  PRINTF("g: %d %d %d %d\n",(int) gx*1000, (int) gy*1000, (int) gz*1000, (int) norm);
  
  // integrate quaternion rate and normalise
  PRINTF("q: %d %d %d %d %d\n", (int)(q0*1000), (int)(q1*1000),(int)(q2*1000),(int)(q3*1000), (int) (norm*1000));
  o.q0 += (-o.q1*gx - o.q2*gy - o.q3*gz)*halfT;
  o.q1 += (o.q0*gx + o.q2*gz - o.q3*gy)*halfT;
  o.q2 += (o.q0*gy - o.q1*gz + o.q3*gx)*halfT;
  o.q3 += (o.q0*gz + o.q1*gy - o.q2*gx)*halfT;
  PRINTF("halft: %d\n",(int) (halfT*1000000));
  PRINTF("q: %d %d %d %d %d\n", (int)(q0*1000), (int)(q1*1000),(int)(q2*1000),(int)(q3*1000), (int) (norm*1000));

  // normalise quaternion
  norm = sqrt(o.q0*o.q0 + o.q1*o.q1 + o.q2*o.q2 + o.q3*o.q3);
  o.q0 /= norm;
  o.q1 /= norm;
  o.q2 /= norm;
  o.q3 /= norm;

  PRINTF("q: %d %d %d %d %d\n", (int)(q0*1000), (int)(q1*1000),(int)(q2*1000),(int)(q3*1000), (int) (norm*1000));

  return &o;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
