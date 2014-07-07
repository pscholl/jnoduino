//=====================================================================================================
// AHRS.h
// S.O.H. Madgwick
// 25th August 2010
//=====================================================================================================
//
// See AHRS.c file for description.
// 
//=====================================================================================================
#ifndef AHRS_h
#define AHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

//---------------------------------------------------------------------------------------------------
// Function declaration

typedef struct {
  unsigned long timestamp;
  float q0, q1, q2, q3, exInt, eyInt, ezInt;	// quaternion elements representing the estimated orientation
} orientation_t;

orientation_t* AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float halfT);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
