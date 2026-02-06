#include "logger.h"


float findError(float measured, float actual)
{
  float r = 0.0;
  r = measured - actual;
  r = r/actual;
  return r;
}

void zeroKalman(KALMAN_DATA *kd)
{
  kd->alt_altitude = 0;
  kd->alt_pressure = 0;
  kd->alt_temp = 0;

  kd->gyr_accl_x = 0;
  kd->gyr_accl_y = 0;
  kd->gyr_accl_z = 0;
  kd->gyr_rot_x = 0;
  kd->gyr_rot_y = 0;
  kd->gyr_rot_z = 0;
  kd->gyr_temp = 0;
}

