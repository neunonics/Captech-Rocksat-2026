#include <TinyGPS++.h>

enum STATE {STATE_NONE, STATE_ARMED, STATE_READY, STATE_LAUNCHED, STATE_LANDED, STATE_LOGGING};

struct TRIGGER {
    enum TRIGGER_NAME {TRIG_A=0, TRIG_B, TRIG_C, TRIG_D};
    enum TRIGGER_TYPE {ABOVE, BELOW};

    double altitude;
    TRIGGER_TYPE trigger;
    int ignition_cord;
    TRIGGER_NAME name;
    int pin;
};

struct KALMAN_DATA {
  double stm_milis;
  double alt_altitude;
  double alt_temp;
  double alt_pressure;
  double gps_altitude;
  double gps_speed;
  double gps_lng;
  double gps_lat;

  
  uint8_t gps_sec;
  uint8_t gps_min;
  uint8_t gps_hou;

  int gps_satilites;

  uint16_t gps_year;
  uint8_t gps_day;
  uint8_t gps_month;

  TinyGPSDate gps_date;

  double gyr_temp;
  double gyr_accl_x;
  double gyr_accl_y;
  double gyr_accl_z;
  double gyr_rot_x;
  double gyr_rot_y;
  double gyr_rot_z;

  double calc_velocity;
  double calc_accl;
};

float findError(float measured, float actual);
void zeroKalman(KALMAN_DATA *kd);
