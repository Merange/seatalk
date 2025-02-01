#ifndef BOAT_STATUS_H
#define BOAT_STATUS_H

#include "seatalk_datagram.h"

#define TIME_TO_LIVE 5
#define HEADING_SIZE 4
#define FLAG_SIZE 4
#define MODE_SIZE 10
#define ANGLE_SIZE 5

void initialize_status(void);

int get_active_alarms(unsigned short int *active_alarms);
void set_active_alarms(unsigned short int alarms);

int get_depth_below_transducer_in_feet_times_10(short int *feet_time_10); // depth in 0.1 foot units
void set_depth_below_transducer_in_feet_times_10(short int feet_times_10);

int get_water_temperature_in_degrees_celsius_times_10(short int* celsius_times_10); // temp in 0.1° celcius units
void set_water_temperature_in_degrees_celsius_times_10(short int celsius_times_10);

int get_engine_status(ENGINE_ID engine_id, ENGINE_STATUS *engine_status);
void set_engine_rpm(ENGINE_ID engine_id, short int rpm);
void set_engine_prop_pitch_percent(ENGINE_ID engine_id, short int prop_pitch_percent);

int get_apparent_wind_angle(short int *degrees);
void set_apparent_wind_angle(short int degrees_right);
int get_apparent_wind_speed_in_knots_times_10(unsigned short int *knots_times_10);
void set_apparent_wind_speed_in_knots_times_10(unsigned short int knots_times_10);

int get_heading(short int *degrees);
void set_heading(short int degrees);
//int get_heading_reference(ANGLE_REFERENCE *referebce);
//void set_heading_reference(ANGLE_REFERENCE reference);
int get_turn_direction(TURN_DIRECTION *direction);
void set_turn_direction(TURN_DIRECTION direction);
int get_water_speed_in_knots_times_100(unsigned short int *knots_times_100);
void set_water_speed_in_knots_times_100(unsigned short int knots_times_100);
int get_average_water_speed_in_knots_times_100(unsigned short int *knots_times_100);
void set_average_water_speed_in_knots_times_100(unsigned short int knots_times_100);
int get_rudder_position_in_degrees_right(short int*degrees_right);
void set_rudder_position_in_degrees_right(short int degrees_right);

int get_course_over_ground(short int *degrees);
void set_course_over_ground(short int degrees);
int get_course_over_ground_reference(ANGLE_REFERENCE *reference);
void set_course_over_ground_reference(ANGLE_REFERENCE reference);
int get_speed_over_ground_in_knots_times_100(unsigned short int *knots_times_100);
void set_speed_over_ground_in_knots_times_100(unsigned short int knots_times_100);
int get_trip_mileage_in_nautical_miles_times_100(unsigned long int *nautical_miles_times_100);
void set_trip_mileage_in_nautical_miles_times_100(unsigned long int nautical_miles_times_100);
int get_total_mileage_in_nautical_miles_times_10(unsigned long int *nautical_miles_times_10);
void set_total_mileage_in_nautical_miles_times_10(unsigned long int nautical_miles_times_10);

//int get_lamp_intensity(int *intensity);
//void set_lamp_intensity(int intensity);

typedef struct {
  LATITUDE_HEMISPHERE hemisphere_latitude;
  int degrees_latitude;
  int minutes_latitude_times_1000;
  LONGITUDE_HEMISPHERE hemisphere_longitude;
  int degrees_longitude;
  int minutes_longitude_times_1000;
} POSITION;

int get_position(POSITION *position);
void set_position_latitude(LATITUDE_HEMISPHERE hemisphere, unsigned char degrees, unsigned short int minutes_times_1000);
void set_position_longitude(LONGITUDE_HEMISPHERE hemisphere, unsigned char degrees, unsigned short int minutes_times_1000);
int get_compass_variation_in_degrees_west(short int*degrees_west);
void set_compass_variation_in_degrees_west(short int degrees_west);

typedef struct {
  int signal_quality_valid;
  int signal_quality;
  int position_error_valid;
  int position_error;
  int antenna_height;
  int satellite_count_valid;
  int satellite_count;
  int geoseparation;
  int dgps_age_valid;
  int dgps_age;
  int dgps_station_id_valid;
  int dgps_station_id;
} GPS_FIX_QUALITY;

int get_gps_fix_quality(GPS_FIX_QUALITY *fix_quality);
void set_gps_fix_signal_quality(short int signal_quality);
void set_gps_fix_position_error(short int  position_error);
void set_gps_fix_antenna_height(short int  antenna_height);
void set_gps_fix_satellite_count(short int  satellite_count, short int  geoseparation);
void set_gps_fix_dgps_age(short int  dgps_age);
void set_gps_fix_dgps_station_id(short int  dgps_station_id);

typedef struct {
  int waypoint_name_valid;
  char waypoint_name_last_4[5];
  int waypoint_position_valid;
  POSITION waypoint_position;
  int waypoint_bearing_and_range_valid;
  int waypoint_bearing;
  ANGLE_REFERENCE waypoint_bearing_reference;
  int waypoint_range_in_nautical_miles_times_100;
  int cross_track_error_valid;
  int cross_track_error_in_nautical_miles_times_100;
} NAVIGATION_STATUS;

int get_navigation(NAVIGATION_STATUS *navigation_status);
void set_navigation_waypoint_name(char *name);
void set_navigation_waypoint_position_latitude(LATITUDE_HEMISPHERE hemisphere, unsigned char degrees, unsigned short int minutes_times_1000);
void set_navigation_waypoint_position_longitude(LONGITUDE_HEMISPHERE hemisphere, unsigned char degrees, unsigned short int minutes_times_1000);
void set_navigation_waypoint_bearing_and_range_in_nautical_miles_times_100(short int degrees, unsigned long int nautical_miles_times_100);
void set_navigation_waypoint_bearing_reference(ANGLE_REFERENCE angle_reference);
void set_navigation_cross_track_error_in_nautical_miles_times_100(unsigned long int nautical_miles_times_100);

typedef struct {
  unsigned short int year;
  unsigned char month;
  unsigned char day;
  unsigned char hour;
  unsigned char minute;
  unsigned char second;
} DATE_AND_TIME;

int get_gmt_date_and_time(DATE_AND_TIME *gmt_date_and_time);
void set_gmt_date(unsigned short int year, unsigned char month, unsigned char day);
void set_gmt_time(unsigned char hour, unsigned char minute, unsigned char second);

typedef struct {
  short int target_heading;
  AUTOPILOT_MODE mode;
  AUTOPILOT_RESPONSE_LEVEL response_level;
  int rudder_gain;
} AUTOPILOT_STATUS;

int get_autopilot(AUTOPILOT_STATUS *new_ap_status);
void set_autopilot_target_heading(short int degrees);
void set_autopilot_mode(AUTOPILOT_MODE mode);
//void set_autopilot_response_level(AUTOPILOT_RESPONSE_LEVEL response_level);
//void set_autopilot_rudder_gain(int rudder_gain);

#endif