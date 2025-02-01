#ifndef SEATALK_DATAGRAM_H
#define SEATALK_DATAGRAM_H

#include <stdbool.h>
#include "datagram_define.h"

#define AUTOPILOT_DISPLAY_OFF 0x01
#define AUTOPILOT_DISPLAY_400G 0x02
#define AUTOPILOT_DISPLAY_NO_DATA 0x8
#define AUTOPILOT_DISPLAY_LARGE_XTE 0x10
#define AUTOPILOT_DISPLAY_AUTO_REL 0x80

#define AUTOPILOT_TYPE_400G 0x08
#define AUTOPILOT_TYPE_150G 0x05

typedef enum { TIMER_MODE_COUNT_UP, TIMER_MODE_COUNT_DOWN, TIMER_MODE_COUNT_UP_AND_START, TIMER_MODE_COUNT_DOWN_AND_START } TIMER_MODE;
typedef enum { COURSE_COMPUTER_FAILURE_TYPE_NONE, COURSE_COMPUTER_FAILURE_TYPE_AUTO_RELEASE_ERROR, COURSE_COMPUTER_FAILURE_TYPE_DRIVE_STOPPED } COURSE_COMPUTER_FAILURE_TYPE;
typedef enum { ST_AUTOPILOT_MODE_STANDBY = 0x0, ST_AUTOPILOT_MODE_AUTO = 0x2, ST_AUTOPILOT_MODE_VANE = 0x4, ST_AUTOPILOT_MODE_TRACK = 0x8 } ST_AUTOPILOT_MODE;
typedef enum {
	ST_AUTOPILOT_COMMAND_AUTO = 0x01,
	ST_AUTOPILOT_COMMAND_STANDBY = 0x02,
	ST_AUTOPILOT_COMMAND_TRACK = 0x03,
	ST_AUTOPILOT_COMMAND_DISP = 0x04,
	ST_AUTOPILOT_COMMAND_TURN_LEFT_1 = 0x05,
	ST_AUTOPILOT_COMMAND_TURN_LEFT_10 = 0x06,
	ST_AUTOPILOT_COMMAND_TURN_RIGHT_1 = 0x07,
	ST_AUTOPILOT_COMMAND_TURN_RIGHT_10 = 0x08,
	ST_AUTOPILOT_COMMAND_DECREASE_GAIN = 0x09,
	ST_AUTOPILOT_COMMAND_INCREASE_GAIN = 0x0a,
	ST_AUTOPILOT_COMMAND_TACK_LEFT = 0x21,
	ST_AUTOPILOT_COMMAND_TACK_RIGHT = 0x22,
	ST_AUTOPILOT_COMMAND_WIND_MODE = 0x23,
	ST_AUTOPILOT_COMMAND_TRACK_MODE = 0x28,
	ST_AUTOPILOT_COMMAND_TOGGLE_RESPONSE_LEVEL = 0x2e,
	ST_AUTOPILOT_COMMAND_RETURN_TO_COURSE = 0x41,
	ST_AUTOPILOT_COMMAND_ENTER_COMPASS_CALIBRATION_MODE = 0x42,
	ST_AUTOPILOT_COMMAND_PRESS_TRACK_LONGER = 0x43,
	ST_AUTOPILOT_COMMAND_PRESS_DISP_LONGER = 0x44,
	ST_AUTOPILOT_COMMAND_PRESS_LEFT_1_LONGER = 0x45,
	ST_AUTOPILOT_COMMAND_PRESS_LEFT_10_LONGER = 0x46,
	ST_AUTOPILOT_COMMAND_PRESS_RIGHT_1_LONGER = 0x47,
	ST_AUTOPILOT_COMMAND_PRESS_RIGHT_10_LONGER = 0x48,
	ST_AUTOPILOT_COMMAND_RETURN_TO_WIND_ANGLE = 0x63,
	ST_AUTOPILOT_COMMAND_PRESS_LEFT_10_RIGHT_10_LONGER = 0x68,
	ST_AUTOPILOT_COMMAND_ENTER_RUDDER_GAIN_MODE = 0x6e,
	ST_AUTOPILOT_COMMAND_HOLD_LEFT_1 = 0x80,
	ST_AUTOPILOT_COMMAND_HOLD_LEFT_10 = 0x81,
	ST_AUTOPILOT_COMMAND_HOLD_RIGHT_1 = 0x82,
	ST_AUTOPILOT_COMMAND_HOLD_RIGHT_10 = 0x83,
	ST_AUTOPILOT_COMMAND_RELEASE_HELD_KEY = 0x84
} ST_AUTOPILOT_COMMAND;
typedef enum { AUTOPILOT_MODE_STANDBY, AUTOPILOT_MODE_AUTO, AUTOPILOT_MODE_VANE, AUTOPILOT_MODE_TRACK } AUTOPILOT_MODE;
typedef enum { AUTOPILOT_RESPONSE_LEVEL_AUTOMATIC_DEADBAND, AUTOPILOT_RESPONSE_LEVEL_MINIMUM_DEADBAND } AUTOPILOT_RESPONSE_LEVEL;

typedef enum { TURN_DIRECTION_NONE, TURN_DIRECTION_LEFT, TURN_DIRECTION_RIGHT } TURN_DIRECTION;

typedef enum { DISTANCE_UNITS_NAUTICAL, DISTANCE_UNITS_STATUTE, DISTANCE_UNITS_METRIC } DISTANCE_UNITS;

//enum DISTANCE_UNITS get_distance_units(void);
//void set_distance_units(enum DISTANCE_UNITS distance_units);

typedef enum { ENGINE_ID_SINGLE, ENGINE_ID_PORT, ENGINE_ID_STARBOARD } ENGINE_ID;
typedef struct {
	short int rpm;
	short int pitch_percent;
} ENGINE_STATUS;

typedef enum { ANGLE_REFERENCE_TRUE, ANGLE_REFERENCE_MAGNETIC, ANGLE_REFERENCE_BOAT } ANGLE_REFERENCE;

typedef enum { LATITUDE_HEMISPHERE_NORTH, LATITUDE_HEMISPHERE_SOUTH } LATITUDE_HEMISPHERE;
typedef enum { LONGITUDE_HEMISPHERE_WEST, LONGITUDE_HEMISPHERE_EAST } LONGITUDE_HEMISPHERE;

unsigned char get_datagram_length(char second_byte);

unsigned char build_depth_below_transducer(unsigned char* datagram, short int depth_in_feet_times_10, bool display_in_metres, unsigned short int active_alarms, bool transducer_defective);
void parse_depth_below_transducer(unsigned char* datagram, short int* depth_in_feet_times_10, bool* display_in_metres, unsigned short int* active_alarms, bool* transducer_defective);
unsigned char build_engine_rpm_and_pitch(unsigned char* datagram, ENGINE_ID engine_id, short int rpm, short int  pitch_percent);
void parse_engine_rpm_and_pitch(unsigned char* datagram, ENGINE_ID* engine_id, short int* rpm, short int* pitch_percent);
unsigned char build_apparent_wind_angle(unsigned char* datagram, short int degrees_right_times_2);
void parse_apparent_wind_angle(unsigned char* datagram, short int* degrees_right_times_2);
unsigned char build_apparent_wind_speed(unsigned char* datagram, unsigned short int knots_times_10, bool display_in_metric);
void parse_apparent_wind_speed(unsigned char* datagram, unsigned short int* knots_times_10, bool* display_in_metric);
unsigned char build_water_speed(unsigned char* datagram, short int knots_times_10);
void parse_water_speed(unsigned char* datagram, short int* knots_times_10);
unsigned char build_trip_mileage(unsigned char* datagram, unsigned long int nmiles_times_100);
void parse_trip_mileage(unsigned char* datagram, unsigned long int* miles_times_100);
unsigned char build_total_mileage(unsigned char* datagram, unsigned long int nmiles_times_10);
void parse_total_mileage(unsigned char* datagram, unsigned long int* miles_times_10);
unsigned char build_water_temperature(unsigned char* datagram, char degrees_celcius, bool transducer_defective);
void parse_water_temperature(unsigned char* datagram, char* degrees_celcius, bool* transducer_defective);
unsigned char build_speed_distance_units(unsigned char* datagram, DISTANCE_UNITS distance_units);
void parse_speed_distance_units(unsigned char* datagram, DISTANCE_UNITS* distance_units);
unsigned char build_total_and_trip_mileage(unsigned char* datagram, unsigned long int total_nm_times_10, unsigned long int trip_nm_times_100);
void parse_total_and_trip_mileage(unsigned char* datagram, unsigned long int* total_mileage_times_10, unsigned long int* trip_mileage_times_100);
unsigned char build_average_water_speed(unsigned char* datagram, short int knots_1_times_100, short int knots_2_times_100, short int speed_1_from_sensor, short int speed_2_is_average, short int average_is_stopped, bool display_in_statute_miles);
void parse_average_water_speed(unsigned char* datagram, short int* knots_1_times_100, short int* knots_2_times_100, short int* speed_1_from_sensor, short int* speed_2_is_average, short int* average_is_stopped, bool* display_in_statute_miles);
unsigned char build_precise_water_temperature(unsigned char* datagram, short int degrees_celsius_times_10);
void parse_precise_water_temperature(unsigned char* datagram, short int* degrees_celsius_times_10);
unsigned char build_lamp_intensity(unsigned char* datagram, unsigned char intensity);
void parse_lamp_intensity(unsigned char* datagram, unsigned char* level);
unsigned char build_cancel_mob(unsigned char* datagram);
unsigned char build_lat_position(unsigned char* datagram, LATITUDE_HEMISPHERE hemisphere, unsigned char degrees, unsigned short int minutes_times_100);
void parse_lat_position(unsigned char* datagram, LATITUDE_HEMISPHERE* hemisphere, unsigned char* degrees, unsigned short int* minutes_times_100);
unsigned char build_lon_position(unsigned char* datagram, LONGITUDE_HEMISPHERE hemisphere, unsigned char degrees, unsigned short int minutes_times_100);
void parse_lon_position(unsigned char* datagram, LONGITUDE_HEMISPHERE* hemisphere, unsigned char* degrees, unsigned short int* minutes_times_100);
unsigned char build_speed_over_ground(unsigned char* datagram, unsigned short int knots_times_10);
void parse_speed_over_ground(unsigned char* datagram, unsigned short int* knots_times_10);
unsigned char build_course_over_ground(unsigned char* datagram, short int true_degrees);
void parse_course_over_ground(unsigned char* datagram, short int* true_degrees);
unsigned char build_gmt_time(unsigned char* datagram, unsigned char hours, unsigned char minutes, unsigned char seconds);
void parse_gmt_time(unsigned char* datagram, unsigned char* hours, unsigned char* minutes, unsigned char* seconds);
unsigned char build_date(unsigned char* datagram, unsigned short int year, unsigned char month, unsigned char day);
void parse_date(unsigned char* datagram, unsigned short int* year, unsigned char* month, unsigned char* day);
unsigned char build_satellite_info(unsigned char* datagram, unsigned char satellite_count, unsigned char horizontal_dilution_of_position);
void parse_satellite_info(unsigned char* datagram, unsigned char* satellite_count, unsigned char* horizontal_dilution_of_position);
unsigned char build_lat_lon_position(unsigned char* datagram, LATITUDE_HEMISPHERE hemisphere_latitude, unsigned char degrees_lat, unsigned short int minutes_lat_times_1000, LONGITUDE_HEMISPHERE hemisphere_longitude, unsigned char  degrees_lon, unsigned short int minutes_lon_times_1000);
void parse_lat_lon_position(unsigned char* datagram, LATITUDE_HEMISPHERE* hemisphere_latitude, unsigned char* degrees_lat, unsigned short int* minutes_lat_times_1000, LONGITUDE_HEMISPHERE* hemisphere_longitude, unsigned char* degrees_lon, unsigned short int* minutes_lon_times_1000);
unsigned char build_countdown_timer(unsigned char* datagram, unsigned char hours, unsigned char minutes, unsigned char seconds, TIMER_MODE mode);
void parse_countdown_timer(unsigned char* datagram, unsigned char* hours, unsigned char* minutes, unsigned char* seconds, TIMER_MODE* mode);
unsigned char build_wind_alarm(unsigned char* datagram, unsigned short int active_alarms);
void parse_wind_alarm(unsigned char* datagram, unsigned short int* active_alarms);
unsigned char build_alarm_acknowledgement(unsigned char* datagram, unsigned short int acknowledged_alarm);
void parse_alarm_acknowledgement(unsigned char* datagram, unsigned short int* acknowledged_alarm);
//void parse_maxview_keystroke(unsigned char *datagram, short int *key_1, short int *key_2, short int *held_longer);
//void accept_maxview_keystroke(unsigned char *datagram);
unsigned char build_target_waypoint_name(unsigned char* datagram, char char1, char char2, char char3, char char4);
int parse_target_waypoint_name(unsigned char* datagram, char* char1, char* char2, char* char3, char* char_4);
unsigned char build_course_computer_failure(unsigned char* datagram, COURSE_COMPUTER_FAILURE_TYPE failure_type);
void parse_course_computer_failure(unsigned char* datagram, COURSE_COMPUTER_FAILURE_TYPE* failure_type);
unsigned char build_autopilot_status(unsigned char* datagram, short int compass_heading, TURN_DIRECTION turning_direction, short int target_heading, AUTOPILOT_MODE mode, short int rudder_position, unsigned short int alarms, unsigned char display_flags);
void parse_autopilot_status(unsigned char* datagram, short int* compass_heading, TURN_DIRECTION* turning_direction, short int* target_heading, AUTOPILOT_MODE* mode, short int* rudder_position, unsigned short int* alarms, unsigned char* display_flags);
unsigned char build_waypoint_navigation(unsigned char* datagram, bool cross_track_error_present, unsigned short int cross_track_error_times_100, bool waypoint_bearing_present, short int waypoint_bearing, bool bearing_is_magnetic, bool waypoint_distance_present, unsigned short int waypoint_distance_times_100, short int direction_to_steer);
int parse_waypoint_navigation(unsigned char* datagram, bool* cross_track_error_present, unsigned short int* cross_track_error_times_100, bool* waypoint_bearing_present, short int* waypoint_bearing, bool* bearing_is_magnetic, bool* waypoint_distance_present, unsigned short int* waypoint_distance_times_100, short int* direction_to_steer);
unsigned char build_autopilot_command(unsigned char* datagram, ST_AUTOPILOT_COMMAND command);
int parse_autopilot_command(unsigned char* datagram, ST_AUTOPILOT_COMMAND* command);
unsigned char build_set_autopilot_response_level(unsigned char* datagram, AUTOPILOT_RESPONSE_LEVEL response_level);
void parse_set_autopilot_response_level(unsigned char* datagram, AUTOPILOT_RESPONSE_LEVEL* response_level);
void parse_autopilot_parameter(unsigned char* datagram, unsigned char* parameter, unsigned char* min_value, unsigned char* max_value, unsigned char* value);
unsigned char build_heading(unsigned char* datagram, short int heading, bool locked_heading_active, short int locked_heading);
void parse_heading(unsigned char* datagram, short int* heading, bool* locked_heading_active, short int* locked_heading);
unsigned char build_set_rudder_gain(unsigned char* datagram, unsigned char rudder_gain);
void parse_set_rudder_gain(unsigned char* datagram, unsigned char* rudder_gain);
//void parse_set_autopilot_parameter(unsigned char *datagram, short int *parameter, short int *value);
//void parse_enter_autopilot_setup(unsigned char *datagram);
unsigned char build_compass_variation(unsigned char* datagram, short int degrees);
void parse_compass_variation(unsigned char* datagram, short int* degrees);
unsigned char build_heading_and_rudder_position(unsigned char* datagram, short int heading, TURN_DIRECTION turning_direction, short int rudder_position);
void parse_heading_and_rudder_position(unsigned char* datagram, short int* heading, TURN_DIRECTION* turning_direction, short int* rudder_position);
//void parse_destination_waypoint_info(unsigned char *datagram, char *last_4, char *first_8, short int *more_records, short int *last_record);
void parse_arrival_info(unsigned char* datagram, bool* perpendicular_passed, bool* circle_entered, char* char_1, char* char_2, char* char_3, char* char_4);
unsigned char build_gps_and_dgps_fix_info(unsigned char* datagram, short int signal_quality_available, short int signal_quality, short int hdop_available, short int hdop, short int antenna_height, short int satellite_count_available, short int satellite_count, short int geoseparation, short int dgps_age_available, short int dgps_age, short int dgps_status_id_available, short int dgps_station_id);
void parse_gps_and_dgps_fix_info(unsigned char* datagram, short int* signal_quality_available, short int* signal_quality, short int* hdop_available, short int* hdop, short int* antenna_height, short int* satellite_count_available, short int* satellite_count, short int* geoseparation, short int* dgps_age_available, short int* dgps_age, short int* dgps_station_id_available, short int* dgps_station_id);

#endif