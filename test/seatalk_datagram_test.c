#include <stdio.h>
#include <stdlib.h>
#include "test.h"
#include "../datagram_define.h"
#include "../seatalk_datagram.h"
#include "../boat_status.h"
#include "../logger.h"

#define TEST_BUILD(DATAGRAM_CODE) TEST(build_##DATAGRAM_CODE)\
  unsigned char datagram[256];

#define TEST_DATAGRAM(DATAGRAM_CODE) TEST(datagram_##DATAGRAM_CODE)
#define CHECK_INT16(NAME) assert_equal_int16(NAME ## _in, NAME ## _out, #NAME)
#define CHECK_UINT32(NAME) assert_equal_uint32(NAME ## _in, NAME ## _out, #NAME)
#define CHECK_BOOL(NAME) assert(NAME ## _in == NAME ## _out, #NAME)
#define CHECK_CHAR(NAME) assert(NAME ## _in == NAME ## _out, #NAME)

void print_datagram(char* datagram, int length) {
	for (int i = 0; i < length; i++) {
		printf("%.2x ", datagram[i]);
	}
	printf("\n");
}

void assert_depth_below_transducer(short int depth_in_feet_times_10_in, bool display_in_metres_in, unsigned short int active_alarms_in, bool transducer_defective_in) {
	unsigned char datagram[256];
	short int depth_in_feet_times_10_out;
	unsigned short int active_alarms_out;
	bool display_in_metres_out, transducer_defective_out;
	assert_equal_int16(DATAGRAM_00_LENGTH, build_depth_below_transducer(datagram, depth_in_feet_times_10_in, display_in_metres_in, active_alarms_in, transducer_defective_in), "incorrect datagram length");
	parse_depth_below_transducer(datagram, &depth_in_feet_times_10_out, &display_in_metres_out, &active_alarms_out, &transducer_defective_out);
	CHECK_INT16(depth_in_feet_times_10);
	CHECK_BOOL(display_in_metres);
	CHECK_INT16(active_alarms);
	CHECK_BOOL(transducer_defective);
}

TEST_DATAGRAM(00) // depth below transducer
for (int i = 0; i < 10000; i++) {
	assert_depth_below_transducer(i, 0, 0, 0);
}
assert_depth_below_transducer(5, 1, 0, 0);
assert_depth_below_transducer(5, 0, ALARM_SHALLOW_WATER, 0);
assert_depth_below_transducer(5, 0, ALARM_DEEP_WATER, 0);
assert_depth_below_transducer(5, 0, ALARM_ANCHOR, 0);
assert_depth_below_transducer(5, 0, ALARM_SHALLOW_WATER | ALARM_DEEP_WATER | ALARM_ANCHOR, 0); // unrealistic situation since shallow and deep can't be active together, of course
assert_depth_below_transducer(5, 0, 0, 1);
}

//TEST_DATAGRAM(01) // equipment id

void assert_engine_rpm_and_pitch(ENGINE_ID engine_id_in, short int rpm_in, short int pitch_percent_in) {
	unsigned char datagram[256];
	ENGINE_ID engine_id_out;
	short int rpm_out;
	short int pitch_percent_out;
	assert_equal_char(DATAGRAM_05_LENGTH, build_engine_rpm_and_pitch(datagram, engine_id_in, rpm_in, pitch_percent_in), "incorrect datagram length");
	parse_engine_rpm_and_pitch(datagram, &engine_id_out, &rpm_out, &pitch_percent_out);
	CHECK_CHAR(engine_id);
	CHECK_INT16(rpm);
	CHECK_INT16(pitch_percent);
}

TEST_DATAGRAM(05) // engine RPM and pitch
for (int i = 0; i < 10000; i++) {
	assert_engine_rpm_and_pitch(ENGINE_ID_SINGLE, i, 0);
}
assert_engine_rpm_and_pitch(ENGINE_ID_PORT, 1000, 0);
assert_engine_rpm_and_pitch(ENGINE_ID_STARBOARD, 1000, 0);
assert_engine_rpm_and_pitch(ENGINE_ID_SINGLE, 1000, 10);
assert_engine_rpm_and_pitch(ENGINE_ID_SINGLE, 1000, -10);
}

void assert_apparent_wind_angle(short int degrees_right_times_2_in) {
	unsigned char datagram[256];
	short int degrees_right_times_2_out;
	assert_equal_int(DATAGRAM_10_LENGTH, build_apparent_wind_angle(datagram, degrees_right_times_2_in), "incorrect datagram length");
	parse_apparent_wind_angle(datagram, &degrees_right_times_2_out);
	CHECK_INT16(degrees_right_times_2);
}

TEST_DATAGRAM(10) // apparent wind angle
assert_apparent_wind_angle(-359);
for (int i = -179; i < 180; i++) {
	for (int j = 0; j <= 1; j++) {
		assert_apparent_wind_angle((2 * i) + j);
	}
}
assert_apparent_wind_angle(360);
}

void assert_apparent_wind_speed(short int knots_times_10_in, bool display_in_metric_in) {
	unsigned char datagram[256];
	short int knots_times_10_out;
	bool display_in_metric_out;
	assert_equal_int(DATAGRAM_11_LENGTH, build_apparent_wind_speed(datagram, knots_times_10_in, display_in_metric_in), "invalid datagram_length");
	parse_apparent_wind_speed(datagram, &knots_times_10_out, &display_in_metric_out);
	CHECK_INT16(knots_times_10);
	CHECK_BOOL(display_in_metric);
}

TEST_DATAGRAM(11) // apparent wind speed
for (int i = 0; i <= 100; i++) {
	for (int j = 0; j <= 9; j++) {
		assert_apparent_wind_speed((i * 10) + j, 0);
	}
}
assert_apparent_wind_speed(100, 1);
}

void assert_water_speed(int knots_times_10_in) {
	unsigned char datagram[256];
	short int knots_times_10_out;
	assert_equal_int(DATAGRAM_20_LENGTH, build_water_speed(datagram, knots_times_10_in), "invalid datagram_length");
	parse_water_speed(datagram, &knots_times_10_out);
	CHECK_INT16(knots_times_10);
}

TEST_DATAGRAM(20) // water speed
for (int i = 0; i < 250; i++) {
	for (int j = 0; j <= 0; j++) {
		assert_water_speed((i * 10) + j);
	}
}
}

void assert_trip_mileage(int mileage_times_100_in) {
	unsigned char datagram[256];
	unsigned long int mileage_times_100_out;
	assert_equal_int(DATAGRAM_21_LENGTH, build_trip_mileage(datagram, mileage_times_100_in), "incorrect datagram length");
	parse_trip_mileage(datagram, &mileage_times_100_out);
	CHECK_UINT32(mileage_times_100);
}

TEST_DATAGRAM(21) // trip mileage
for (int i = 0; i <= 10484 / 100; i += 101) {
	for (int j = 0; j <= 99; j++) {
		assert_trip_mileage((i * 100) + j);
	}
}
}

void assert_total_mileage(unsigned long int mileage_times_10_in) {
	unsigned char datagram[256];
	unsigned long int mileage_times_10_out;
	assert_equal_int(DATAGRAM_22_LENGTH, build_total_mileage(datagram, mileage_times_10_in), "incorrect_datagram_length");
	parse_total_mileage(datagram, &mileage_times_10_out);
	CHECK_UINT32(mileage_times_10);
}

TEST_DATAGRAM(22) // total mileage
for (int i = 0; i <= 0xfff0 / 10; i++) {
	for (int j = 0; j <= 9; j++) {
		assert_total_mileage((i * 10) + j);
	}
}
}

void assert_water_temperature(char degrees_celcius_in, bool transducer_defective_in) {
	unsigned char datagram[256];
	char degrees_celcius_out;
	bool transducer_defective_out;
	assert_equal_int(DATAGRAM_23_LENGTH, build_water_temperature(datagram, degrees_celcius_in, transducer_defective_in), "incorrect datagram length");
	parse_water_temperature(datagram, &degrees_celcius_out, &transducer_defective_out);
	CHECK_INT16(degrees_celcius);
	CHECK_BOOL(transducer_defective);
}

TEST_DATAGRAM(23) // water temperature
for (char i = -80; i <= 50; i++) {
	assert_water_temperature(i, 0);
}
assert_water_temperature(75, 1);
}

void assert_speed_distance_units(DISTANCE_UNITS distance_units_in) {
	unsigned char datagram[256];
	DISTANCE_UNITS distance_units_out;
	assert_equal_int(DATAGRAM_24_LENGTH, build_speed_distance_units(datagram, distance_units_in), "incorrect datagram length");
	parse_speed_distance_units(datagram, &distance_units_out);
	CHECK_CHAR(distance_units);
}

TEST_DATAGRAM(24) // display units for mileage and speed
assert_speed_distance_units(DISTANCE_UNITS_NAUTICAL);
assert_speed_distance_units(DISTANCE_UNITS_STATUTE);
assert_speed_distance_units(DISTANCE_UNITS_METRIC);
}

void assert_total_and_trip_mileage(int total_nm_times_10_in, int trip_nm_times_100_in) {
	unsigned char datagram[256];
	unsigned long int total_nm_times_10_out, trip_nm_times_100_out;
	assert_equal_int(DATAGRAM_25_LENGTH, build_total_and_trip_mileage(datagram, total_nm_times_10_in, trip_nm_times_100_in), "incorrect datagram length");
	parse_total_and_trip_mileage(datagram, &total_nm_times_10_out, &trip_nm_times_100_out);
	CHECK_UINT32(total_nm_times_10);
	CHECK_UINT32(trip_nm_times_100);
}

TEST_DATAGRAM(25) // total and trip mileage
for (int i = 0; i < 104856; i += 101) {
	for (int j = 0; j <= 9; j++) {
		assert_total_and_trip_mileage((i * 10) + j, 0);
	}
}
for (int i = 0; i < 10484; i += 97) {
	for (int j = 0; j <= 99; j++) {
		assert_total_and_trip_mileage(0, (i * 100) + j);
	}
}
}

void assert_average_water_speed(short int knots_1_times_100_in, short int knots_2_times_100_in, short int speed_1_from_sensor_in, short int speed_2_is_average_in, short int average_is_stopped_in, bool display_in_statute_miles_in) {
	unsigned char datagram[256];
	short int knots_1_times_100_out, knots_2_times_100_out, speed_1_from_sensor_out, speed_2_is_average_out, average_is_stopped_out;
	bool display_in_statute_miles_out;
	assert_equal_int(DATAGRAM_26_LENGTH, build_average_water_speed(datagram, knots_1_times_100_in, knots_2_times_100_in, speed_1_from_sensor_in, speed_2_is_average_in, average_is_stopped_in, display_in_statute_miles_in), "incorrect datagram length");
	parse_average_water_speed(datagram, &knots_1_times_100_out, &knots_2_times_100_out, &speed_1_from_sensor_out, &speed_2_is_average_out, &average_is_stopped_out, &display_in_statute_miles_out);
	CHECK_INT16(knots_1_times_100);
	CHECK_INT16(knots_2_times_100);
	CHECK_INT16(speed_1_from_sensor);
	CHECK_INT16(speed_2_is_average);
	CHECK_BOOL(average_is_stopped);
	CHECK_BOOL(display_in_statute_miles);
}

TEST_DATAGRAM(26) // water speed for two sensors or with average
for (int i = 0; i < 65000; i += 101) {
	assert_average_water_speed(i, 600, 1, 1, 0, 0);
	assert_average_water_speed(600, i, 1, 1, 0, 0);
}
assert_average_water_speed(600, 600, 0, 0, 1, 1);
}

void assert_precise_water_temperature(short int degrees_celsius_times_10_in) {
	unsigned char datagram[256];
	short int degrees_celsius_times_10_out;
	assert_equal_int(DATAGRAM_27_LENGTH, build_precise_water_temperature(datagram, degrees_celsius_times_10_in), "incorrect datagram length");
	parse_precise_water_temperature(datagram, &degrees_celsius_times_10_out);
	CHECK_INT16(degrees_celsius_times_10);
}

TEST_DATAGRAM(27) // water temperature precise to 0.1 degrees C
for (int i = 0; i <= 650; i += 101) {
	for (int j = 0; j < 100; j++) {
		assert_precise_water_temperature((i * 100) + j);
	}
}
}

void assert_lamp_intensity(unsigned char intensity_in) {
	unsigned char datagram[256];
	unsigned char intensity_out;
	assert_equal_int(DATAGRAM_30_LENGTH, build_lamp_intensity(datagram, intensity_in), "incorrect datagram length");
	parse_lamp_intensity(datagram, &intensity_out);
	CHECK_CHAR(intensity);
}

TEST_DATAGRAM(30) // set lamp intensity
for (int i = 0; i <= 3; i++) {
	assert_lamp_intensity(i);
}
}

void assert_lat_position(LATITUDE_HEMISPHERE hemisphere_in, unsigned char degrees_in, unsigned short int minutes_times_100_in) {
	unsigned char datagram[256];
	unsigned char degrees_out;
	unsigned short int minutes_times_100_out;
	LATITUDE_HEMISPHERE hemisphere_out;
	assert_equal_int(DATAGRAM_50_LENGTH, build_lat_position(datagram, hemisphere_in, degrees_in, minutes_times_100_in), "incorrect datagram length");
	parse_lat_position(datagram, &hemisphere_out, &degrees_out, &minutes_times_100_out);
	CHECK_CHAR(hemisphere);
	CHECK_CHAR(degrees);
	CHECK_INT16(minutes_times_100);
}

TEST_DATAGRAM(50) // latitude
for (int i = 0; i < 90; i += 5) {
	for (int j = 0; j < 6000; j += 37) {
		assert_lat_position(LATITUDE_HEMISPHERE_NORTH, i, j);
		assert_lat_position(LATITUDE_HEMISPHERE_SOUTH, i, j);
	}
}
}

void assert_lon_position(LONGITUDE_HEMISPHERE hemisphere_in, unsigned char degrees_in, unsigned short int minutes_times_100_in) {
	unsigned char datagram[256];
	unsigned char degrees_out;
	unsigned short int minutes_times_100_out;
	LONGITUDE_HEMISPHERE hemisphere_out;
	assert_equal_int(DATAGRAM_51_LENGTH, build_lon_position(datagram, hemisphere_in, degrees_in, minutes_times_100_in), "incorrect datagram length");
	parse_lon_position(datagram, &hemisphere_out, &degrees_out, &minutes_times_100_out);
	CHECK_CHAR(hemisphere);
	CHECK_CHAR(degrees);
	CHECK_INT16(minutes_times_100);
}

TEST_DATAGRAM(51) // longitude
for (int i = 0; i < 180; i += 5) {
	for (int j = 6000; j <= 6000; j += 37) {
		assert_lon_position(LONGITUDE_HEMISPHERE_WEST, i, j);
		assert_lon_position(LONGITUDE_HEMISPHERE_EAST, i, j);
	}
}
}

void assert_speed_over_ground(unsigned short int knots_times_10_in) {
	unsigned char datagram[256];
	unsigned short int knots_times_10_out;
	assert_equal_int(DATAGRAM_52_LENGTH, build_speed_over_ground(datagram, knots_times_10_in), "incorrect datagram length");
	parse_speed_over_ground(datagram, &knots_times_10_out);
	CHECK_INT16(knots_times_10);
}

TEST_DATAGRAM(52) // speed over ground
for (int i = 0; i <= 6500; i += 101) {
	for (int j = 0; j <= 9; j++) {
		assert_speed_over_ground((i * 10) + j);
	}
}
}

void assert_course_over_ground(short int degrees_in) {
	unsigned char datagram[256];
	short int degrees_out;
	assert_equal_int(DATAGRAM_53_LENGTH, build_course_over_ground(datagram, degrees_in), "incorrect datagram length");
	parse_course_over_ground(datagram, &degrees_out);
	CHECK_INT16(degrees);
}

TEST_DATAGRAM(53) // course over ground
for (int i = 0; i < 360; i++) {
	assert_course_over_ground(i);
}
}

void assert_gmt_time(unsigned char hours_in, unsigned char minutes_in, unsigned char seconds_in) {
	unsigned char datagram[256];
	unsigned char hours_out, minutes_out, seconds_out;
	assert_equal_int(DATAGRAM_54_LENGTH, build_gmt_time(datagram, hours_in, minutes_in, seconds_in), "incorrect datagram length");
	parse_gmt_time(datagram, &hours_out, &minutes_out, &seconds_out);
	CHECK_CHAR(hours);
	CHECK_CHAR(minutes);
	CHECK_CHAR(seconds);
}

TEST_DATAGRAM(54) // gmt time
for (int i = 0; i < 24; i++) {
	for (int j = 0; j < 60; j += 7) {
		for (int k = 0; k < 60; k += 11) {
			assert_gmt_time(i, j, k);
		}
	}
}
}

void assert_date(unsigned short int year_in, unsigned char month_in, unsigned char day_in) {
	unsigned char datagram[256];
	unsigned short int year_out;
	unsigned char month_out, day_out;
	assert_equal_int(DATAGRAM_56_LENGTH, build_date(datagram, year_in, month_in, day_in), "incorrect datagram length");
	parse_date(datagram, &year_out, &month_out, &day_out);
	CHECK_CHAR(year);
	CHECK_CHAR(month);
	CHECK_CHAR(day);
}

TEST_DATAGRAM(56) // current date
for (int i = 1975; i < 2100; i++) {
	for (int j = 1; j <= 12; j++) {
		for (int k = 1; k <= 28; k += 3) {
			assert_date(i, j, k);
		}
	}
}
}

void assert_satellite_info(unsigned char satellite_count_in, unsigned char horizontal_dilution_of_position_in) {
	unsigned char datagram[256];
	unsigned char satellite_count_out, horizontal_dilution_of_position_out;
	assert_equal_int(DATAGRAM_57_LENGTH, build_satellite_info(datagram, satellite_count_in, horizontal_dilution_of_position_in), "incorrect datagram length");
	parse_satellite_info(datagram, &satellite_count_out, &horizontal_dilution_of_position_out);
	CHECK_INT16(satellite_count);
	CHECK_INT16(horizontal_dilution_of_position);
}

TEST_DATAGRAM(57) // satellite info
for (int i = 3; i < 16; i++) {
	for (int j = 10; j < 100; j++) {
		assert_satellite_info(i, j);
	}
}
}

void assert_lat_lon_position(LATITUDE_HEMISPHERE hemisphere_latitude_in, unsigned char degrees_lat_in, unsigned short int minutes_lat_times_1000_in, LONGITUDE_HEMISPHERE hemisphere_longitude_in, unsigned char  degrees_lon_in, unsigned short int minutes_lon_times_1000_in) {
	unsigned char datagram[256];
	unsigned char degrees_lat_out, degrees_lon_out;
	unsigned short int minutes_lat_times_1000_out, minutes_lon_times_1000_out;
	LATITUDE_HEMISPHERE hemisphere_latitude_out;
	LONGITUDE_HEMISPHERE hemisphere_longitude_out;
	assert_equal_int(DATAGRAM_58_LENGTH, build_lat_lon_position(datagram, hemisphere_latitude_in, degrees_lat_in, minutes_lat_times_1000_in, hemisphere_longitude_in, degrees_lon_in, minutes_lon_times_1000_in), "incorrect datagram length");
	parse_lat_lon_position(datagram, &hemisphere_latitude_out, &degrees_lat_out, &minutes_lat_times_1000_out, &hemisphere_longitude_out, &degrees_lon_out, &minutes_lon_times_1000_out);
	CHECK_INT16(hemisphere_latitude);
	CHECK_INT16(degrees_lat);
	CHECK_INT16(minutes_lat_times_1000);
	CHECK_INT16(hemisphere_longitude);
	CHECK_INT16(degrees_lon);
	CHECK_INT16(minutes_lon_times_1000);
}

TEST_DATAGRAM(58) // lat/lon position
for (int i = 0; i < 180; i++) {
	for (int j = 0; j < 60000; j += 1001) {
		assert_lat_lon_position(LATITUDE_HEMISPHERE_NORTH, i / 2, 60000 - j, LONGITUDE_HEMISPHERE_WEST, i, j);
		assert_lat_lon_position(LATITUDE_HEMISPHERE_SOUTH, i / 2, 60000 - j, LONGITUDE_HEMISPHERE_EAST, i, j);
		assert_lat_lon_position(LATITUDE_HEMISPHERE_SOUTH, i / 2, 60000 - j, LONGITUDE_HEMISPHERE_WEST, i, j);
		assert_lat_lon_position(LATITUDE_HEMISPHERE_NORTH, i / 2, 60000 - j, LONGITUDE_HEMISPHERE_EAST, i, j);
	}
}
}

void assert_countdown_timer(unsigned char hours_in, unsigned char minutes_in, unsigned char seconds_in, TIMER_MODE mode_in) {
	unsigned char datagram[256];
	unsigned char hours_out, minutes_out, seconds_out;
	TIMER_MODE mode_out;
	assert_equal_int(DATAGRAM_59_LENGTH, build_countdown_timer(datagram, hours_in, minutes_in, seconds_in, mode_in), "incorrect datagram length");
	parse_countdown_timer(datagram, &hours_out, &minutes_out, &seconds_out, &mode_out);
	CHECK_CHAR(hours);
	CHECK_CHAR(minutes);
	CHECK_CHAR(seconds);
	CHECK_CHAR(mode);
}

TEST_DATAGRAM(59) // countdown timer
for (int i = 0; i <= 9; i++) {
	for (int j = 0; j <= 63; j++) {
		for (int k = 0; k <= 59; k++) {
			assert_countdown_timer(i, j, k, TIMER_MODE_COUNT_UP_AND_START);
		}
	}
}
assert_countdown_timer(0, 5, 0, TIMER_MODE_COUNT_UP);
assert_countdown_timer(0, 5, 0, TIMER_MODE_COUNT_DOWN);
assert_countdown_timer(0, 5, 0, TIMER_MODE_COUNT_UP_AND_START);
assert_countdown_timer(0, 5, 0, TIMER_MODE_COUNT_DOWN_AND_START);
}

void assert_wind_alarm(short int active_alarms_in) {
	unsigned char datagram[256];
	short int active_alarms_out;
	assert_equal_int(DATAGRAM_66_LENGTH, build_wind_alarm(datagram, active_alarms_in), "incorrect datagram length");
	parse_wind_alarm(datagram, &active_alarms_out);
	CHECK_INT16(active_alarms);
}

TEST_DATAGRAM(66) // wind alarm
assert_wind_alarm(ALARM_APPARENT_WIND_ANGLE_LOW);
assert_wind_alarm(ALARM_APPARENT_WIND_ANGLE_HIGH);
assert_wind_alarm(ALARM_APPARENT_WIND_SPEED_LOW);
assert_wind_alarm(ALARM_APPARENT_WIND_SPEED_HIGH);
assert_wind_alarm(ALARM_TRUE_WIND_ANGLE_LOW);
assert_wind_alarm(ALARM_TRUE_WIND_ANGLE_HIGH);
assert_wind_alarm(ALARM_TRUE_WIND_SPEED_LOW);
assert_wind_alarm(ALARM_TRUE_WIND_SPEED_HIGH);
assert_wind_alarm(ALARM_APPARENT_WIND_ANGLE_LOW | ALARM_APPARENT_WIND_ANGLE_HIGH | ALARM_APPARENT_WIND_SPEED_LOW | ALARM_APPARENT_WIND_SPEED_HIGH | ALARM_TRUE_WIND_ANGLE_LOW | ALARM_TRUE_WIND_ANGLE_HIGH | ALARM_TRUE_WIND_SPEED_LOW | ALARM_TRUE_WIND_SPEED_HIGH);
}

void assert_alarm_acknowledgement(short int acknowledged_alarm_in) {
	unsigned char datagram[256];
	short int acknowledged_alarm_out;
	assert_equal_int(DATAGRAM_68_LENGTH, build_alarm_acknowledgement(datagram, acknowledged_alarm_in), "incorrect datagram length");
	parse_alarm_acknowledgement(datagram, &acknowledged_alarm_out);
	CHECK_INT16(acknowledged_alarm);
}

TEST_DATAGRAM(68) // acknowledge alarm
assert_alarm_acknowledgement(ALARM_SHALLOW_WATER);
assert_alarm_acknowledgement(ALARM_DEEP_WATER);
assert_alarm_acknowledgement(ALARM_ANCHOR);
assert_alarm_acknowledgement(ALARM_TRUE_WIND_SPEED_HIGH);
assert_alarm_acknowledgement(ALARM_TRUE_WIND_SPEED_LOW);
assert_alarm_acknowledgement(ALARM_TRUE_WIND_ANGLE_HIGH);
assert_alarm_acknowledgement(ALARM_TRUE_WIND_ANGLE_LOW);
assert_alarm_acknowledgement(ALARM_APPARENT_WIND_SPEED_HIGH);
assert_alarm_acknowledgement(ALARM_APPARENT_WIND_SPEED_LOW);
assert_alarm_acknowledgement(ALARM_APPARENT_WIND_ANGLE_HIGH);
assert_alarm_acknowledgement(ALARM_APPARENT_WIND_ANGLE_LOW);
}

void assert_target_waypoint_name(char char1_in, char char2_in, char char3_in, char char4_in) {
	unsigned char datagram[256];
	char char1_out, char2_out, char3_out, char4_out;
	assert_equal_int(DATAGRAM_82_LENGTH, build_target_waypoint_name(datagram, char1_in, char2_in, char3_in, char4_in), "incorrect_datagram_length");
	assert_equal_int(0, parse_target_waypoint_name(datagram, &char1_out, &char2_out, &char3_out, &char4_out), "failed checksum");
	CHECK_CHAR(char1);
	CHECK_CHAR(char2);
	CHECK_CHAR(char3);
	CHECK_CHAR(char4);
}

TEST_DATAGRAM(82) // target waypoint name
for (int i = 'A'; i < 'Z'; i++) {
	for (int j = 'A'; j < 'Z'; j++) {
		assert_target_waypoint_name(i, i + 1, j, j + 1);
	}
}
}

void assert_course_computer_failure(COURSE_COMPUTER_FAILURE_TYPE failure_type_in) {
	unsigned char datagram[256];
	COURSE_COMPUTER_FAILURE_TYPE failure_type_out;
	assert_equal_int(DATAGRAM_83_LENGTH, build_course_computer_failure(datagram, failure_type_in), "incorrect datagram length");
	parse_course_computer_failure(datagram, &failure_type_out);
	CHECK_INT16(failure_type);
}

TEST_DATAGRAM(83) // course computer failure
assert_course_computer_failure(COURSE_COMPUTER_FAILURE_TYPE_NONE);
assert_course_computer_failure(COURSE_COMPUTER_FAILURE_TYPE_AUTO_RELEASE_ERROR);
assert_course_computer_failure(COURSE_COMPUTER_FAILURE_TYPE_DRIVE_STOPPED);
}

void assert_autopilot_status(int compass_heading_in, TURN_DIRECTION turning_direction_in, int target_heading_in, AUTOPILOT_MODE mode_in, int rudder_position_in, int alarms_in, int display_flags_in) {
	unsigned char datagram[256];
	short int compass_heading_out, target_heading_out;
	short int rudder_position_out;
	unsigned short int alarms_out;
	unsigned char display_flags_out;
	TURN_DIRECTION turning_direction_out;
	AUTOPILOT_MODE mode_out;
	assert_equal_int(DATAGRAM_84_LENGTH, build_autopilot_status(datagram, compass_heading_in, turning_direction_in, target_heading_in, mode_in, rudder_position_in, alarms_in, display_flags_in), "incorrect datagram length");
	parse_autopilot_status(datagram, &compass_heading_out, &turning_direction_out, &target_heading_out, &mode_out, &rudder_position_out, &alarms_out, &display_flags_out);
	CHECK_INT16(compass_heading);
	CHECK_INT16(turning_direction);
	CHECK_INT16(target_heading);
	CHECK_INT16(mode);
	CHECK_INT16(alarms);
	CHECK_INT16(rudder_position);
	CHECK_INT16(display_flags);
}

TEST_DATAGRAM(84) // autopilot status
for (int i = 0; i <= 359; i++) {
	for (int j = -3; j <= 3; j++) {
		assert_autopilot_status(i, TURN_DIRECTION_LEFT, 300, AUTOPILOT_MODE_AUTO, j, 0, 0);
	}
}
assert_autopilot_status(359, TURN_DIRECTION_RIGHT, 300, AUTOPILOT_MODE_AUTO, 0, ALARM_AUTOPILOT_OFF_COURSE, AUTOPILOT_DISPLAY_OFF);
assert_autopilot_status(0, TURN_DIRECTION_LEFT, 300, AUTOPILOT_MODE_VANE, 0, ALARM_AUTOPILOT_WIND_SHIFT, AUTOPILOT_DISPLAY_NO_DATA | AUTOPILOT_DISPLAY_LARGE_XTE);
assert_autopilot_status(180, TURN_DIRECTION_RIGHT, 300, AUTOPILOT_MODE_TRACK, 0, 0, AUTOPILOT_DISPLAY_AUTO_REL);
}

//TEST(parse_84)
//  // 84: compass heading, autopilot course and rudder position
//  // sample message states compass = 016
//  // autopilot not turning
//  // autopilot course 000
//  // autopilot in standby mode
//  // no alarms
//  // rudder position on centre
//  // no display instructions
//  // 0x0f in final byte meaning unknown; signal copied from Autohelm 1000+
//  struct AUTOPILOT_STATUS ap_status;
//  handle_seatalk_datagram("\x84\xc6\x07\x00\x00\x00\x00\x00\x0f");
//  get_autopilot_status(&ap_status);
//  assert_equal_int(16, ap_status.compass_heading,
//    "ap_status.compass_heading");
//  assert_equal_string("right", ap_status.turning_direction,
//    "ap_status.turning_direction");
//  assert_equal_int(0, ap_status.target_heading, "ap_status.target");
//  assert_equal_string("standby", ap_status.mode, "ap_status.mode");
//  assert_equal_int(0, ap_status.off_course_alarm,
//    "ap_status.off_course_alarm");
//  assert_equal_int(0, ap_status.wind_shift_alarm,
//    "ap_status.wind_shift_alarm");
//  assert_equal_int(0, ap_status.rudder_position,
//    "ap_status.rudder_position");
//  assert_equal_int(0, ap_status.heading_display_off,
//    "ap_status.heading_display_off");
//  assert_equal_int(0, ap_status.no_data, "ap_status.no_data");
//  assert_equal_int(0, ap_status.large_xte, "ap_status.large_xte");
//  assert_equal_int(0, ap_status.auto_rel, "ap_status.auto_rel");
//  // printf("ap_status: %s", buf);
//}

void assert_waypoint_navigation(bool cross_track_error_present_in, unsigned short int cross_track_error_times_100_in, bool waypoint_bearing_present_in, unsigned short int waypoint_bearing_in, bool bearing_is_magnetic_in, bool waypoint_distance_present_in, unsigned short int waypoint_distance_times_100_in, short int direction_to_steer_in) {
	unsigned char datagram[256];
	bool cross_track_error_present_out, waypoint_bearing_present_out, bearing_is_magnetic_out, waypoint_distance_present_out;
	unsigned short int cross_track_error_times_100_out, waypoint_bearing_out, waypoint_distance_times_100_out;
	short int direction_to_steer_out;
	assert_equal_int(DATAGRAM_85_LENGTH, build_waypoint_navigation(datagram, cross_track_error_present_in, cross_track_error_times_100_in, waypoint_bearing_present_in, waypoint_bearing_in, bearing_is_magnetic_in, waypoint_distance_present_in, waypoint_distance_times_100_in, direction_to_steer_in), "incorrect datagram length");
	assert_equal_int(0, parse_waypoint_navigation(datagram, &cross_track_error_present_out, &cross_track_error_times_100_out, &waypoint_bearing_present_out, &waypoint_bearing_out, &bearing_is_magnetic_out, &waypoint_distance_present_out, &waypoint_distance_times_100_out, &direction_to_steer_out), "checksum");;
	CHECK_INT16(cross_track_error_present);
	CHECK_INT16(cross_track_error_times_100);
	CHECK_INT16(waypoint_bearing_present);
	CHECK_INT16(waypoint_bearing);
	CHECK_INT16(bearing_is_magnetic);
	CHECK_INT16(waypoint_distance_present);
	CHECK_INT16(direction_to_steer);
}

TEST_DATAGRAM(85) // waypoint navigation
for (int i = 0; i <= 40; i++) {
	for (int j = 0; j < 360; j++) {
		assert_waypoint_navigation(1, i, 1, j, 1, 1, i + 1, 1);
		assert_waypoint_navigation(1, i, 1, j, 1, 1, i + 1, 1);
	}
}
assert_waypoint_navigation(0, 100, 1, 200, 1, 1, 300, 1);
assert_waypoint_navigation(1, 100, 0, 200, 1, 1, 300, 1);
assert_waypoint_navigation(1, 100, 1, 200, 0, 1, 300, 1);
assert_waypoint_navigation(1, 100, 1, 200, 1, 0, 300, 1);
assert_waypoint_navigation(1, 100, 1, 200, 1, 1, 300, -1);
}

void assert_autopilot_command(ST_AUTOPILOT_COMMAND command_in) {
	unsigned char datagram[256];
	ST_AUTOPILOT_COMMAND command_out;
	assert_equal_int(DATAGRAM_86_LENGTH, build_autopilot_command(datagram, command_in), "incorrect datagram length");
	assert_equal_int(0, parse_autopilot_command(datagram, &command_out), "checksum");
	CHECK_INT16(command);
}

TEST_DATAGRAM(86) // autopilot command
assert_autopilot_command(ST_AUTOPILOT_COMMAND_AUTO);
assert_autopilot_command(ST_AUTOPILOT_COMMAND_STANDBY);
}

void assert_set_autopilot_response_level(AUTOPILOT_RESPONSE_LEVEL response_level_in) {
	unsigned char datagram[256];
	AUTOPILOT_RESPONSE_LEVEL response_level_out;
	assert_equal_int(DATAGRAM_87_LENGTH, build_set_autopilot_response_level(datagram, response_level_in), "incorrect datagram length");
	parse_set_autopilot_response_level(datagram, &response_level_out);
	CHECK_CHAR(response_level);
}

TEST_DATAGRAM(87) // set autopilot response level
assert_set_autopilot_response_level(AUTOPILOT_RESPONSE_LEVEL_AUTOMATIC_DEADBAND);
assert_set_autopilot_response_level(AUTOPILOT_RESPONSE_LEVEL_MINIMUM_DEADBAND);
}

void assert_heading(int heading_in, short int locked_heading_active_in, short int locked_heading_in) {
	unsigned char datagram[256];
	short int heading_out, locked_heading_out;
	bool locked_heading_active_out;
	assert_equal_int(DATAGRAM_89_LENGTH, build_heading(datagram, heading_in, locked_heading_active_in, locked_heading_in), "incorrect datagram length");
	parse_heading(datagram, &heading_out, &locked_heading_active_out, &locked_heading_out);
	CHECK_INT16(heading);
	CHECK_BOOL(locked_heading_active);
	CHECK_INT16(locked_heading);
}

TEST_DATAGRAM(89) // heading
for (int i = 0; i <= 359; i++) {
	assert_heading(i, 0, 0);
	assert_heading(10, 1, i);
	assert_heading(12, 0, i);
}
}

void assert_set_rudder_gain(unsigned char rudder_gain_in) {
	unsigned char datagram[256];
	unsigned char rudder_gain_out;
	assert_equal_int(DATAGRAM_91_LENGTH, build_set_rudder_gain(datagram, rudder_gain_in), "incorrect datagram length");
	parse_set_rudder_gain(datagram, &rudder_gain_out);
	CHECK_CHAR(rudder_gain);
}

TEST_DATAGRAM(91) // set rudder gain
for (int i = 1; i <= 9; i++) {
	assert_set_rudder_gain(i);
}
}

void assert_compass_variation(short int degrees_in) {
	unsigned char datagram[256];
	short int degrees_out;
	assert_equal_char(DATAGRAM_99_LENGTH, build_compass_variation(datagram, degrees_in), "incorrect datagram length");
	parse_compass_variation(datagram, &degrees_out);
	CHECK_INT16(degrees);
}

TEST_DATAGRAM(99) // compass variation
for (int i = -45; i <= 45; i++) {
	assert_compass_variation(i);
}
}

void assert_heading_and_rudder_position(short int heading_in, TURN_DIRECTION turning_direction_in, short int rudder_position_in) {
	unsigned char datagram[256];
	short int heading_out;
	short int rudder_position_out;
	TURN_DIRECTION turning_direction_out;
	assert_equal_int(DATAGRAM_9C_LENGTH, build_heading_and_rudder_position(datagram, heading_in, turning_direction_in, rudder_position_in), "incorrect datagram length");
	//  print_datagram(datagram, DATAGRAM_9C_LENGTH);
	//  printf("heading_in: %d; turning_direction_in: %d\n", heading_in, turning_direction_in);
	parse_heading_and_rudder_position(datagram, &heading_out, &turning_direction_out, &rudder_position_out);
	CHECK_INT16(heading);
	CHECK_INT16(turning_direction);
	CHECK_CHAR(rudder_position);
}

TEST_DATAGRAM(9c) // compass heading and rudder position
for (int i = -3; i <= 3; i++) {
	for (int j = 0; j < 360; j++) {
		assert_heading_and_rudder_position(j, TURN_DIRECTION_LEFT, i);
		assert_heading_and_rudder_position(j, TURN_DIRECTION_RIGHT, i);
	}
}
}

void test_seatalk_datagram() {
	printf("\n--- Testing seatalk_datagram.c\n");
	test_datagram_00();
	test_datagram_05();
	test_datagram_10();
	test_datagram_11();
	test_datagram_20();
	test_datagram_21();
	test_datagram_22();
	test_datagram_23();
	test_datagram_24();
	test_datagram_25();
	test_datagram_26();
	test_datagram_27();
	test_datagram_30();
	// test_datagram_36(); // this datagram has no variability
	test_datagram_50();
	test_datagram_51();
	test_datagram_52();
	test_datagram_53();
	test_datagram_54();
	// test_datagram_55(); // TRACK keystroke
	test_datagram_56();
	test_datagram_57();
	test_datagram_58();
	test_datagram_59();
	test_datagram_66();
	test_datagram_68();
	test_datagram_82();
	test_datagram_83();
	test_datagram_84();
	test_datagram_85();
	test_datagram_86();
	test_datagram_87();
	test_datagram_89();
	test_datagram_91();
	test_datagram_99();
	test_datagram_9c();
	//  test_parse_84();
}
