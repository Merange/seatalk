#include "seatalk_protocol.h"
#include "boat_sensor.h"
#include "seatalk_datagram.h"
#include "seatalk_transport_layer.h"
#include "seatalk_command.h"
#include "logger.h"

void wake_transmitter(void) {
	initiate_seatalk_transmitter();
}

#define SENSOR_PENDING(NAME, ID_NAME) if (has_##NAME##_sensor_value_to_transmit()) {\
  *sensor_id = SENSOR_ID_##ID_NAME;\
}

int seatalk_sensor_pending(SENSOR_ID* sensor_id) {
	SENSOR_PENDING(heading, HEADING)
else SENSOR_PENDING(water_speed_in_knots_times_100, WATER_SPEED)
  else SENSOR_PENDING(apparent_wind_angle, APPARENT_WIND_ANGLE)
  else SENSOR_PENDING(apparent_wind_speed_in_knots_times_10, APPARENT_WIND_SPEED)
  else SENSOR_PENDING(depth_below_transducer_in_feet_times_10, DEPTH_BELOW_TRANSDUCER)
  else SENSOR_PENDING(course_over_ground, COURSE_OVER_GROUND)
  else SENSOR_PENDING(speed_over_ground_in_knots_times_100, SPEED_OVER_GROUND)
  else SENSOR_PENDING(water_temperature_in_degrees_celsius_times_10, WATER_TEMPERATURE)
  else SENSOR_PENDING(rudder_position_in_degrees_right, RUDDER_POSITION)
  else {
	  *sensor_id = SENSOR_ID_NONE;
  }
  return *sensor_id != SENSOR_ID_NONE;
}

unsigned char build_heading_sensor_datagram(unsigned char* datagram) {
	short int heading;
	short int rudder_position_in_degrees_right;
	if (pop_heading_sensor_value(&heading) == 0) {
		if (pop_rudder_position_in_degrees_right_sensor_value(&rudder_position_in_degrees_right) == 0) {
			return build_heading_and_rudder_position(datagram, heading, 0, rudder_position_in_degrees_right);
		}
		else {
			return build_heading(datagram, heading, 0, 0);
		}
	}
	return 0;
}

unsigned char build_water_speed_sensor_datagram(unsigned char* datagram) {
	unsigned short int water_speed_in_knots_times_100;
	if (pop_water_speed_in_knots_times_100_sensor_value(&water_speed_in_knots_times_100) == 0) {
		return build_average_water_speed(datagram, water_speed_in_knots_times_100, 0, 1, 0, 1, 0);
	}
	return 0;
}

unsigned char build_apparent_wind_angle_sensor_datagram(unsigned char* datagram) {
	short int apparent_wind_angle;
	if (pop_apparent_wind_angle_sensor_value(&apparent_wind_angle) == 0) {
		return build_apparent_wind_angle(datagram, apparent_wind_angle * 2);
	}
	return 0;
}

unsigned char build_apparent_wind_speed_sensor_datagram(unsigned char* datagram) {
	unsigned short int apparent_wind_speed_in_knots_times_10;
	if (pop_apparent_wind_speed_in_knots_times_10_sensor_value(&apparent_wind_speed_in_knots_times_10) == 0) {
		return build_apparent_wind_speed(datagram, apparent_wind_speed_in_knots_times_10, 0);
	}
	return 0;
}

unsigned char build_depth_below_transducer_sensor_datagram(unsigned char* datagram) {
	short int depth_below_transducer_in_feet_times_10;
	if (pop_depth_below_transducer_in_feet_times_10_sensor_value(&depth_below_transducer_in_feet_times_10) == 0) {
		return build_depth_below_transducer(datagram, depth_below_transducer_in_feet_times_10, 0, 0, 0);
	}
	return 0;
}

unsigned char build_course_over_ground_sensor_datagram(unsigned char* datagram) {
	short int course_over_ground;
	if (pop_course_over_ground_sensor_value(&course_over_ground) == 0) {
		return build_course_over_ground(datagram, course_over_ground);
	}
	return 0;
}

unsigned char build_speed_over_ground_sensor_datagram(unsigned char* datagram) {
	unsigned short int speed_over_ground_in_knots_times_100;
	if (pop_speed_over_ground_in_knots_times_100_sensor_value(&speed_over_ground_in_knots_times_100) == 0) {
		return build_speed_over_ground(datagram, speed_over_ground_in_knots_times_100 / 10);
	}
	return 0;
}

unsigned char build_water_temperature_sensor_datagram(unsigned char* datagram) {
	short int water_temperature_in_degrees_celsius_times_10;
	if (pop_water_temperature_in_degrees_celsius_times_10_sensor_value(&water_temperature_in_degrees_celsius_times_10) == 0) {
		return build_precise_water_temperature(datagram, water_temperature_in_degrees_celsius_times_10);
	}
	return 0;
}

unsigned char build_rudder_position_sensor_datagram(unsigned char* datagram) {
	short int rudder_position_in_degrees_right;
	short int heading;
	if (pop_rudder_position_in_degrees_right_sensor_value(&rudder_position_in_degrees_right) == 0) {
		if (pop_heading_sensor_value(&heading) == 0) {
			build_heading_and_rudder_position(datagram, heading, 0, rudder_position_in_degrees_right);
		}
		else {
			// no datagram for just rudder position; skip
		}
	}
	return 0;
}

unsigned char build_sensor_datagram(unsigned char* datagram) {
	SENSOR_ID sensor_id;
	if (!seatalk_sensor_pending(&sensor_id)) {
		return 0;
	}
	switch (sensor_id) {
	case SENSOR_ID_HEADING:
		return build_heading_sensor_datagram(datagram);
		break;
	case SENSOR_ID_WATER_SPEED:
		return build_water_speed_sensor_datagram(datagram);
		break;
	case SENSOR_ID_APPARENT_WIND_ANGLE:
		return build_apparent_wind_angle_sensor_datagram(datagram);
		break;
	case SENSOR_ID_APPARENT_WIND_SPEED:
		return build_apparent_wind_speed_sensor_datagram(datagram);
		break;
	case SENSOR_ID_DEPTH_BELOW_TRANSDUCER:
		return build_depth_below_transducer_sensor_datagram(datagram);
		break;
	case SENSOR_ID_COURSE_OVER_GROUND:
		return build_course_over_ground_sensor_datagram(datagram);
		break;
	case SENSOR_ID_SPEED_OVER_GROUND:
		return build_speed_over_ground_sensor_datagram(datagram);
		break;
	case SENSOR_ID_WATER_TEMPERATURE:
		return build_water_temperature_sensor_datagram(datagram);
		break;
	case SENSOR_ID_RUDDER_POSITION:
		return build_rudder_position_sensor_datagram(datagram);
		break;
	default:
		return 0;
	}
}

int get_pending_datagram(unsigned char* datagram) {
	int length;
	if ((length = get_command_datagram(datagram))) {
		return length;
	}
	else if ((length = build_sensor_datagram(datagram))) {
		return length;
	}
	return 0;
}

void update_depth_below_transducer(unsigned char* datagram) {
	short int depth_in_feet_times_10;
	bool display_in_metres, transducer_defective;
	unsigned short int active_alarms;
	unsigned short int current_active_alarms;
	parse_depth_below_transducer(datagram, &depth_in_feet_times_10, &display_in_metres, &active_alarms, &transducer_defective);
	set_depth_below_transducer_in_feet_times_10(depth_in_feet_times_10);
	if (active_alarms) {
		if (get_active_alarms(&current_active_alarms) != 0) {
			current_active_alarms = 0;
		}
		set_active_alarms(current_active_alarms | active_alarms);
	}
}

void update_engine_rpm_and_pitch(unsigned char* datagram) {
	ENGINE_ID engine_id;
	short int rpm;
	short int pitch_percent;
	parse_engine_rpm_and_pitch(datagram, &engine_id, &rpm, &pitch_percent);
	set_engine_rpm(engine_id, rpm);
	set_engine_prop_pitch_percent(engine_id, pitch_percent);
}

void update_apparent_wind_angle(unsigned char* datagram) {
	short int degrees_right_times_2;
	parse_apparent_wind_angle(datagram, &degrees_right_times_2);
	set_apparent_wind_angle(degrees_right_times_2 >> 1);
}

void update_apparent_wind_speed(unsigned char* datagram) {
	short int knots_times_10;
	bool display_in_metric;
	parse_apparent_wind_speed(datagram, &knots_times_10, &display_in_metric);
	set_apparent_wind_speed_in_knots_times_10(knots_times_10);
}

void update_water_speed(unsigned char* datagram) {
	short int knots_times_10;
	parse_water_speed(datagram, &knots_times_10);
	set_water_speed_in_knots_times_100(knots_times_10 * 10);
}

void update_trip_mileage(unsigned char* datagram) {
	unsigned long int miles_times_100;
	parse_trip_mileage(datagram, &miles_times_100);
	set_trip_mileage_in_nautical_miles_times_100(miles_times_100);
}

void update_total_mileage(unsigned char* datagram) {
	unsigned long int miles_times_10;
	parse_total_mileage(datagram, &miles_times_10);
	set_total_mileage_in_nautical_miles_times_10(miles_times_10);
}

void update_water_temperature(unsigned char* datagram) {
	char celcius;
	bool transducer_defective;
	parse_water_temperature(datagram, &celcius, &transducer_defective);
	set_water_temperature_in_degrees_celsius_times_10(celcius*10);
}

void update_speed_distance_units(unsigned char* datagram) {
	DISTANCE_UNITS distance_units;
	parse_speed_distance_units(datagram, &distance_units);
	//  set_speed_distance_units(units);
}

void update_total_and_trip_mileage(unsigned char* datagram) {
	unsigned long int total_mileage_times_10, trip_mileage_times_100;
	parse_total_and_trip_mileage(datagram, &total_mileage_times_10, &trip_mileage_times_100);
	set_trip_mileage_in_nautical_miles_times_100(trip_mileage_times_100);
	set_total_mileage_in_nautical_miles_times_10(total_mileage_times_10);
}

void update_average_water_speed(unsigned char* datagram) {
	short int knots_1_times_100, knots_2_times_100, speed_1_from_sensor, speed_2_is_average, average_is_stopped;
	bool units;
	parse_average_water_speed(datagram, &knots_1_times_100, &knots_2_times_100, &speed_1_from_sensor, &speed_2_is_average, &average_is_stopped, &units);
	// this just assumes speed_2 is average
	set_average_water_speed_in_knots_times_100(knots_2_times_100);
}

void update_precise_water_temperature(unsigned char* datagram) {
	short int celsius_times_10;
	parse_precise_water_temperature(datagram, &celsius_times_10);
	set_water_temperature_in_degrees_celsius_times_10(celsius_times_10);
}

void update_lamp_intensity(unsigned char* datagram) {
	unsigned char level;
	parse_lamp_intensity(datagram, &level);
	//  set_lamp_intensity(level);
}

void accept_cancel_mob(unsigned char* datagram) {
	//  set_mob(0);
}

void update_lat_position(unsigned char* datagram) {
	unsigned char degrees;
	unsigned short minutes_times_100;
	LATITUDE_HEMISPHERE hemisphere;
	parse_lat_position(datagram, &hemisphere, &degrees, &minutes_times_100);
	set_position_latitude(hemisphere, degrees, minutes_times_100 * 10);
}

void update_lon_position(unsigned char* datagram) {
	unsigned char degrees;
	unsigned short minutes_times_100;
	LONGITUDE_HEMISPHERE hemisphere;
	parse_lon_position(datagram, &hemisphere, &degrees, &minutes_times_100);
	set_position_longitude(hemisphere, degrees, minutes_times_100 * 10);
}

void update_speed_over_ground(unsigned char* datagram) {
	unsigned short int knots_times_10;
	parse_speed_over_ground(datagram, &knots_times_10);
	set_speed_over_ground_in_knots_times_100(knots_times_10 * 10);
}

void update_course_over_ground(unsigned char* datagram) {
	short int true_degrees;
	parse_course_over_ground(datagram, &true_degrees);
	set_course_over_ground(true_degrees);
}

void update_gmt_time(unsigned char* datagram) {
	unsigned char hours =0, minutes=0, seconds=0;
	parse_gmt_time(datagram, &hours, &minutes, &seconds);
	set_gmt_time(hours, minutes, seconds);
}

void update_date(unsigned char* datagram) {
	unsigned short int year;
	unsigned char month, day;
	parse_date(datagram, &year, &month, &day);
	set_gmt_date(year, month, day);
}

void update_satellite_info(unsigned char* datagram) {
	unsigned char satellite_count, horizontal_dilution_of_position;
	parse_satellite_info(datagram, &satellite_count, &horizontal_dilution_of_position);
	set_gps_fix_satellite_count(satellite_count, 0);
	set_gps_fix_position_error(horizontal_dilution_of_position);
}

void update_lat_lon_position(unsigned char* datagram) {
	unsigned char lat, lon;
	unsigned short int minutes_lat_times_1000, minutes_lon_times_1000;
	LATITUDE_HEMISPHERE lat_hemisphere;
	LONGITUDE_HEMISPHERE lon_hemisphere;
	parse_lat_lon_position(datagram, &lat_hemisphere, &lat, &minutes_lat_times_1000, &lon_hemisphere, &lon, &minutes_lon_times_1000);
	set_position_latitude(lat_hemisphere, lat, minutes_lat_times_1000);
	set_position_longitude(lon_hemisphere, lon, minutes_lon_times_1000);
}

void update_countdown_timer(unsigned char* datagram) {
	unsigned char hours, minutes, seconds;
	TIMER_MODE mode;
	parse_countdown_timer(datagram, &hours, &minutes, &seconds, &mode);
	set_countdown_timer(hours, minutes, seconds, mode);
}

void update_wind_alarm(unsigned char* datagram) {
	unsigned short int active_alarms, current_active_alarms;
	parse_wind_alarm(datagram, &active_alarms);
	if (active_alarms) {
		if (get_active_alarms(&current_active_alarms) != 0) {
			current_active_alarms = 0;
		}
		set_active_alarms(current_active_alarms | active_alarms);
	}
}

void accept_alarm_acknowledgement(unsigned char* datagram) {
	short int acknowledged_alarm;
	parse_alarm_acknowledgement(datagram, &acknowledged_alarm);
	//  cancel_alarm(shallow_water, deep_water, anchor, true_wind_high, true_wind_low, true_wind_angle_high, true_wind_angle_low, apparent_wind_high, apparent_wind_low, apparent_wind_angle_high, apparent_win$
}

void accept_set_mob(unsigned char* datagram) {
	//  set_mob(1);
}

void accept_maxview_keystroke(unsigned char* datagram) {
}

void update_target_waypoint_name(unsigned char* datagram) {
	char char_1, char_2, char_3, char_4;
	char s[5];
	if (parse_target_waypoint_name(datagram, &char_1, &char_2, &char_3, &char_4) == 0) {
		s[0] = char_1;
		s[1] = char_2;
		s[2] = char_3;
		s[3] = char_4;
		s[4] = 0;
		//    set_target_waypoint_name(s);
	}
}

void update_course_computer_failure(unsigned char* datagram) {
	COURSE_COMPUTER_FAILURE_TYPE failure_type;
	parse_course_computer_failure(datagram, &failure_type);
	//  set_course_computer_failure_type(failure_type);
}

void update_autopilot_status(unsigned char* datagram) {
	short int compass_heading, target_heading;
	unsigned short int alarms;
	unsigned char display_flags;
	short int rudder_position;
	TURN_DIRECTION turning_direction;
	AUTOPILOT_MODE mode;
	unsigned short int current_active_alarms;
	parse_autopilot_status(datagram, &compass_heading, &turning_direction, &target_heading, &mode, &rudder_position, &alarms, &display_flags);
	set_heading(compass_heading);
	set_turn_direction(turning_direction);
	set_autopilot_target_heading(target_heading);
	set_autopilot_mode(mode);
	set_rudder_position_in_degrees_right(rudder_position);
	if (alarms) {
		if (get_active_alarms(&current_active_alarms) != 0) {
			current_active_alarms = 0;
		}
		set_active_alarms(current_active_alarms | alarms);
	}
}

void update_waypoint_navigation(unsigned char* datagram) {
	bool cross_track_error_present, waypoint_bearing_present, waypoint_distance_present, bearing_is_magnetic;
	unsigned short int cross_track_error_times_100, waypoint_distance_times_100;
	short int waypoint_bearing, direction_to_steer;
	if (parse_waypoint_navigation(datagram, &cross_track_error_present, &cross_track_error_times_100, &waypoint_bearing_present, &waypoint_bearing, &bearing_is_magnetic, &waypoint_distance_present, &waypoint_distance_times_100, &direction_to_steer) != 0) {
		return;
	}
	if (cross_track_error_present) {
		set_navigation_cross_track_error_in_nautical_miles_times_100(cross_track_error_times_100);
	}
	if (waypoint_bearing_present && waypoint_distance_present) {
		set_navigation_waypoint_bearing_and_range_in_nautical_miles_times_100(waypoint_bearing, waypoint_distance_times_100);
		set_navigation_waypoint_bearing_reference(bearing_is_magnetic ? ANGLE_REFERENCE_MAGNETIC : ANGLE_REFERENCE_TRUE);
	}
}

void accept_autopilot_command(unsigned char* datagram) {
	//  int button_1_value, button_2_value, pressed_longer, held_down, z101_remote;
	//  if (parse_autopilot_remote_control_keystroke(datagram, &button_1_value, &button_2_value, &pressed_longer, &held_down, &z101_remote) == 0) {
	//    if (!(pressed_longer || held_down)) {
	//      if (button_2_value) { // single key pressed
	//        if (button_1_value < 100) { // direction command key pressed
	//          change_autopilot_heading(button_1_value);
	//        } else {
	//          switch (button_1_value) {
	//            case 101: // auto
	//              set_autopilot_mode(
	//      }
	//      if (button_1_value && button_2_value) { // key combination
	//      }
	//    }
	//  }
}

void accept_set_autopilot_response_level(unsigned char* datagram) {
	AUTOPILOT_RESPONSE_LEVEL response_level;
	parse_set_autopilot_response_level(datagram, &response_level);
	//  set_autopilot_response_level(response_level);
}

void update_autopilot_parameter(unsigned char* datagram) {
	unsigned char parameter, min_value, max_value, value;
	parse_autopilot_parameter(datagram, &parameter, &min_value, &max_value, &value);
	//  set_autopilot_parameter(parameter, min_value, max_value, value);
}

void update_heading(unsigned char* datagram) {
	short int heading, locked_heading;
	bool locked_heading_active;
	parse_heading(datagram, &heading, &locked_heading_active, &locked_heading);
	set_heading(heading);
	//, locked_heading_active, locked_heading);
}

void accept_set_rudder_gain(unsigned char* datagram) {
	unsigned char rudder_gain;
	parse_set_rudder_gain(datagram, &rudder_gain);
	//  set_rudder_gain(rudder_gain);
}

//void accept_set_autopilot_parameter(unsigned char* datagram) {
//	int parameter, value;
//	parse_set_autopilot_parameter(datagram, &parameter, &value);
//	// not doing anything with this command
//}

//void accept_enter_autopilot_setup(unsigned char* datagram) {
//	parse_enter_autopilot_setup(datagram);
//}

void update_compass_variation(unsigned char* datagram) {
	short int compass_variation;
	parse_compass_variation(datagram, &compass_variation);
	set_compass_variation_in_degrees_west(compass_variation);
}

void update_heading_and_rudder_position(unsigned char* datagram) {
	short int heading;
	short int rudder_position;
	TURN_DIRECTION turning_direction;
	parse_heading_and_rudder_position(datagram, &heading, &turning_direction, &rudder_position);
	set_heading(heading);
	set_turn_direction(turning_direction);
	set_rudder_position_in_degrees_right(rudder_position);
}

//void update_destination_waypoint_info(unsigned char* datagram) {
//	// todo
//}
//
//void update_arrival_info(unsigned char* datagram) {
//	bool perpendicular_passed, circle_entered;
//	char char_1, char_2, char_3, char_4;
//	char name[5];
//	parse_arrival_info(datagram, &perpendicular_passed, &circle_entered, &char_1, &char_2, &char_3, &char_4);
//	name[0] = char_1;
//	name[1] = char_2;
//	name[2] = char_3;
//	name[3] = char_4;
//	name[4] = 0;
//	set_arrival_info(perpendicular_passed, circle_entered, name);
//}

void update_gps_and_dgps_fix_info(unsigned char* datagram) {
	short int signal_quality_available, signal_quality, hdop_available, hdop, antenna_height, satellite_count_available, satellite_count, geoseparation, dgps_age_available, dgps_age, dgps_station_id_available, dgps_station_id;
	parse_gps_and_dgps_fix_info(datagram, &signal_quality_available, &signal_quality, &hdop_available, &hdop, &antenna_height, &satellite_count_available, &satellite_count, &geoseparation, &dgps_age_available, &dgps_age, &dgps_station_id_available, &dgps_station_id);
	if (signal_quality_available) {
		set_gps_fix_signal_quality(signal_quality);
	}
	if (hdop_available) {
		set_gps_fix_position_error(hdop);
	}
	set_gps_fix_antenna_height(antenna_height);
	if (satellite_count_available) {
		set_gps_fix_satellite_count(satellite_count, geoseparation);
	}
	if (dgps_age_available) {
		set_gps_fix_dgps_age(dgps_age);
	}
	if (dgps_station_id_available) {
		set_gps_fix_dgps_station_id(dgps_station_id);
	}
}

void update_seatalk_state(unsigned char* datagram) {
	switch (datagram[0]) {
	case 0x00: // depth below transducer
		update_depth_below_transducer(datagram);
		break;
	case 0x01: // equipment ID
		break;
	case 0x05: // engine RPM and blade pitch
		update_engine_rpm_and_pitch(datagram);
		break;
	case 0x10: // apparent wind angle
		update_apparent_wind_angle(datagram);
		break;
	case 0x11: // apparent wind speed
		update_apparent_wind_speed(datagram);
		break;
	case 0x20: // water speed xxxx/10 kt
		update_water_speed(datagram);
		break;
	case 0x21: // trip mileage xxxxx/100 nm
		update_trip_mileage(datagram);
		break;
	case 0x22: // trip mileage xxxx/10 nm
		update_total_mileage(datagram);
		break;
	case 0x23: // water temperature
		update_water_temperature(datagram);
		break;
	case 0x24: // display units for mileage & speed
		update_speed_distance_units(datagram);
		break;
	case 0x25: // total & trip log
		update_total_and_trip_mileage(datagram);
		break;
	case 0x26: // water speed with average
		update_average_water_speed(datagram);
		break;
	case 0x27: // water teperature (xxxx-100)/10 degrees C break;
		update_precise_water_temperature(datagram);
		break;
	case 0x30: // set lamp intensity
		update_lamp_intensity(datagram);
		break;
	case 0x36: // cancel MOB
		accept_cancel_mob(datagram);
		break;
	case 0x38: // codelock data
		break;
	case 0x50: // lat position
		update_lat_position(datagram);
		break;
	case 0x51: // lon position
		update_lon_position(datagram);
		break;
	case 0x52: // speed over ground
		update_speed_over_ground(datagram);
		break;
	case 0x53: // course over ground
		update_course_over_ground(datagram);
		break;
	case 0x54: // gmt time
		update_gmt_time(datagram);
		break;
	case 0x55: // TRACK keystroke on GPS unit
		// accept_autopilot_remote_control_keystroke(datagram);
		break;
	case 0x56: // date
		update_date(datagram);
		break;
	case 0x57: // satellite status
		update_satellite_info(datagram);
		break;
	case 0x58: // lat/lon
		update_lat_lon_position(datagram);
		break;
	case 0x59: // set countdown timer
		update_countdown_timer(datagram);
		break;
	case 0x61: // e-80 unit initialization broadcast
		break;
	case 0x65: // select fathom or feed depth display units
		break;
	case 0x66: // wind alarm
		update_wind_alarm(datagram);
		break;
	case 0x68: // alarm acknowledgement keystroke
		accept_alarm_acknowledgement(datagram);
		break;
	case 0x6c: // second equipment id message
		break;
	case 0x6e: // MOB keystroke
		accept_set_mob(datagram);
		break;
	case 0x70: // ST60 remote control keystroke
		accept_maxview_keystroke(datagram);
		break;
	case 0x80: // set lamp intensity
		update_lamp_intensity(datagram); // same as 0x30
		break;
	case 0x81: // set by course computer during setup when going past user cal
		break;
	case 0x82: // target waypoint name
		break;
	case 0x83: // course computer powered up or cleared failure condition
		break;
	case 0x84: // autopilot status broadcast
		update_autopilot_status(datagram);
		break;
	case 0x85: // navigation to waypoint status
		update_waypoint_navigation(datagram);
		break;
	case 0x86: // autopilot command
		// accept_autopilot_remote_control_keystroke(datagram);
		break;
	case 0x87: // set autopilot response level
		accept_set_autopilot_response_level(datagram);
		break;
	case 0x88: // autopilot parameter
		update_autopilot_parameter(datagram);
		break;
	case 0x89: // steering compass course
		update_heading(datagram);
		break;
	case 0x90: // device identification
		break;
	case 0x91: // set rudder gain
		accept_set_rudder_gain(datagram);
		break;
	//case 0x92: // set autopilot parameter
	//	accept_set_autopilot_parameter(datagram);
	//	break;
	//case 0x93: // enter autopilot setup
	//	accept_enter_autopilot_setup(datagram);
	//	break;
	case 0x95: // replaces 84 while autpilot is in value setting mode
		break;
	case 0x99: // compass variation sent by st40 or autopilot
		update_compass_variation(datagram);
		break;
	case 0x9a: // version string
		break;
	case 0x9c: // compass heading and rudder position
		update_heading_and_rudder_position(datagram);
		break;
	case 0x9e: // waypoint definition
		break;
	case 0xa1: // destination waypoint info
		break;
	//case 0xa2: // waypoint arrival notice
	//	update_arrival_info(datagram);
	//	break;
	case 0xa4: // broadcast query to identify all devices on the bus
		break;
	case 0xa5: // GPS and DGPS info
		update_gps_and_dgps_fix_info(datagram);
		break;
	case 0xa7: // unknown
		break;
	case 0xa8: // alarm notice for guard #1 or guard #2
		break;
	case 0xab: // alarm notice for guard #1 or guard #2
		break;
	}
}

void handle_seatalk_datagram(unsigned char* datagram) {
	update_seatalk_state(datagram);
}
