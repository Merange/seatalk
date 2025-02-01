#include "seatalk_datagram.h"

#define INITIALIZE_DATAGRAM(DATAGRAM_NUMBER, HIGH_NIBBLE_OF_SECOND_BYTE) initialize_datagram(datagram, 0x##DATAGRAM_NUMBER, DATAGRAM_##DATAGRAM_NUMBER##_LENGTH, HIGH_NIBBLE_OF_SECOND_BYTE)


static unsigned char first_nibble(unsigned char c) {
	return (c & 0xf0) >> 4;
}

static unsigned char last_nibble(unsigned char c) {
	return (c & 0x0f);
}

static unsigned short int from_two_bytes(unsigned char* datagram, unsigned short int index) {
	unsigned char byte1, byte2;
	byte1 = datagram[index];
	byte2 = datagram[index + 1];
	return (byte2 << 8) | byte1;
}

static void to_two_bytes(unsigned short int value, unsigned char* datagram, unsigned short int index) {
	datagram[index] = value & 0xFF;
	datagram[index + 1] = value >> 8;
}

unsigned char complement_checksum(unsigned char value) {
	return value ^ 0xff;
}

short int fix_twos_complement_char(unsigned char value) {
	if (value & 0x80) {
		return -1 * ((value ^ 0xff) + 1);
	}
	else {
		return value;
	}
}

static short int fix_twos_complement_word(short int value) {
	if (value & 0x8000) {
		return -1 * ((value ^ 0xffff) + 1);
	}
	else {
		return value;
	}
}

bool flag(unsigned short int flags, unsigned short int mask) {
	return (flags & mask) != 0;
}

unsigned char get_datagram_length(char second_byte) {
	return last_nibble(second_byte);
}

static void initialize_datagram(unsigned char* datagram, unsigned char datagram_number, short int total_length, short int high_nibble_of_second_byte) {
	int i;
	datagram[0] = datagram_number;
	datagram[1] = (total_length - 3) + (high_nibble_of_second_byte << 4);
	for (i = 2; i < total_length; i++) { datagram[i] = 0; }
}

static void uvw_compass(unsigned char* u, unsigned char* vw, short int compass) {
	short int temp_compass = compass;
	*u = (unsigned char)(temp_compass / 90);
	temp_compass = temp_compass - (*u * 90);
	*vw = (unsigned char)(temp_compass / 2);
	temp_compass -= *vw * 2;
	*u |= (unsigned char)((temp_compass * 2) << 2);
}

static void uvw_heading_and_turning_direction(short int heading, TURN_DIRECTION turning_direction, unsigned char* u, unsigned char* vw) {
	short int temp_direction, odd;
	temp_direction = heading;
	odd = temp_direction % 2;
	*u = (unsigned char)turning_direction == TURN_DIRECTION_RIGHT ? 0x8 : 0;
	if (odd) {
		if (*u & 0x8) {
			// high bit of u adds 1 to heading so nothing to do
		}
		else {
			*u |= 0x4;
		}
	}
	else {
		if (*u & 0x8) {
			// high bit of u adds 1 to heading so need to reduce vw component
			if (temp_direction == 0) {
				temp_direction = 359;
			}
			else {
				temp_direction -= 1;
			}
			*u |= 0x4;
		}
		else {
			// heading reflected accurately with no extra from u
		}
	}
	*u |= (unsigned char)(temp_direction / 90);
	temp_direction = temp_direction % 90;
	*vw = (unsigned char)(temp_direction / 2);
}

static int alarm_is_active(int active_alarms, short int alarm) {
	return (active_alarms & alarm) ? 0xffff : 0;
}

unsigned char build_depth_below_transducer(unsigned char* datagram, short int depth_in_feet_times_10, bool display_in_metres, unsigned short int active_alarms, bool transducer_defective) {
	// 00 02 yz xx xx
	// depth below transducer (in feet): xxxx/10
	// anchor alarm: y & 0x8
	// metric display: y & 4
	// deep water alarm: z & 2
	// shallow water alarm: z & 1
	short int xxxx = depth_in_feet_times_10;
	unsigned char y = (alarm_is_active(active_alarms, ALARM_ANCHOR) & 0x8) | (display_in_metres ? 0x4 : 0x0);
	unsigned char z = (alarm_is_active(active_alarms, ALARM_DEEP_WATER) & 0x2) | (alarm_is_active(active_alarms, ALARM_SHALLOW_WATER) & 0x1) | (transducer_defective ? 0x4 : 0);;
	INITIALIZE_DATAGRAM(00, 0);
	datagram[2] = (y << 4) | z;
	to_two_bytes(xxxx, datagram, 3);
	return DATAGRAM_00_LENGTH;
}

void parse_depth_below_transducer(unsigned char* datagram, short int* depth_in_feet_times_10, bool* display_in_metres, unsigned short int* active_alarms, bool* transducer_defective) {
	short int y, z, xxxx;
	y = first_nibble(datagram[2]);
	z = last_nibble(datagram[2]);
	xxxx = from_two_bytes(datagram, 3);
	*depth_in_feet_times_10 = xxxx;
	*display_in_metres = flag(y, 0x4);
	*active_alarms = 0;
	*active_alarms |= flag(z, 0x1) ? ALARM_SHALLOW_WATER : 0;
	*active_alarms |= flag(z, 0x2) ? ALARM_DEEP_WATER : 0;
	*active_alarms |= flag(y, 0x8) ? ALARM_ANCHOR : 0;
	*transducer_defective = flag(z, 0x4);
}

unsigned char build_engine_rpm_and_pitch(unsigned char* datagram, ENGINE_ID engine_id, short int rpm, short int  pitch_percent) {
	// 05 03 0x yy zz pp
	// x = 0: single engine (supply starboard_indicator as -1)
	// x = 1: starboard engine (supply staboard_indicator as 1)
	// x = 2: port engine (supply starboard_indicator as 0)
	// rpm: yy * 256 + zz (signed value)
	// pp: prop pitch, signed value
	unsigned char x, pp;
	switch (engine_id) {
	case ENGINE_ID_PORT:
		x = 2;
		break;
	case ENGINE_ID_STARBOARD:
		x = 1;
		break;
	default:
		x = 0;
	}
	pp = (unsigned char) pitch_percent;
	INITIALIZE_DATAGRAM(05, 0);
	datagram[2] = x;
	to_two_bytes(rpm, datagram, 3);
	datagram[5] = pp;
	return DATAGRAM_05_LENGTH;
}

void parse_engine_rpm_and_pitch(unsigned char* datagram, ENGINE_ID* engine_id, short int* rpm, short int * pitch_percent) {
	unsigned char x, pp;
	short int yyzz;
	x = datagram[2];
	yyzz = from_two_bytes(datagram, 3);
	pp = datagram[5];
	switch (x) {
	case 1:
		*engine_id = ENGINE_ID_STARBOARD;
		break;
	case 2:
		*engine_id = ENGINE_ID_PORT;
		break;
	default:
		*engine_id = ENGINE_ID_SINGLE;
	}
	*rpm = yyzz;
	*pitch_percent = fix_twos_complement_char(pp);
}

unsigned char build_apparent_wind_angle(unsigned char* datagram, short int degrees_right_times_2) {
	// 10 01 xx yy
	// xxyy/2 = degrees right of bow
  // byte order : MSB-LSB not LSB-MSB like most of the other multibytes information
	degrees_right_times_2 %= 720;
	if (degrees_right_times_2 < 0) {
		degrees_right_times_2 += 720;
	}
	unsigned char xx = (unsigned char)(degrees_right_times_2 >> 8);
	unsigned char yy = (unsigned char)(degrees_right_times_2 & 0xff);
	INITIALIZE_DATAGRAM(10, 0);
	datagram[2] = xx;
	datagram[3] = yy;
	return DATAGRAM_10_LENGTH;
}

void parse_apparent_wind_angle(unsigned char* datagram, short int* degrees_right_times_2) {
	short int xxyy;
	xxyy = (unsigned short int) (datagram[2] << 8) + datagram[3];
	if (xxyy > 360) {
		xxyy -= 720;
	}
	*degrees_right_times_2 = xxyy;
}

unsigned char build_apparent_wind_speed(unsigned char* datagram, unsigned short int knots_times_10, bool display_in_metric) {
	// 11 01 xx 0y
	// wind speed: (xx & 0x7f) + y/10
	// units flag xx & 0x80; 0 knots/0x80 m/s
	unsigned char xx, y;
	xx = (unsigned char)(knots_times_10 / 10);
	y = (unsigned char)(knots_times_10 % 10);
	if (xx > 0x7f) {
		xx = 0x7f;
	}
	if (display_in_metric) {
		xx |= 0x80;
	}
	INITIALIZE_DATAGRAM(11, 0);
	datagram[2] = xx;
	datagram[3] = y;
	return DATAGRAM_11_LENGTH;
}

void parse_apparent_wind_speed(unsigned char* datagram, unsigned short int* knots_times_10, bool* display_in_metric) {
	short int xx, y;
	xx = datagram[2];
	y = last_nibble(datagram[3]);
	*knots_times_10 = (xx & 0x7f) * 10 + y;
	*display_in_metric = flag(xx, 0x80);
}

unsigned char build_water_speed(unsigned char* datagram, short int knots_times_10) {
	// 20 01 xx xx
	// water speed = xxxx/10 knots
	INITIALIZE_DATAGRAM(20, 0);
	to_two_bytes(knots_times_10, datagram, 2);
	return DATAGRAM_20_LENGTH;
}

void parse_water_speed(unsigned char* datagram, short int* knots_times_10) {
	short int xxxx;
	xxxx = from_two_bytes(datagram, 2);
	*knots_times_10 = xxxx;
}

unsigned char build_trip_mileage(unsigned char* datagram, unsigned long int nmiles_times_100) {
	// 21 02 xx xx 0x
	// mileage = xxxxx/100 nautical  miles
	unsigned char xx1;
	xx1 = (unsigned char)(nmiles_times_100 >> 16);
	INITIALIZE_DATAGRAM(21, 0);
	to_two_bytes(nmiles_times_100 & 0xFFFF, datagram, 2);
	datagram[4] = xx1;
	return DATAGRAM_21_LENGTH;
}

void parse_trip_mileage(unsigned char* datagram, unsigned long int* miles_times_100) {
	unsigned long int xxxxx;
	xxxxx = from_two_bytes(datagram, 2) | (unsigned long int)last_nibble(datagram[4]) << 16;
	*miles_times_100 = xxxxx;
}

unsigned char build_total_mileage(unsigned char* datagram, unsigned long int nmiles_times_10) {
	// 22 02 x xx 0x
	// mileage = xxxx/10 nautical  miles
	INITIALIZE_DATAGRAM(22, 0);
	to_two_bytes((unsigned short)nmiles_times_10, datagram, 2);
	datagram[4] = 0;
	return DATAGRAM_22_LENGTH;
}

void parse_total_mileage(unsigned char* datagram, unsigned long int* miles_times_10) {
	unsigned long int xxxx;
	xxxx = from_two_bytes(datagram, 2);
	*miles_times_10 = xxxx;
}

unsigned char build_water_temperature(unsigned char* datagram, char degrees_celcius, bool transducer_defective) {
	// 23 z1 xx yy
	// z & 0x4 sensor problem
	// xx degrees Celsius, yy degrees Fahrenheit from -80°c to +50°c (Farenheit overflow if outside this bounds)
	char xx, yy;
	unsigned char z;
	xx = (char)degrees_celcius;
	yy = (int) (((float)degrees_celcius * 9.0 / 5) + 32);
	z = transducer_defective ? 0x4 : 0;
	INITIALIZE_DATAGRAM(23, z);
	datagram[2] = xx;
	datagram[3] = yy;
	return DATAGRAM_23_LENGTH;
}

void parse_water_temperature(unsigned char* datagram, char* degrees_celcius, bool* transducer_defective) {
	char xx, z;
	xx = datagram[2];
	z = first_nibble(datagram[1]);
	*degrees_celcius = xx;
	*transducer_defective = flag(z, 0x4);
}

unsigned char build_speed_distance_units(unsigned char* datagram, DISTANCE_UNITS distance_units) {
	// 24 02 00 00 xx
	// xx == 00 nm/kts; 06 sm/mph; 86 km/kph
	unsigned char xx;
	switch (distance_units) {
	case DISTANCE_UNITS_NAUTICAL:
		xx = 0;
		break;
	case DISTANCE_UNITS_STATUTE:
		xx = 0x06;
		break;
	default:
		xx = 0x86;
		break;
	}
	INITIALIZE_DATAGRAM(24, 0);
	datagram[4] = xx;
	return DATAGRAM_24_LENGTH;
}

void parse_speed_distance_units(unsigned char* datagram, DISTANCE_UNITS* distance_units) {
	unsigned char  xx;
	xx = datagram[4];
	if (xx == 0) {
		*distance_units = DISTANCE_UNITS_NAUTICAL;
	}
	else if (xx == 6) {
		*distance_units = DISTANCE_UNITS_STATUTE;
	}
	else {
		*distance_units = DISTANCE_UNITS_METRIC;
	}
}

unsigned char build_total_and_trip_mileage(unsigned char* datagram, unsigned long int total_nm_times_10, unsigned long int trip_nm_times_100) {
	// 25 z4 xx yy uu vv aw
	// total = (z * 4096 + yy * 256 + xx) / 10 (max 104857.5 nm)
	// trip = (w * 65546 + vv * 256 + uu) / 100 (max 10485.85 nm)
	unsigned char z, w;
	z = (unsigned char)(total_nm_times_10 >> 16);
	if (z > 0x0f) {
		z = 0x0f;
	}
	INITIALIZE_DATAGRAM(25, z);
	to_two_bytes((unsigned short)total_nm_times_10, datagram, 2);
	w = (unsigned char)(trip_nm_times_100 >> 16);
	if (w > 0x0f) {
		w = 0x0f;
	}
	to_two_bytes((unsigned short)trip_nm_times_100, datagram, 4);
	datagram[6] = 0xa0 | w;
	return DATAGRAM_25_LENGTH;
}

void parse_total_and_trip_mileage(unsigned char* datagram, unsigned long int* total_mileage_times_10, unsigned long int* trip_mileage_times_100) {
	unsigned char xx, yy, uu, vv, z, w;
	xx = datagram[2];
	yy = datagram[3];
	uu = datagram[4];
	vv = datagram[5];
	z = first_nibble(datagram[1]);
	w = last_nibble(datagram[6]);
	*total_mileage_times_10 = ((unsigned long int)z << 16) | (yy << 8) | xx;
	*trip_mileage_times_100 = ((unsigned long int)w << 16) | (vv << 8) | uu;
}

unsigned char build_average_water_speed(unsigned char* datagram, short int knots_1_times_100, short int knots_2_times_100, short int speed_1_from_sensor, short int speed_2_is_average, short int average_is_stopped, bool display_in_statute_miles) {
	// 26 04 xx xx yy yy de
	// xxxx/100 = current speed (kts), valid if d & 0x4 == 4
	// yyyy/100 = average speed (kts), d & 0x8: 8 => sensor; 0 => trip log/time
	// e & 0x1 => calculation stopped
	// e & 0x2 => display in statute miles
	unsigned char xx1, xx2, yy1, yy2, d, e;
	xx2 = knots_1_times_100 >> 8;
	xx1 = knots_1_times_100 & 0xff;
	yy2 = knots_2_times_100 >> 8;
	yy1 = knots_2_times_100 & 0xff;
	d = 0;
	d |= (speed_1_from_sensor ? 0x4 : 0);
	d |= (speed_2_is_average ? 0x8 : 0);
	e = 0;
	e |= (average_is_stopped ? 1 : 0);
	e |= (display_in_statute_miles ? 2 : 0);
	INITIALIZE_DATAGRAM(26, 0);
	datagram[2] = xx1;
	datagram[3] = xx2;
	datagram[4] = yy1;
	datagram[5] = yy2;
	datagram[6] = (d << 4) | e;
	return DATAGRAM_26_LENGTH;
}

void parse_average_water_speed(unsigned char* datagram, short int* knots_1_times_100, short int* knots_2_times_100, short int* speed_1_from_sensor, short int* speed_2_is_average, short int* average_is_stopped, bool* display_in_statute_miles) {
	short int xxxx, yyyy, d, e;
	xxxx = from_two_bytes(datagram, 2);
	yyyy = from_two_bytes(datagram, 4);
	d = first_nibble(datagram[6]);
	e = last_nibble(datagram[6]);
	*knots_1_times_100 = xxxx;
	*knots_2_times_100 = yyyy;
	*speed_1_from_sensor = flag(d, 0x4);
	*speed_2_is_average = flag(d, 0x8);
	*average_is_stopped = flag(e, 0x1);
	*display_in_statute_miles = flag(e, 0x2);
}

unsigned char build_precise_water_temperature(unsigned char* datagram, short int degrees_celsius_times_10) {
	// 27 01 xx xx
	// (xxxx - 100) / 10 = temperature in Celsius
	unsigned short int temp_temperature;
	temp_temperature = degrees_celsius_times_10 + 100;
	INITIALIZE_DATAGRAM(27, 0);
	to_two_bytes(temp_temperature, datagram, 2);
	return DATAGRAM_27_LENGTH;
}

void parse_precise_water_temperature(unsigned char* datagram, short int* degrees_celsius_times_10) {
	short int xxxx;
	xxxx = from_two_bytes(datagram, 2);
	*degrees_celsius_times_10 = xxxx - 100;
}

unsigned char build_lamp_intensity(unsigned char* datagram, unsigned char intensity) {
	// 30 00 0x x value/intensity 0x0/0, 0x4/1, 0x8/2, 0xc/3
	unsigned char x;
	switch (intensity) {
	case 0:
		x = 0x0;
		break;
	case 1:
		x = 0x4;
		break;
	case 2:
		x = 0x8;
		break;
	default:
		x = 0xc;
	}
	INITIALIZE_DATAGRAM(30, 0);
	datagram[2] = x;
	return DATAGRAM_30_LENGTH;
}

void parse_lamp_intensity(unsigned char* datagram, unsigned char* level) {
	unsigned char x;
	x = last_nibble(datagram[2]);
	switch (x) {
	case 0x0:
		*level = 0;
		break;
	case 0x4:
		*level = 1;
		break;
	case 0x8:
		*level = 2;
		break;
	case 0xc:
		*level = 3;
		break;
	}
}

unsigned char build_cancel_mob(unsigned char* datagram) {
	// 36 00 01 no variation
	INITIALIZE_DATAGRAM(36, 0);
	datagram[2] = 1;
	return DATAGRAM_36_LENGTH;
}

unsigned char build_lat_position(unsigned char* datagram, LATITUDE_HEMISPHERE hemisphere, unsigned char degrees, unsigned short int minutes_times_100) {
	// use -ve degrees for southern hemisphere
	// 50 Z2 XX YY YY
	// XX degrees, (YYYY & 0x7fff)/100 minutes
	// YYYY & 0x8000 south if set, north if clear
	// Z = 0xa or 0x0 (Raystar 120 GPS, meaning unknown)
	unsigned char xx, z;
	unsigned short int yyyy;
	xx = degrees;
	yyyy = hemisphere == LATITUDE_HEMISPHERE_NORTH ? 0 : 0x8000;
	yyyy |= minutes_times_100;
	z = 0; // not impersonating a Raystar 120 so always 0
	INITIALIZE_DATAGRAM(50, z);
	datagram[2] = xx;
	to_two_bytes(yyyy, datagram, 3);
	return DATAGRAM_50_LENGTH;
}

void parse_lat_position(unsigned char* datagram, LATITUDE_HEMISPHERE* hemisphere, unsigned char* degrees, unsigned short int* minutes_times_100) {
	unsigned char xx = datagram[2];
	unsigned short int yyyy = from_two_bytes(datagram, 3);
	*hemisphere = flag(yyyy, 0x8000) ? LATITUDE_HEMISPHERE_SOUTH : LATITUDE_HEMISPHERE_NORTH;
	*degrees = xx;
	*minutes_times_100 = yyyy & 0x7fff;
}

unsigned char build_lon_position(unsigned char* datagram, LONGITUDE_HEMISPHERE hemisphere, unsigned char degrees, unsigned short int minutes_times_100) {
	// use -ve degrees for eastern hemisphere
	// 51 Z2 XX YY YY
	// XX degrees, (YYYY & 0x7fff)/100 minutes
	// YYYY & 0x8000 east if set, west if clear
	// Z = 0xa or 0x0 (Raystar 120 GPS, meaning unknown)
	unsigned char xx = degrees;
	unsigned short int yyyy = hemisphere == LONGITUDE_HEMISPHERE_WEST ? 0 : 0x8000;
	yyyy |= minutes_times_100;
	unsigned char z = 0; // not impersonaing a Raystar 120 so always 0)
	INITIALIZE_DATAGRAM(51, z);
	datagram[2] = xx;
	to_two_bytes(yyyy, datagram, 3);
	return DATAGRAM_51_LENGTH;
}

void parse_lon_position(unsigned char* datagram, LONGITUDE_HEMISPHERE* hemisphere, unsigned char* degrees, unsigned short int* minutes_times_100) {
	unsigned char xx = datagram[2];
	unsigned short int	yyyy = from_two_bytes(datagram, 3);
	*hemisphere = flag(yyyy, 0x8000) ? LONGITUDE_HEMISPHERE_EAST : LONGITUDE_HEMISPHERE_WEST;
	*degrees = xx;
	*minutes_times_100 = yyyy & 0x7fff;
}

unsigned char build_speed_over_ground(unsigned char* datagram, unsigned short int knots_times_10) {
	// 52 01 XX XX
	// XXXX/10 is speed over ground in kts
	short int xxxx = knots_times_10;
	INITIALIZE_DATAGRAM(52, 0);
	to_two_bytes(xxxx, datagram, 2);
	return DATAGRAM_52_LENGTH;
}

void parse_speed_over_ground(unsigned char* datagram, unsigned short int* knots_times_10) {
	short int xxxx;
	xxxx = from_two_bytes(datagram, 2);
	*knots_times_10 = xxxx;
}

unsigned char build_course_over_ground(unsigned char* datagram, short int true_degrees) {
	// 53 U0 VW
	// true course = (U & 0x3) * 90 + (VW & 0x3F) * 2 + (U & 0xC) >> 2
	unsigned char u, vw;
	uvw_compass(&u, &vw, true_degrees);
	INITIALIZE_DATAGRAM(53, u);
	datagram[2] = vw;
	return DATAGRAM_53_LENGTH;
}

void parse_course_over_ground(unsigned char* datagram, short int* true_degrees) {
	short int u, vw;
	u = first_nibble(datagram[1]);
	vw = datagram[2];
	*true_degrees = ((u & 0x3) * 90) + ((vw & 0x3f) * 2) + (u >> 3);
}

unsigned char build_gmt_time(unsigned char* datagram, unsigned char hours, unsigned char minutes, unsigned char seconds) {
	// 54, T1, RS, HH
	// HH = hours
	// 6 highest bits of RST = minutes = (RS & 0xfc) >> 2
	// 6 lowest bits of RST = seconds = ST & 0x3f
	unsigned char t, rs, hh;
	hh = hours;
	rs = minutes << 2;
	rs |= seconds >> 4;
	t = seconds & 0xf;
	INITIALIZE_DATAGRAM(54, t);
	datagram[2] = rs;
	datagram[3] = hh;
	return DATAGRAM_54_LENGTH;
}

void parse_gmt_time(unsigned char* datagram, unsigned char* hours, unsigned char* minutes, unsigned char* seconds) {
	unsigned char t, rs, hh;
	t = first_nibble(datagram[1]);
	rs = datagram[2];
	hh = datagram[3];
	*hours = hh;
	*minutes = rs >> 2;
	*seconds = ((rs & 0x3) << 4) | t;
}

unsigned char build_date(unsigned char* datagram, unsigned short int year, unsigned char month, unsigned char day) {
	// 56 M1 DD YY
	// YY year M month D day of month
	unsigned char yy, m, dd;
	yy = (unsigned char)year - 2000; // arbitrary epoch offset until real data available
	m = month;
	dd = day;
	INITIALIZE_DATAGRAM(56, m);
	datagram[2] = dd;
	datagram[3] = yy;
	return DATAGRAM_56_LENGTH;
}

void parse_date(unsigned char* datagram, unsigned short int* year, unsigned char* month, unsigned char* day) {
	unsigned char m, dd;
	m = first_nibble(datagram[1]);
	dd = datagram[2];
	*year = fix_twos_complement_char (datagram[3]) + 2000; // arbitrary epoch offset until real data available;
	*month = m;
	*day = dd;
}

unsigned char build_satellite_info(unsigned char* datagram, unsigned char satellite_count, unsigned char horizontal_dilution_of_position) {
	// 58 z5 la xx yy lo qq rr
	// lat/lon position
	// la, lo degrees latitude, longitude
	// minutes lat xxyy/1000
	// minutes lon qqrr/1000
	// z: south if z&1
	//    east if z&2
	unsigned char s, dd;
	s = satellite_count;
	dd = horizontal_dilution_of_position;
	INITIALIZE_DATAGRAM(57, s);
	datagram[2] = dd;
	return DATAGRAM_57_LENGTH;
}

void parse_satellite_info(unsigned char* datagram, unsigned char* satellite_count, unsigned char* horizontal_dilution_of_position) {
	unsigned char s, dd;
	s = first_nibble(datagram[1]);
	dd = datagram[2];
	*satellite_count = s;
	*horizontal_dilution_of_position = dd;
}

unsigned char build_lat_lon_position(unsigned char* datagram, LATITUDE_HEMISPHERE hemisphere_latitude, unsigned char degrees_lat, unsigned short int minutes_lat_times_1000, LONGITUDE_HEMISPHERE hemisphere_longitude, unsigned char  degrees_lon, unsigned short int minutes_lon_times_1000) {
	unsigned char z, la, /*xx, yy,*/ lo/*, qq, rr*/;
	z = 0;
	if (hemisphere_latitude == LATITUDE_HEMISPHERE_SOUTH) {
		z |= 0x1;
	}
	la = degrees_lat;
	// xx = minutes_lat_times_1000 >> 8;
	// yy = minutes_lat_times_1000 & 0xff;
	if (hemisphere_longitude == LONGITUDE_HEMISPHERE_EAST) {
		z |= 0x02;
	}
	lo = degrees_lon;
	// qq = minutes_lon_times_1000 >> 8;
	// rr = minutes_lon_times_1000 & 0xff;
	INITIALIZE_DATAGRAM(58, z);
	datagram[2] = la;
	to_two_bytes(minutes_lat_times_1000, datagram, 3);
	// datagram[3] = xx;
	// datagram[4] = yy;
	datagram[5] = lo;
	to_two_bytes(minutes_lon_times_1000, datagram, 6);
	// datagram[6] = rr;
	// datagram[7] = qq;
	return DATAGRAM_58_LENGTH;
}

void parse_lat_lon_position(unsigned char* datagram, LATITUDE_HEMISPHERE* hemisphere_latitude, unsigned char* degrees_lat, unsigned short int* minutes_lat_times_1000, LONGITUDE_HEMISPHERE* hemisphere_longitude, unsigned char* degrees_lon, unsigned short int* minutes_lon_times_1000) {
	unsigned char z, la, lo;
	unsigned short int xxyy, qqrr;
	z = first_nibble(datagram[1]);
	la = datagram[2];
	xxyy = from_two_bytes(datagram, 3);
	lo = datagram[5];
	qqrr = from_two_bytes(datagram, 6);
	*hemisphere_latitude = flag(z, 0x1) ? LATITUDE_HEMISPHERE_SOUTH : LATITUDE_HEMISPHERE_NORTH;
	*degrees_lat = la;
	*minutes_lat_times_1000 = xxyy;
	*hemisphere_longitude = flag(z, 0x2) ? LONGITUDE_HEMISPHERE_EAST : LONGITUDE_HEMISPHERE_WEST;
	*degrees_lon = lo;
	*minutes_lon_times_1000 = qqrr;
}

unsigned char build_countdown_timer(unsigned char* datagram, unsigned char hours, unsigned char minutes, unsigned char seconds, TIMER_MODE mode) {
	// 59 22 ss mm xh
	// set countdown timer
	// mm minutes, ss seconds, h hours
	// msb mm count up start flag
	// x: counter mode
	// 0 = count up and start
	// 4 = count down
	// 8 = count down and start
	unsigned char ss, mm, x, h;
	h = hours;
	mm = minutes;
	ss = seconds;
	switch (mode) {
	case TIMER_MODE_COUNT_DOWN:
		x = 4;
		break;
	case TIMER_MODE_COUNT_UP_AND_START:
		x = 0;
		mm |= 0x80;
		break;
	case TIMER_MODE_COUNT_DOWN_AND_START:
		x = 8;
		break;
	default:
		x = 0;
	}
	INITIALIZE_DATAGRAM(59, 2);
	datagram[2] = ss;
	datagram[3] = mm;
	datagram[4] = (x << 4) | h;
	return DATAGRAM_59_LENGTH;
}

void parse_countdown_timer(unsigned char* datagram, unsigned char* hours, unsigned char* minutes, unsigned char* seconds, TIMER_MODE* mode) {
	unsigned char ss, mm, x, h;
	ss = datagram[2];
	mm = datagram[3];
	x = first_nibble(datagram[4]);
	h = last_nibble(datagram[4]);
	*hours = h;
	*minutes = mm & 0x7f;
	*seconds = ss;
	if (x == 0) {
		if (mm & 0x80) {
			*mode = TIMER_MODE_COUNT_UP_AND_START;
		}
		else {
			*mode = TIMER_MODE_COUNT_UP;
		}
	}
	else if (x == 4) {
		*mode = TIMER_MODE_COUNT_DOWN;
	}
	else if (x == 8) {
		*mode = TIMER_MODE_COUNT_DOWN_AND_START;
	}
}

unsigned char build_wind_alarm(unsigned char* datagram, unsigned short int active_alarms) {
	// 66 00 xy
	// x: apparent wind alarm; y: true wind
	// bits: & 0x8 angle low, 0x4 angle high, 0x2 speed low, 0x1 speed high
	// xy = 0: end all wind alarms
	short int x, y;
	x = 0;
	y = 0;
	x |= alarm_is_active(active_alarms, ALARM_APPARENT_WIND_ANGLE_LOW) & 0x8;
	x |= alarm_is_active(active_alarms, ALARM_APPARENT_WIND_ANGLE_HIGH) & 0x4;
	x |= alarm_is_active(active_alarms, ALARM_APPARENT_WIND_SPEED_LOW) & 0x2;
	x |= alarm_is_active(active_alarms, ALARM_APPARENT_WIND_SPEED_HIGH) & 0x1;
	y |= alarm_is_active(active_alarms, ALARM_TRUE_WIND_ANGLE_LOW) & 0x8;
	y |= alarm_is_active(active_alarms, ALARM_TRUE_WIND_ANGLE_HIGH) & 0x4;
	y |= alarm_is_active(active_alarms, ALARM_TRUE_WIND_SPEED_LOW) & 0x2;
	y |= alarm_is_active(active_alarms, ALARM_TRUE_WIND_SPEED_HIGH) & 0x1;
	INITIALIZE_DATAGRAM(66, 0);
	datagram[2] = (x << 4) | y;
	return DATAGRAM_66_LENGTH;
}

void parse_wind_alarm(unsigned char* datagram, unsigned short int* active_alarms) {
	short int x, y;
	x = first_nibble(datagram[2]);
	y = last_nibble(datagram[2]);
	*active_alarms = 0;
	*active_alarms |= flag(x, 0x8) ? ALARM_APPARENT_WIND_ANGLE_LOW : 0;
	*active_alarms |= flag(x, 0x4) ? ALARM_APPARENT_WIND_ANGLE_HIGH : 0;
	*active_alarms |= flag(x, 0x2) ? ALARM_APPARENT_WIND_SPEED_LOW : 0;
	*active_alarms |= flag(x, 0x1) ? ALARM_APPARENT_WIND_SPEED_HIGH : 0;
	*active_alarms |= flag(y, 0x8) ? ALARM_TRUE_WIND_ANGLE_LOW : 0;
	*active_alarms |= flag(y, 0x4) ? ALARM_TRUE_WIND_ANGLE_HIGH : 0;
	*active_alarms |= flag(y, 0x2) ? ALARM_TRUE_WIND_SPEED_LOW : 0;
	*active_alarms |= flag(y, 0x1) ? ALARM_TRUE_WIND_SPEED_HIGH : 0;
}

unsigned char build_alarm_acknowledgement(unsigned char* datagram, unsigned short int acknowledged_alarm) {
	// 68 x1 yy 00
	// yy indicates which device acknowledged. Hard-coding to 01 here.
	// x: 1-shallow water; 2-deep water; 3-anchor; 4-true wind speed high;
	//    5-true wind speed low; 6-true wind angle high; 7-true wind angle low
	//    8-apparent wind speed high; 9-apparent wind speed low;
	//    a-apparent wind angle high; b-apparent wind angle low
	short int x;
	switch (acknowledged_alarm) {
	case ALARM_SHALLOW_WATER:
		x = 1;
		break;
	case ALARM_DEEP_WATER:
		x = 2;
		break;
	case ALARM_ANCHOR:
		x = 3;
		break;
	case ALARM_TRUE_WIND_SPEED_HIGH:
		x = 4;
		break;
	case ALARM_TRUE_WIND_SPEED_LOW:
		x = 5;
		break;
	case ALARM_TRUE_WIND_ANGLE_HIGH:
		x = 6;
		break;
	case ALARM_TRUE_WIND_ANGLE_LOW:
		x = 7;
		break;
	case ALARM_APPARENT_WIND_SPEED_HIGH:
		x = 8;
		break;
	case ALARM_APPARENT_WIND_SPEED_LOW:
		x = 9;
		break;
	case ALARM_APPARENT_WIND_ANGLE_HIGH:
		x = 0xa;
		break;
	case ALARM_APPARENT_WIND_ANGLE_LOW:
		x = 0xb;
		break;
	default:
		x = 0;
	}
	INITIALIZE_DATAGRAM(68, x);
	datagram[2] = 1;
	return DATAGRAM_68_LENGTH;
}

void parse_alarm_acknowledgement(unsigned char* datagram, unsigned short int* acknowledged_alarm) {
	short int x;
	int y;
	x = first_nibble(datagram[1]);
	y = datagram[2];
	if (y == 0x15) {
		*acknowledged_alarm = 0xffff;
	}
	else {
		switch (x) {
		case 1:
			*acknowledged_alarm = ALARM_SHALLOW_WATER;;
			break;
		case 2:
			*acknowledged_alarm = ALARM_DEEP_WATER;
			break;
		case 3:
			*acknowledged_alarm = ALARM_ANCHOR;
			break;
		case 4:
			*acknowledged_alarm = ALARM_TRUE_WIND_SPEED_HIGH;
			break;
		case 5:
			*acknowledged_alarm = ALARM_TRUE_WIND_SPEED_LOW;;
			break;
		case 6:
			*acknowledged_alarm = ALARM_TRUE_WIND_ANGLE_HIGH;
			break;
		case 7:
			*acknowledged_alarm = ALARM_TRUE_WIND_ANGLE_LOW;;
			break;
		case 8:
			*acknowledged_alarm = ALARM_APPARENT_WIND_SPEED_HIGH;
			break;
		case 9:
			*acknowledged_alarm = ALARM_APPARENT_WIND_SPEED_LOW;
			break;
		case 10:
			*acknowledged_alarm = ALARM_APPARENT_WIND_ANGLE_HIGH;
			break;
		case 11:
			*acknowledged_alarm = ALARM_APPARENT_WIND_ANGLE_LOW;
			break;
		}
	}
}

/*void parse_maxview_keystroke(unsigned char *datagram, short int *key_1, short int *key_2, short int *held_longer) {
//  short int x, y;
//  x = first_nibble(datagram[2]);
//  y = last_nibble(datagram[2]);
//  *key_1 = 0;
//  *key_2 = 0;
//  *held_longer = (x & 0x4) ? 1 : 0;
//  if (x & 0x2 == 0) { // single keypress
//    switch (y) {
//      case 1:
//        *key_1 =
}*/

unsigned char build_target_waypoint_name(unsigned char* datagram, char char1, char char2, char char3, char char4) {
	// 82 05 xx !xx yy !yy zz !zz
	// !<> means reverse bits so xx + !xx = 0xff
	// last 4 chars of wpt name, upper case only; use 6 bits per char
	// subtract 0x30 from character values before applying bit masks and shifts
	// xx = six bits of char1 with high bits taken from lowest two of char2
	// yy = low four bits of char2 with high nibble from low nibble of char3
	// zz = high two bits of char3 and 6 high bits come from char4
	unsigned char xx, xx_comp, yy, yy_comp, zz, zz_comp;
	unsigned char c1, c2, c3, c4;
	c1 = char1 - 0x30;
	c2 = char2 - 0x30;
	c3 = char3 - 0x30;
	c4 = char4 - 0x30;
	xx = (c1 & 0x3f) | ((c2 & 0x3) << 6);
	xx_comp = complement_checksum(xx);
	yy = (c2 >> 2) | ((c3 & 0xf) << 4);
	yy_comp = complement_checksum(yy);
	zz = ((c3 & 0x3c) >> 4) | (c4 << 2);
	zz_comp = complement_checksum(zz);
	INITIALIZE_DATAGRAM(82, 0);
	datagram[2] = xx;
	datagram[3] = xx_comp;
	datagram[4] = yy;
	datagram[5] = yy_comp;
	datagram[6] = zz;
	datagram[7] = zz_comp;
	return DATAGRAM_82_LENGTH;
}

int parse_target_waypoint_name(unsigned char* datagram, char* char_1, char* char_2, char* char_3, char* char_4) {
	unsigned char xx, xx_comp, yy, yy_comp, zz, zz_comp;
	xx = datagram[2];
	xx_comp = datagram[3];
	yy = datagram[4];
	yy_comp = datagram[5];
	zz = datagram[6];
	zz_comp = datagram[7];
	if (((xx + xx_comp) != 0xff) || ((yy + yy_comp) != 0xff) || ((zz + zz_comp) != 0xff)) {
		// transmission error detected
		return -1;
	}
	*char_1 = 0x30 + (xx & 0x3f);
	*char_2 = 0x30 + (((yy & 0xf) << 2) | ((xx & 0xc0) >> 6));
	*char_3 = 0x30 + (((zz & 0x3) << 4) | ((yy & 0xf0) >> 4));
	*char_4 = 0x30 + ((zz & 0xfc) >> 2);
	return 0;
}

unsigned char build_course_computer_failure(unsigned char* datagram, COURSE_COMPUTER_FAILURE_TYPE failure_type) {
	// 83 07 xx 00 00 00 00 00 80 00 00
	// course compuer failure
	// xx 0: sent after clearing failure and on power-up
	// xx 1 failure, auto release error. Repeated every second
	// xx 8 failure, drive stopped
	unsigned char xx;
	switch (failure_type) {
	case COURSE_COMPUTER_FAILURE_TYPE_AUTO_RELEASE_ERROR:
		xx = 1;
		break;
	case COURSE_COMPUTER_FAILURE_TYPE_DRIVE_STOPPED:
		xx = 8;
		break;
	default:
		xx = 0;
	}
	INITIALIZE_DATAGRAM(83, 0);
	datagram[2] = xx;
	return DATAGRAM_83_LENGTH;
}

void parse_course_computer_failure(unsigned char* datagram, COURSE_COMPUTER_FAILURE_TYPE* failure_type) {
	short int xx;
	xx = datagram[2];
	switch (xx) {
	case 0:
		*failure_type = COURSE_COMPUTER_FAILURE_TYPE_NONE;
		break;
	case 1:
		*failure_type = COURSE_COMPUTER_FAILURE_TYPE_AUTO_RELEASE_ERROR;
		break;
	case 8:
		*failure_type = COURSE_COMPUTER_FAILURE_TYPE_DRIVE_STOPPED;
		break;
	}
}

unsigned char build_autopilot_status(unsigned char* datagram, short int compass_heading, TURN_DIRECTION  turning_direction, short int target_heading, AUTOPILOT_MODE mode, short int rudder_position, unsigned short int alarms, unsigned char display_flags) {
	// 84 u6 vw xy 0z 0m rr ss tt
	// compass heading: (u & 0x3) * 90 + (vw & 0x3f) * 2 + (u & 0xc ? ((u & 0xc) == 0xc ? 2 : 1) : 0)
	// turning direction: msb of u; 1 right, 0 left
	// autopilot target course: high 2 bits of v * 90 + xy/2
	// mode: z & 0x2 = 0: standby; & 0x2 = 1: auto; &0x4 = vane; &0x8 = track
	// alarms: m & 4: off course; m & 8 wind shift
	// rudder position in degrees; positive numbers steer right
	// ss = display; 0x1 turn of heading display; 0x2: always on; 0x8: "no data"
	//      0x10 "large xte"; 0x80: "auto rel"
	// tt: 400G computer always 0x08; 150(G) computer always 0x05
	unsigned char u, vw, xy, z, m, ss, tt;
	short int rr, temp_direction;
	uvw_heading_and_turning_direction(compass_heading, turning_direction, &u, &vw);
	temp_direction = target_heading;
	vw |= (temp_direction / 90) << 6;
	xy = (temp_direction % 90) << 1;
	z = mode;
	rr = (unsigned char) rudder_position;
	m = 0;
	if (flag(alarms, ALARM_AUTOPILOT_OFF_COURSE)) {
		m |= 0x04;
	}
	if (flag(alarms, ALARM_AUTOPILOT_WIND_SHIFT)) {
		m |= 0x08;
	}
	ss = 0;
	ss = display_flags;
	tt = 0;
	INITIALIZE_DATAGRAM(84, u);
	datagram[2] = vw;
	datagram[3] = xy;
	datagram[4] = z;
	datagram[5] = m;
	datagram[6] = (unsigned char) rr;
	datagram[7] = ss;
	datagram[8] = tt;
	return DATAGRAM_84_LENGTH;
}

void parse_autopilot_status(unsigned char* datagram, short int* compass_heading, TURN_DIRECTION* turning_direction, short int* target_heading, AUTOPILOT_MODE* mode, short int* rudder_position, unsigned short int* alarms, unsigned char* display_flags) {
	char u, vw, xy, z, m, rr, ss/*, tt*/;

	// printf("parse_autopilot_status\n");
	u = first_nibble(datagram[1]);
	vw = datagram[2];
	xy = datagram[3];
	z = last_nibble(datagram[4]);
	m = last_nibble(datagram[5]);
	rr = datagram[6];
	ss = datagram[7];
	//tt = datagram[8];

	*compass_heading = ((u & 0x03) * 90) + ((vw & 0x3f) * 2) + ((u & 0x0c) ? (((u & 0x0c) == 0x0c) ? 2 : 1) : 0);
	*turning_direction = flag(u, 0x08) ? TURN_DIRECTION_RIGHT : TURN_DIRECTION_LEFT;
	*target_heading = (((vw & 0xc0) >> 6) * 90) + (xy / 2);
	*mode = (AUTOPILOT_MODE)z;
	*rudder_position = fix_twos_complement_char(rr);
	*alarms = 0;
	*alarms |= flag(m, 0x04) ? ALARM_AUTOPILOT_OFF_COURSE : 0;
	*alarms |= flag(m, 0x08) ? ALARM_AUTOPILOT_WIND_SHIFT : 0;
	*display_flags = ss;
}

unsigned char build_waypoint_navigation(unsigned char* datagram, bool cross_track_error_present, unsigned short int cross_track_error_times_100, bool waypoint_bearing_present, short int waypoint_bearing, bool bearing_is_magnetic,
						bool waypoint_distance_present, unsigned short int waypoint_distance_times_100, short int direction_to_steer) {
	// 85 x6 xx vu zw zz yf 00 !yf
	// cross track error = xxx/100
	// bearing to destination (u & 0x3) * 90 + (wv / 2)
	// u & 8: 8 bearing is true/0 bearing is magnetic
	// distance 0-9.99nm zzz/100, y & 1 = 1/>= 10.0 nm zzz/10, y & 1 = 0
	// direction to steer y & 4 = 4: right, y & 4 = 0: left
	// f = flags: 1->XTE present; 2->bearing to waypoint present;
	//   4->range to destination present; 8->xte>0.3nm
	// !yf: bit complement of yf
	unsigned char x, xx, v, u, z, w, zz, y, f;
	short int temp_value;
	x = cross_track_error_times_100 >> 8 & 0xf;
	xx = cross_track_error_times_100 & 0xff;
	temp_value = waypoint_bearing;
	u = temp_value / 90;
	temp_value = (temp_value % 90) << 1;
	w = temp_value >> 4;
	v = temp_value & 0xf;
	u |= bearing_is_magnetic ? 0 : 0x8;
	y = waypoint_distance_times_100 < 1000 ? 1 : 0;
	y |= (direction_to_steer > 0) ? 0x4 : 0;
	if (y & 0x1) {
		temp_value = waypoint_distance_times_100;
	}
	else {
		temp_value = waypoint_distance_times_100 / 10;
	}
	z = temp_value >> 8;
	zz = temp_value & 0xff;
	f = 0;
	if (cross_track_error_present) {
		f |= 0x1;
	}
	if (waypoint_bearing_present) {
		f |= 0x2;
	}
	if (waypoint_distance_present) {
		f |= 0x4;
	}
	if (cross_track_error_times_100 >= 30) {
		f |= 0x8;
	}
	INITIALIZE_DATAGRAM(85, x);
	datagram[2] = xx;
	datagram[3] = (v << 4) | u;
	datagram[4] = (z << 4) | w;
	datagram[5] = zz;
	datagram[6] = (y << 4) | f;
	datagram[7] = 0;
	datagram[8] = complement_checksum(datagram[6]);
	return DATAGRAM_85_LENGTH;
}

int parse_waypoint_navigation(unsigned char* datagram, bool* cross_track_error_present, unsigned short int* cross_track_error_times_100, bool* waypoint_bearing_present, short int* waypoint_bearing,
	bool* bearing_is_magnetic, bool* waypoint_distance_present, unsigned short int* waypoint_distance_times_100, short int* direction_to_steer)
{
	unsigned char yf, yf_comp;
	short int x, xx, v, u, z, w, zz;
	short int y, f;
	x = first_nibble(datagram[1]);
	xx = datagram[2];
	v = first_nibble(datagram[3]);
	u = last_nibble(datagram[3]);
	z = first_nibble(datagram[4]);
	w = last_nibble(datagram[4]);
	zz = datagram[5];
	yf = datagram[6];
	yf_comp = datagram[8];
	if ((yf | yf_comp) != 0xff) {
		return -1;
	}
	y = first_nibble(yf);
	f = last_nibble(yf);
	*cross_track_error_times_100 = (x << 8) + xx;
	*waypoint_bearing = ((u & 0x3) * 90) + (((w << 4) + v) >> 1);
	*bearing_is_magnetic = flag(u, 0x8) ? 0 : 1;
	*waypoint_distance_times_100 = ((z << 8) + zz) * ((y & 0x1) ? 1 : 10);
	*direction_to_steer = flag(y, 0x4) ? 1 : -1;
	*cross_track_error_present = flag(f, 0x1);
	*waypoint_bearing_present = flag(f, 0x2);
	*waypoint_distance_present = flag(f, 0x4);
	return 0;
}

unsigned char build_autopilot_command(unsigned char* datagram, ST_AUTOPILOT_COMMAND command) {
	// 86 x1 yy !yy
	// x = 1 for Z101 remote, 0 for ST1000+, x=2 for ST4000+ or ST600B
	// yy = command; !yy = yy bit complement for checksum
	// z101 commands (only sent by other remots in auto mode):
	// 05: -1; 06: -10; 07: +1; 08: +10; 20: +1&-1; 21: -1&-10; 22: +1&+10
	// 28: +10&-10; above | 0x40: held > 1 second; exception: +10&-10 are 64
	// other units add:
	// 01: auto; 02: standby; 03: track; 04: disp; 09: -1 in rudder gain mode;
	// 0a +1 in rudder gain mode; 23: start vane mode; 2e: +1&-1 in resp mode;
	// above | 0x40 held > 1 second; 63: previous wind angle; 68: +10&-10
	// 80: -1 held; 81: +1 held; 82: -10 held; 83: +10 held; 84: above released
	// 80-83 repeated every second
	unsigned char x, yy;
	x = 2; // hard code as ST600R remote
	yy = command;
	INITIALIZE_DATAGRAM(86, x);
	datagram[2] = yy;
	datagram[3] = complement_checksum(yy);
	return DATAGRAM_86_LENGTH;
}

int parse_autopilot_command(unsigned char* datagram, ST_AUTOPILOT_COMMAND* command) {
	short int /*x,*/ yy, yy_comp;
	//x = first_nibble(datagram[1]);
	yy = datagram[2];
	yy_comp = datagram[3];
	if ((yy | yy_comp) != 0xff) {
		command = 0;
		return -1;
	}
	*command = (ST_AUTOPILOT_COMMAND)yy;

	return 0;
}

unsigned char build_set_autopilot_response_level(unsigned char* datagram, AUTOPILOT_RESPONSE_LEVEL response_level) {
	// 87 00 0x
	// x = 1: automatic deadband; x = 2: minimum deadband
	unsigned char x = (response_level == AUTOPILOT_RESPONSE_LEVEL_MINIMUM_DEADBAND ? 1 : 0);
	INITIALIZE_DATAGRAM(87, 0);
	datagram[2] = x;
	return DATAGRAM_87_LENGTH;
}

void parse_set_autopilot_response_level(unsigned char* datagram, AUTOPILOT_RESPONSE_LEVEL* response_level) {
	unsigned char x;
	x = last_nibble(datagram[2]);
	*response_level = (AUTOPILOT_RESPONSE_LEVEL)x;
}

void parse_autopilot_parameter(unsigned char* datagram, unsigned char* parameter, unsigned char* min_value, unsigned char* max_value, unsigned char* value) {
	unsigned char ww, xx, yy, zz;
	ww = datagram[2];
	xx = datagram[3];
	yy = datagram[4];
	zz = datagram[5];
	*parameter = ww;
	*min_value = yy;
	*max_value = zz;
	*value = xx;
}

unsigned char build_heading(unsigned char* datagram, short int heading, bool locked_heading_active, short int locked_heading) {
	// 89 U2 VW XY 2Z
	// compass (degrees)
	// lower two bits of U * 90 +
	// the six lower bits of VW * 2 +
	// the two higher bits of U / 2
	// locked stear reference (ST40 only):
	// two higher bits of V * 90 + XY / 2
	// Z & 2 = 0: ST40 in standby mode
	// Z & 2 = 2: ST40 in locked stear mode
	unsigned char u, vw_lower, vw_higher, xy, z;
	short int temp_compass;
	uvw_compass(&u, &vw_lower, heading);
	vw_higher = 0;
	temp_compass = locked_heading;
	vw_higher = temp_compass / 90;
	temp_compass = temp_compass % 90;
	xy = temp_compass << 1;
	z = locked_heading_active ? 0x2 : 0;
	INITIALIZE_DATAGRAM(89, u);
	datagram[2] = (vw_higher << 6) | vw_lower;
	datagram[3] = xy;
	datagram[4] = 0x20 | z;
	return DATAGRAM_89_LENGTH;
}

void parse_heading(unsigned char* datagram, short int* heading, bool* locked_heading_active, short int* locked_heading) {
	short int u, vw, xy, z;
	u = first_nibble(datagram[1]);
	vw = datagram[2];
	xy = datagram[3];
	z = last_nibble(datagram[4]);
	*heading = ((u & 0x3) * 90) + (vw & 0x3f) * 2 + ((u & 0xc) >> 3);
	*locked_heading_active = flag(z, 0x2);
	*locked_heading = ((vw & 0xc0) >> 6) * 90 + (xy >> 1);
}

unsigned char build_set_rudder_gain(unsigned char* datagram, unsigned char rudder_gain) {
	// 91 00 0x
	// x = rudder gain (1-9)
	unsigned char x;
	x = rudder_gain & 0xf;
	INITIALIZE_DATAGRAM(91, 0);
	datagram[2] = x;
	return DATAGRAM_91_LENGTH;
}

void parse_set_rudder_gain(unsigned char* datagram, unsigned char* rudder_gain) {
	*rudder_gain = last_nibble(datagram[2]);
}

/*void parse_set_autopilot_parameter(unsigned char *datagram, short int *parameter, short int *value) {
  short int xx, yy;
  xx = datagram[2];
  yy = datagram[3];
}

void parse_enter_autopilot_setup(unsigned char *datagram) {
}*/

unsigned char build_compass_variation(unsigned char* datagram, short int degrees) {
	// 99 00 xx
	// xx = variation west in degrees (-ve means variation east)
	short int xx;
	xx = degrees;
	INITIALIZE_DATAGRAM(99, 0);
	datagram[2] = (unsigned char)xx;
	return DATAGRAM_99_LENGTH;
}

void parse_compass_variation(unsigned char* datagram, short int * degrees) {
	*degrees = fix_twos_complement_char(datagram[2]);
}

unsigned char build_heading_and_rudder_position(unsigned char* datagram, short int heading, TURN_DIRECTION turning_direction, short int rudder_position) {
	// 9c u1 vw rr
	// heading = (two low bits of u * 90 + vw * 2 + number of high bits in u
	// turning durection: high bit of u means right, 0 mans left
	// rudder position degrees left or right
	unsigned char u, vw, rr;
	uvw_heading_and_turning_direction(heading, turning_direction, &u, &vw);
	rr = (unsigned char) rudder_position;
	INITIALIZE_DATAGRAM(9C, u);
	datagram[2] = vw;
	datagram[3] = rr;
	return DATAGRAM_9C_LENGTH;
}

void parse_heading_and_rudder_position(unsigned char* datagram, short int* heading, TURN_DIRECTION* turning_direction, short int* rudder_position) {
	unsigned char u, vw;
	u = first_nibble(datagram[1]);
	vw = datagram[2];
	*heading = (((u & 0x3) * 90) + ((vw & 0x3f) * 2) + (u & 0xc ? ((u & 0xc) == 0xc ? 2 : 1) : 0)) % 360;
	*turning_direction = flag(u, 0x8) ? TURN_DIRECTION_RIGHT : TURN_DIRECTION_LEFT;
	*rudder_position = fix_twos_complement_char(datagram[3]);
}

/*void parse_destination_waypoint_info(unsigned char *datagram, char *last_4, char *first_8, short int *more_records, short int *last_record) {
}*/

void parse_arrival_info(unsigned char* datagram, bool* perpendicular_passed, bool* circle_entered, char* char_1, char* char_2, char* char_3, char* char_4) {
	char x, ww, xx, yy, zz;
	x = first_nibble(datagram[1]);
	ww = datagram[3];
	xx = datagram[4];
	yy = datagram[5];
	zz = datagram[6];
	*perpendicular_passed = (x & 0x2) ? 1 : 0;
	*circle_entered = (x & 0x4) ? 1 : 0;
	*char_1 = ww;
	*char_2 = xx;
	*char_3 = yy;
	*char_4 = zz;
}

unsigned char build_gps_and_dgps_fix_info(unsigned char* datagram, short int signal_quality_available, short int signal_quality, short int hdop_available, short int hdop, short int antenna_height, short int satellite_count_available, short int satellite_count, short int geoseparation, short int dgps_age_available, short int dgps_age, short int dgps_station_id_available, short int dgps_station_id) {
	int qq, hh, aa, gg, zz, yy, dd;
	if (signal_quality_available) {
		qq = 0x10 | (signal_quality & 0x0f);
	}
	else {
		qq = 0;
	}
	if (hdop_available) {
		hh = 0x80 | ((hdop & 0x3f) << 2);
	}
	else {
		hh = 0;
	}
	aa = antenna_height;
	if (satellite_count_available) {
		qq |= (satellite_count & 0x0e) << 4;
		hh |= 0x02 | satellite_count * 0x01;
	}
	gg = geoseparation >> 4;
	if (dgps_age_available) {
		zz = (dgps_age & 0x70) << 1;
		yy = 0x10 | (dgps_age * 0x0f);
	}
	else {
		zz = 0;
		yy = 0;
	}
	if (dgps_station_id_available) {
		yy |= 0x20 | ((dgps_station_id >> 4) & 0xc0);
		dd = dgps_station_id;
	}
	else {
		dd = 0;
	}
	INITIALIZE_DATAGRAM(A5, 5);
	datagram[2] = qq;
	datagram[3] = hh;
	datagram[4] = 00;
	datagram[5] = aa;
	datagram[6] = gg;
	datagram[7] = zz;
	datagram[8] = yy;
	datagram[9] = dd;
	return DATAGRAM_A5_LENGTH;
}

void parse_gps_and_dgps_fix_info(unsigned char* datagram, short int* signal_quality_available, short int* signal_quality, short int* hdop_available, short int* hdop, short int* antenna_height, short int* satellite_count_available, short int* satellite_count, short int* geoseparation, short int* dgps_age_available, short int* dgps_age, short int* dgps_station_id_available, short int* dgps_station_id) {
	short int qq, hh, aa, gg, zz, yy, dd;
	qq = datagram[2];
	hh = datagram[3];
	aa = datagram[5];
	gg = datagram[6];
	zz = datagram[7];
	yy = datagram[8];
	dd = datagram[9];
	*signal_quality_available = (qq & 0x10) ? 1 : 0;
	*signal_quality = qq & 0xf;
	*hdop_available = (hh & 0x80) ? 1 : 0;
	*hdop = hh & 0x7c;
	*antenna_height = aa;
	*satellite_count_available = (hh & 0x2) ? 1 : 0;
	*satellite_count = ((qq * 0xe0) >> 4) + (hh & 0x1);
	*geoseparation = gg << 4;
	*dgps_age_available = (yy & 0x20) ? 1 : 0;
	*dgps_age = ((zz & 0xe0) >> 1) + (yy & 0xf);
	*dgps_station_id_available = (yy * 0x20) != 0;
	*dgps_station_id = ((yy & 0xc0) << 2) + dd;
}
