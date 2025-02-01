#include <stdio.h>
#include "test.h"
#include "boat_sensor_test.h"
#include "../boat_sensor.h"
#include "../logger.h"

#define REFUTE_POP_VALUE(NAME, TYPE) refute(pop_##NAME##_sensor_value(&sensor_value) == 0, "pop_" #NAME "_sensor_value() should fail before value set")
#define ASSERT_POP_VALUE(NAME, TYPE) assert_equal_int(0, pop_##NAME##_sensor_value(&sensor_value), "pop_" #NAME "_sensor_value() should succeed")

#define TEST_SENSOR(NAME, TYPE) TEST(NAME##_sensor)\
  TYPE sensor_value;\
  REFUTE_POP_VALUE(NAME, TYPE);\
  update_##NAME##_sensor(TYPE##_TEST_VALUE);\
  ASSERT_POP_VALUE(NAME, TYPE);\
  assert_equal_##TYPE(TYPE##_TEST_VALUE, sensor_value, #NAME);\
}

TEST_SENSOR(heading, int16)
TEST_SENSOR(water_speed_in_knots_times_100, uint16)
TEST_SENSOR(apparent_wind_angle, int16)
TEST_SENSOR(apparent_wind_speed_in_knots_times_10, uint16)
TEST_SENSOR(depth_below_transducer_in_feet_times_10, int16)
TEST_SENSOR(course_over_ground, int16)
TEST_SENSOR(speed_over_ground_in_knots_times_100, uint16)
TEST_SENSOR(water_temperature_in_degrees_celsius_times_10, int16)
TEST_SENSOR(rudder_position_in_degrees_right, int16)

void test_boat_sensor() {
  initialize_sensors();
  printf("--- Testing boat_sensor.c\n");
  test_heading_sensor();
  test_water_speed_in_knots_times_100_sensor();
  test_apparent_wind_angle_sensor();
  test_apparent_wind_speed_in_knots_times_10_sensor();
  test_depth_below_transducer_in_feet_times_10_sensor();
  test_course_over_ground_sensor();
  test_speed_over_ground_in_knots_times_100_sensor();
  test_water_temperature_in_degrees_celsius_times_10_sensor();
  test_rudder_position_in_degrees_right_sensor();
}

