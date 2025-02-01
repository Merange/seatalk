#ifndef _TEST_H
#define _TEST_H


#include <string.h>
#include "stdio.h"

#include "defines.h"
#include "assertions.h"

#define TEST(NAME) void test_##NAME() {\
  sprintf(test_name, #NAME);\
  printf("Running %s\n", test_name);

extern char test_name[256];

#define int_TEST_VALUE 137
#define int16_TEST_VALUE 1376
#define uint16_TEST_VALUE 753
#define char_TEST_VALUE 159
#define uint32_TEST_VALUE 80808

#define TURN_DIRECTION_TEST_VALUE TURN_DIRECTION_RIGHT

#endif