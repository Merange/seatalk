#ifndef ASSERTIONS_H
#define ASSERTIONS_H

#include "defines.h"

int total_assertions(void);
int total_failures(void);
void assert(int true_or_false, char* explanation);
void refute(int true_or_false, char* explanation);
void assert_equal_char(char reference, char test, char* explanation);
void assert_equal_int16(short int reference, short int test, char* explanation);
void assert_equal_uint16(unsigned short int reference, unsigned short int test, char* explanation);
void assert_equal_uint32(unsigned long int reference, unsigned long int test, char* explanation);
void assert_equal_string(char* reference, char* test, char* explanation);
void assert_equal_float(float reference, float test, char* explanation);
void assert_equal_int(int reference, int test, char* explanation);

#define assert_equal_TURN_DIRECTION(REF_VALUE, TEST_VALUE, MESSAGE) assert_equal_int(REF_VALUE, TEST_VALUE, MESSAGE)

#endif // !ASSERTIONS_H

