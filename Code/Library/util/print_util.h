#ifndef PRINT_UTIL_H
#define PRINT_UTIL_H

//#include "usart.h"
#include "streams.h"
#include "maths.h"

#define MAX_DIGITS 10
//#define STDOUT 0

#ifdef __cplusplus
extern "C" {
#endif

void dbg_print_init(byte_stream_t* debug_stream);
byte_stream_t* get_debug_stream(void);

void putstring(byte_stream_t *out_stream, const char* s);
void putdigit(byte_stream_t *out_stream, unsigned c);
void putnum(byte_stream_t *out_stream, long c, char base);
void putfloat(byte_stream_t *out_stream, float c, int after_digits);
void print_matrix(byte_stream_t *out_stream, float v[], int rows, int columns, int after_digits);
void print_vector(byte_stream_t *out_stream, float v[], int after_digits); 
void print_quaternion(byte_stream_t *out_stream, UQuat_t *quat, int after_digits); 

void dbg_print(const char* s);
void dbg_print_num(long c, char base);
void dbg_log_value(const char* msg, long value, char base);
void dbg_putfloat(float c, int after_digits);

void dbg_print_vector(float v[], int after_digits); 
void dbg_print_quaternion(UQuat_t *quat, int after_digits); 

#ifdef __cplusplus
}
#endif

#endif