#ifndef PRINT_UTIL_H
#define PRINT_UTIL_H

//#include "usart.h"
#include "streams.h"

#define MAX_DIGITS 10
//#define STDOUT 0

void dbg_print_init(byte_stream_t* debug_stream);


void putstring(byte_stream_t *out_stream, const char* s);
void putdigit(byte_stream_t *out_stream, unsigned c);
void putnum(byte_stream_t *out_stream, long c, char base);

void dbg_print(const char* s);
void dbg_print_num(long c, char base);
void dbg_log_value(const char* msg, long value, char base);

#endif