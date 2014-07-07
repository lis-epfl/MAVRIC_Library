/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file print_util.h
 * 
 * Some utilities for printing strings and numbers 
 */


#ifndef PRINT_UTIL_H
#define PRINT_UTIL_H

#ifdef __cplusplus
extern "C" 
{
#endif

#include "streams.h"
#include "quaternions.h"

#define MAX_DIGITS 10


/**
 * \brief              		Init debug stream
 * 
 * \param 	debug_stream 	Stream to be forwarded to the debug stream
 */
void print_util_dbg_print_init(byte_stream_t* debug_stream);


/**
 * \brief              Get pointer to debug stream
 * 
 * \return             Pointer to debug stream
 */
byte_stream_t* print_util_get_debug_stream(void);


/**
 * \brief              		Writes string of character to a stream
 * 
 * \param 	out_stream   	Pointer to ouput stream
 * \param 	s            	Character string
 */
void print_util_putstring(byte_stream_t *out_stream, const char* s);


/**
 * \brief              		Writes an alphabet character to a stream,
 *  
 * \details 				The number is selected according to its position in the following list
 * 							alphabet[36] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
 * 							
 * \param 	out_stream  	Pointer to output stream
 * \param 	c            	Number between 0 and 35
 */
void print_util_putdigit(byte_stream_t *out_stream, uint32_t  c);


/**
 * \brief              Writes a number in any base to a stream
 * 
 * \param out_stream   Pointer to output stream
 * \param c            Number
 * \param base         Base in which the number should be printed
 */
void print_util_putnum(byte_stream_t *out_stream, int32_t c, char base);


/**
 * \brief              		Writes a float to a stream
 * 
 * \param 	out_stream   	Pointer to output stream
 * \param 	c            	Floating point value
 * \param 	after_digits 	Number of digits to write after the radix point
 */
void print_util_putfloat(byte_stream_t *out_stream, float c, int32_t after_digits);


/**
 * \brief              		Writes a matrix to a stream
 * 
 * \param 	out_stream   	Pointer
 * \param 	v            	Array of floats containing the matrix elements
 * \param 	rows 			Number of rows
 * \param 	columns			Number of columns
 * \param 	after_digits 	Number of digits to write after the radix point
 */
void print_util_print_matrix(byte_stream_t *out_stream, float v[], int32_t rows, int32_t columns, int32_t after_digits);


/**
 * \brief              		Writes a vector to a stream
 * 
 * \param 	out_stream   	Pointer to output stream
 * \param 	v            	Array of floats containing the vector elements
 * \param 	after_digits 	Number of digits to write after the radix point
 */
void print_util_print_vector(byte_stream_t *out_stream, float v[], int32_t after_digits); 


/**
 * \brief              		Writes a quaternion to stream
 * 
 * \param 	out_stream   	Pointer to output stream
 * \param 	quat         	Unit quaternion
 * \param 	after_digits 	Number of digits to write after the radix point
 */
void print_util_print_quaternion(byte_stream_t *out_stream, UQuat_t *quat, int32_t after_digits); 


/**
 * \brief              Writes a character string to the debug stream
 * 
 * \param s            Character string
 */
void print_util_dbg_print(const char* s);


/**
 * \brief              Writes a number in any base to the debug stream
 * 
 * \param c            Number
 * \param base         Base in which the number should be printed
 */
void print_util_dbg_print_num(int32_t c, char base);


/**
 * \brief              	Writes a log-like message to the debug stream
 * 
 * \details 			The message has the following format: <MSG> <VALUE>. 
 * 						example: Altitude: 42
 * 						
 * \param 	msg         Descriptive message
 * \param 	value       Associated value
 * \param 	base        Base in which the number should be printed
 */
void print_util_dbg_log_value(const char* msg, int32_t value, char base);


/**
 * \brief              		Writes a floating point value to the debug stream
 * 
 * \param 	c           	Floating point value
 * \param 	after_digits  	Number of digits to write after the radix point
 */
void print_util_dbg_putfloat(float c, int32_t after_digits);


/**
 * \brief              		Writes a vector to the debug stream
 * 
 * \param 	v            	Array of float containing the vector elements
 * \param 	after_digits 	Number of digits to write after the radix point
 */
void print_util_dbg_print_vector(float v[], int32_t after_digits); 


/*!
 * \brief              		Writes a quaternion to the debug stream
 * 
 * \param 	quat         	Unit quaternion
 * \param 	after_digits 	Number of digits to write after the radix point
 */
void print_util_dbg_print_quaternion(UQuat_t *quat, int32_t after_digits); 


#ifdef __cplusplus
}
#endif

#endif