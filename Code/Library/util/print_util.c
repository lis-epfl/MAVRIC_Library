#include "print_util.h"
//#include "sysclk.h"
//#include "gpio.h"
//#include "conf_usart_serial.h"
//#include "uart_int.h"

//#include "delay.h"

byte_stream_t* deb_stream;


byte_stream_t* get_debug_stream(){
	return deb_stream;
}

Bool blocking;

/**
 * \brief some utilities for printing strings and numbers 
 */
void dbg_print_init(byte_stream_t* debug_stream)
{
	deb_stream=debug_stream;
}

void putstring(byte_stream_t *out_stream, const char* s) {
	if ((out_stream==NULL) || (out_stream->put==NULL)) return;
	while (*s != 0) {
		out_stream->put(out_stream->data, *s);
		s++;
	}
}

static const char alphabet[36] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

void putdigit(byte_stream_t *out_stream, unsigned c){
if ((out_stream==NULL) || (out_stream->put==NULL)) return;
	
if (c > 35)
    return;
  out_stream->put(out_stream->data,  alphabet[c]);
}


/*
  Outputs numbers with less than 10 digits
*/
void putnum(byte_stream_t *out_stream, long c, char base){
  char storage[MAX_DIGITS];
  long i = MAX_DIGITS;
  if ((out_stream==NULL) || (out_stream->put==NULL)) return;
  /* Take Care of the sign */
  if(c < 0){
    out_stream->put(out_stream->data,   '-');
    c = c*-1;
  } else {
	  out_stream->put(out_stream->data,  ' ');
    
  }

  do{
    i--;
    storage[i] = c % base;
    c = c / base;
  }while((i >= 0) 
  && (c > 0)
  );
  /* i is the index of the last digit calculated */

  /* Hence, there is no need to initialize i */
  for( ; i<MAX_DIGITS; i++){
     putdigit(out_stream, storage[i]);
  }

}

void putnum_tight(byte_stream_t *out_stream, long c, char base){
  char storage[MAX_DIGITS];
  long i = MAX_DIGITS;
  if ((out_stream==NULL) || (out_stream->put==NULL)) return;
  /* Take Care of the sign */
  if(c < 0){
    out_stream->put(out_stream->data,   '-');
    c = c*-1;
  } else {
    
  }

  do{
    i--;
    storage[i] = c % base;
    c = c / base;
  }while((i >= 0) 
  && (c > 0)
  );
  /* i is the index of the last digit calculated */

  /* Hence, there is no need to initialize i */
  for( ; i<MAX_DIGITS; i++){
     putdigit(out_stream, storage[i]);
  }

}

void putfloat(byte_stream_t *out_stream, float c, int after_digits){
	int i;
	float num=c;
	if (c<0) {
		putstring(out_stream, "-");
		num=-c;
	} else putstring(out_stream, "");
	int whole=abs((int)num);
	float after=(num-(float)whole);

	putnum_tight(out_stream, whole, 10);
	putstring(out_stream, "."); 
	
	for (i=0; i<after_digits; i++) 
	{
		after*=10;
		putdigit(out_stream, (int)after);
		after=after-(int)after;
	}
	
}


void print_vector(byte_stream_t *out_stream, float v[], int after_digits) {
	int i;
	putstring(out_stream, "(");
	for (i=0; i<3; i++) {
		putfloat(out_stream, v[i], after_digits);
		if (i<2) putstring(out_stream, ", ");

	}
	putstring(out_stream, ") ");

}

void print_quaternion(byte_stream_t *out_stream, UQuat_t *quat, int after_digits) {
	putstring(out_stream, "(");
	putfloat(out_stream, quat->s, after_digits);
	putstring(out_stream, ", ");
	print_vector(out_stream, quat->v, after_digits);
	putstring(out_stream, ") ");
}


void print_matrix(byte_stream_t *out_stream, float v[], int rows, int columns, int after_digits) {
	int i, j;
	
	for (i=0; i<rows; i++) {
		putstring(out_stream, "| ");
		for (j=0; j<columns; j++) {
			putfloat(out_stream, v[i*rows+j], after_digits);
			if (j<columns-1) putstring(out_stream, ", ");
		}
		putstring(out_stream, " |\n");
	}
}


void dbg_print(const char* s) {
	putstring(deb_stream, s);

}

void dbg_print_num(long c, char base) {
	putnum(deb_stream, c, base);

}

void dbg_putfloat(float c, int after_digits) {
	putfloat(deb_stream, c, after_digits);
}

void dbg_print_vector(float v[], int after_digits) {
	print_vector(deb_stream, v, after_digits);
}

void dbg_print_quaternion(UQuat_t *quat, int after_digits) {
	print_quaternion(deb_stream, quat, after_digits);
}


void dbg_log_value(const char* msg, long value, char base) {
	dbg_print(msg);
	if (base>1) {
		dbg_print_num(value, base);
	}
	dbg_print("\n");

}
