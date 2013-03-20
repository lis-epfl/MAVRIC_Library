#include "print_util.h"
#include "sysclk.h"
#include "gpio.h"
#include "conf_usart_serial.h"
#include "uart_int.h"

#include "delay.h"

byte_stream_t* deb_stream;


Bool blocking;

/**
 * \brief some utilities for printing strings and numbers 
 */
void dbg_print_init(byte_stream_t* debug_stream)
{
	deb_stream=debug_stream;
}

void putstring(byte_stream_t *out_stream, const char* s) {
	
	while (*s != 0) {
		out_stream->put(out_stream->data, *s);
		s++;
	}
}

static const char alphabet[36] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

void putdigit(byte_stream_t *out_stream, unsigned c){
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

void dbg_print(const char* s) {
	putstring(deb_stream, s);

}

void dbg_print_num(long c, char base) {
	putnum(deb_stream, c, base);

}

void dbg_log_value(const char* msg, long value, char base) {
	dbg_print(msg);
	if (base>1) {
		dbg_print_num(value, base);
	}
	dbg_print("\n");

}
