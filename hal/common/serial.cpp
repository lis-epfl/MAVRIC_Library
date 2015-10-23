#include "serial.hpp"

/**
 * \brief 	write newline character to stream ('\n\r')
 *
 * \return 	success
 */
bool Serial::newline()
{
	const uint8_t newline[2] = {'\n','\r'};
	return write(newline,2);
}