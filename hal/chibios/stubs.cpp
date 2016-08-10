#include <string>
#include <algorithm>

extern "C"
{
#include <sys/types.h>
#include <stdlib.h>
}

extern "C" pid_t _getpid(void)  __attribute__((used));
pid_t _getpid(void)
{
	return 0;
}

extern "C" void _exit( int status ) __attribute__((used));
void _exit( int status )
{
	(void)status;
	while( 1 );
}

extern "C" int _kill( int pid, int sig ) __attribute__((used));
int _kill( int pid, int sig )
{
	(void)pid; (void)sig;
	return -1;
}


int _open_r(void *reent, const char *file, int flags, int mode)
{
	(void)reent; (void)file; (void)flags; (void)mode;
	return -1;
}

void *__dso_handle = NULL;


/*
 * The default pulls in 70K of garbage
 */
namespace __gnu_cxx {

  void __verbose_terminate_handler() {
    for(;;);
  }
}


/*
 * The default pulls in about 12K of garbage
 */

extern "C" void __cxa_pure_virtual() {
  for(;;);
}
