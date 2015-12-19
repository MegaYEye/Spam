//-------------------------------------------------------------------------
//
//#include <stdio.h>
//#include <GL/glut.h>

//-------------------------------------------------------------------------

//// Util
#if defined(__unix__)
#include <unistd.h>
#include <pwd.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/types.h>
#endif
//#define WIN32
#ifdef WIN32
#include <time.h>
#endif

float claudio_used_time()
{ 
#if defined(__unix__)

#if defined(CLK_TCK)
  long clk_tck = CLK_TCK;
#else
  //  long clk_tck = HZ;
  long clk_tck = sysconf(_SC_CLK_TCK);
#endif

  tms x;
  times(&x);
  return  float(x.tms_utime)/clk_tck;

#else

  return  float(clock())/CLOCKS_PER_SEC;

#endif
};


float claudio_used_time(float& T){ 
	float t = T;
	T = claudio_used_time();
	return  T-t;
};

//-------------------------------------------------------------------------
