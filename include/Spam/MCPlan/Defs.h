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
//#include <boost/shared_ptr.hpp>
//#include <Vec3.h>
#endif
//#define WIN32
#ifdef WIN32
//#include <time.h>
//#include <boost/shared_ptr.hpp>
//#include <Golem/Math/Math.h>
//#include <Golem/Math/Vec3.h>
//#include <Golem/Tools/Context.h>
//#include <Golem/Math/Sample.h>
//
//typedef golem::Vec3 Vec3;
//typedef std::vector<golem::Vec3> Vec3Seq;
//typedef golem::U32 U32;
//typedef golem::U64 U64;
//typedef golem::I32 I32;
//typedef golem::Real Real;
#endif

float claudio_used_time() { 
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

float claudio_used_time(float& T) { 
	float t = T;
	T = claudio_used_time();
	return  T-t;
};

//-------------------------------------------------------------------------
