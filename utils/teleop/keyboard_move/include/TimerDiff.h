#ifndef _TIMERDIFF_KYOSHO_20110903_
#define _TIMERDIFF_KYOSHO_20110903_


//#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>

class cTimerDiff
{

private:
	boost::posix_time::ptime beginTime;
	boost::posix_time::ptime endTime;
public:
	cTimerDiff(void);
	~cTimerDiff(void);

	void Begin();
	float End();

	float GetTime() const;
	void PrintString();
};

#endif // _TIMERDIFF_KYOSHO_20110903_