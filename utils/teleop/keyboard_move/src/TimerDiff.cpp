#include "TimerDiff.h"

cTimerDiff::cTimerDiff(void)
{
}

cTimerDiff::~cTimerDiff(void)
{
}

void cTimerDiff::Begin() {
	boost::posix_time::ptime t(
		boost::posix_time::microsec_clock::local_time());
	beginTime = t;
}

float cTimerDiff::End() {
	boost::posix_time::ptime t(
		boost::posix_time::microsec_clock::local_time());
	endTime = t;

	boost::posix_time::time_duration t3 = (endTime - beginTime);

	return t3.total_microseconds();

}

float cTimerDiff::GetTime() const{
	boost::posix_time::ptime t(
		boost::posix_time::microsec_clock::local_time());
	boost::posix_time::time_duration t3 = (t - beginTime);
	
	return t3.total_microseconds();
}

void cTimerDiff::PrintString(){
	//printf("@Use Time = %f\n",GetTime());
}
