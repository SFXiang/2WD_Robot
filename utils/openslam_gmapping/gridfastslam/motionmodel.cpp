#include <gmapping/gridfastslam/motionmodel.h>
#include <gmapping/utils/stat.h>
#include <iostream>

#define MotionModelConditioningLinearCovariance 0.01
#define MotionModelConditioningAngularCovariance 0.001

namespace GMapping {



OrientedPoint 
MotionModel::drawFromMotion (const OrientedPoint& p, double linearMove, double angularMove) const{
	OrientedPoint n(p);
	double lm=linearMove  + fabs( linearMove ) * sampleGaussian( srr ) + fabs( angularMove ) * sampleGaussian( str );
	double am=angularMove + fabs( linearMove ) * sampleGaussian( srt ) + fabs( angularMove ) * sampleGaussian( stt );
	n.x+=lm*cos(n.theta+.5*am);
	n.y+=lm*sin(n.theta+.5*am);
	n.theta+=am;
	n.theta=atan2(sin(n.theta), cos(n.theta));
	return n;
}

OrientedPoint 
MotionModel::drawFromMotion(const OrientedPoint& p, const OrientedPoint& pnew, const OrientedPoint& pold) const{
	double sxy=0.3*srr;//我理解是xy之间的耦合误差。
	OrientedPoint delta=absoluteDifference(pnew, pold);//计算前后两个时刻位姿差 delta
	OrientedPoint noisypoint(delta);//对delta 叠加噪声
	//按着pf中的做法下一时刻的该粒子的位姿是当前时刻的位姿叠加一个随机高斯噪声，这里分别还叠加了平移和旋转误差
	noisypoint.x+=sampleGaussian(srr*fabs(delta.x)+str*fabs(delta.theta)+sxy*fabs(delta.y));
	noisypoint.y+=sampleGaussian(srr*fabs(delta.y)+str*fabs(delta.theta)+sxy*fabs(delta.x));
	noisypoint.theta+=sampleGaussian(stt*fabs(delta.theta)+srt*sqrt(delta.x*delta.x+delta.y*delta.y));
	noisypoint.theta=fmod(noisypoint.theta, 2*M_PI);
	if (noisypoint.theta>M_PI)
		noisypoint.theta-=2*M_PI;//将角度限定在-pi-pi之间。
	return absoluteSum(p,noisypoint);//由p也即t-1时刻该粒子的位姿叠加噪声得到t时刻的位姿
}


/*
OrientedPoint 
MotionModel::drawFromMotion(const OrientedPoint& p, const OrientedPoint& pnew, const OrientedPoint& pold) const{
	
	//compute the three stps needed for perfectly matching the two poses if the noise is absent
	
	OrientedPoint delta=pnew-pold;
	double aoffset=atan2(delta.y, delta.x);
	double alpha1=aoffset-pold.theta;
	alpha1=atan2(sin(alpha1), cos(alpha1));
	double rho=sqrt(delta*delta);
	double alpha2=pnew.theta-aoffset;
	alpha2=atan2(sin(alpha2), cos(alpha2));
	
	OrientedPoint pret=drawFromMotion(p, 0, alpha1);
	pret=drawFromMotion(pret, rho, 0);
	pret=drawFromMotion(pret, 0, alpha2);
	return pret;
}
*/


Covariance3 MotionModel::gaussianApproximation(const OrientedPoint& pnew, const OrientedPoint& pold) const{
	OrientedPoint delta=absoluteDifference(pnew,pold);
	double linearMove=sqrt(delta.x*delta.x+delta.y*delta.y);
	double angularMove=fabs(delta.x);
	double s11=srr*srr*linearMove*linearMove;
	double s22=stt*stt*angularMove*angularMove;
	double s12=str*angularMove*srt*linearMove;
	Covariance3 cov;
	double s=sin(pold.theta),c=cos(pold.theta);
	cov.xx=c*c*s11+MotionModelConditioningLinearCovariance;
	cov.yy=s*s*s11+MotionModelConditioningLinearCovariance;
	cov.tt=s22+MotionModelConditioningAngularCovariance;
	cov.xy=s*c*s11;
	cov.xt=c*s12;
	cov.yt=s*s12;
	return cov;
}

};

