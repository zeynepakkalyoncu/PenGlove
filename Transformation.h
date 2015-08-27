#ifndef _TRANSFORMATION_H_
#define _TRANSFORMATION_H_

#include <vector>

class Transformation {
public:
	double angularVelocity;
	double relativeAcceleration;

	//	vector<vector<double> > matrix;

	double currentAngle;

	double velocityX;
	double velocityY;
	double velocityZ;

	double posX;
	double posY;
	double posZ;
	//public:	
	Transformation();

	void cancelGravity(std::vector<double> realAcceleration);
	double getRotationAngle(double angularVelocity, double dt);
	std::vector<std::vector<double> > getTransformationMatrix(double gyroXrate, double gyroYrate, double gyroZrate, double dt);
	std::vector<double> transformToNormal(double kalAngleX, double kalAngleY, double kalAngleZ, double gyroXrate, double gyroYrate, double gyroZrate, double dt);
	double getAbsoluteVelocity(double kalAngleX, double kalAngleY, double kalAngleZ, double gyroXrate, double gyroYrate, double gyroZrate, double dt, int axis);
	double getAbsolutePosition(double kalAngleX, double kalAngleY, double kalAngleZ, double gyroXrate, double gyroYrate, double gyroZrate, double dt, int axis);
};


#endif
