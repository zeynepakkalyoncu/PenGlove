#include "Transformation.h"
#include <cmath>
#include <vector>

Transformation::Transformation(){
	velocityX = 0;
	velocityY = 0;
}

void Transformation::cancelGravity(std::vector<double> realAcceleration) {
	realAcceleration[2] -= 9.81;
}

double Transformation::getRotationAngle(double angularVelocity, double dt) {
	currentAngle += angularVelocity * dt;
	return currentAngle;
}

std::vector<std::vector<double> > Transformation::getTransformationMatrix(double gyroXrate, double gyroYrate, double gyroZrate, double dt) {
	double alpha = getRotationAngle(gyroXrate, dt);
	double beta = getRotationAngle(gyroYrate, dt);
	double gamma = getRotationAngle(gyroZrate, dt);
	std::vector<std::vector<double> > matrix = { { cos(beta)*cos(gamma), -sin(gamma), sin(beta) },
	{ sin(gamma), cos(alpha)*cos(gamma), -sin(alpha) },
	{ -sin(beta), sin(alpha), cos(alpha)*cos(beta) } };
	return matrix;
}

std::vector<double> Transformation::transformToNormal(double kalAngleX, double kalAngleY, double kalAngleZ, double gyroXrate, double gyroYrate, double gyroZrate, double dt) {
	std::vector<std::vector<double> > transformationMatrix = getTransformationMatrix(gyroXrate, gyroYrate, gyroZrate, dt);
	std::vector<double> absoluteAcceleration;
	for (int i = 0; i < 3; i++) {
		absoluteAcceleration[i] = kalAngleX * transformationMatrix[0][i]
			+ kalAngleY * transformationMatrix[1][i]
			+ kalAngleZ * transformationMatrix[2][i];
	}
	cancelGravity(absoluteAcceleration);
	return absoluteAcceleration;
}

double Transformation::getAbsoluteVelocity(double kalAngleX, double kalAngleY, double kalAngleZ, double gyroXrate, double gyroYrate, double gyroZrate, double dt, int axis) {
	std::vector<double> absoluteAcceleration = transformToNormal(kalAngleX, kalAngleY, kalAngleZ, gyroXrate, gyroYrate, gyroZrate, dt);
	velocityX += absoluteAcceleration[0] * dt;
	velocityY += absoluteAcceleration[1] * dt;
	velocityZ += absoluteAcceleration[2] * dt;
	switch (axis) {
	case 1:
		return velocityX;
	case 2:
		return velocityY;
	case 3:
		return velocityZ;
	default:
		return 0;
	}
}

double Transformation::getAbsolutePosition(double kalAngleX, double kalAngleY, double kalAngleZ, double gyroXrate, double gyroYrate, double gyroZrate, double dt, int axis) {
	std::vector<double> absoluteAcceleration = transformToNormal(kalAngleX, kalAngleY, kalAngleZ, gyroXrate, gyroYrate, gyroZrate, dt);
	velocityX += absoluteAcceleration[0] * dt;
	velocityY += absoluteAcceleration[1] * dt;
	velocityZ += absoluteAcceleration[2] * dt;
	posX += velocityX * dt;
	posY += velocityY * dt;
	posZ += velocityZ * dt;
	switch (axis) {
	case 1:
		return posX;
	case 2:
		return posY;
	case 3:
		return posZ;
	default:
		return 0;
	}
}
