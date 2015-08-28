#include "Transformation.h"
#include <cmath>
#include <vector>

void Transformation::cancelGravity(std::vector<double> realAcceleration) {
	realAcceleration[2] -= 9.81;
}
/*
double Transformation::getRotationAngle(double angularVelocity, double dt) {
	currentAngle += angularVelocity * dt;
	return currentAngle;
}
*/
void Transformation::getTransformationMatrix(double gyroXangle, double gyroYangle, double gyroZangle, double dt) {
/*	double alpha = getRotationAngle(gyroXrate, dt);
	double beta = getRotationAngle(gyroYrate, dt);
	double gamma = getRotationAngle(gyroZrate, dt);
	matrix[0][0] = cos(beta)*cos(gamma);
	matrix[0][1] = -sin(gamma);
	matrix[0][2] = sin(beta);
	matrix[1][0] = sin(gamma);
	matrix[1][1] = cos(alpha)*cos(gamma);
	matrix[1][2] = -sin(alpha);
	matrix[2][0] = -sin(beta);
	matrix[2][1] = sin(alpha);
	matrix[2][2] = cos(alpha)*cos(beta);
	* */
	matrix[0][0] = cos(gyroYangle)*cos(gyroZangle);
	matrix[0][1] = -sin(gyroZangle);
	matrix[0][2] = sin(gyroYangle);
	matrix[1][0] = sin(gyroZangle);
	matrix[1][1] = cos(gyroXangle)*cos(gyroZangle);
	matrix[1][2] = -sin(gyroXangle);
	matrix[2][0] = -sin(gyroYangle);
	matrix[2][1] = sin(gyroXangle);
	matrix[2][2] = cos(gyroXangle)*cos(gyroYangle);
	
}

std::vector<double> Transformation::transformToNormal(double kalAngleX, double kalAngleY, double kalAngleZ, double gyroXangle, double gyroYangle, double gyroZangle, double dt) {
	getTransformationMatrix(gyroXangle, gyroYangle, gyroZangle, dt);
	std::vector<double> absoluteAcceleration(3);
	for (int i = 0; i < 3; i++) {
		absoluteAcceleration[i] = kalAngleX * matrix[0][i]
			+ kalAngleY * matrix[1][i]
			+ kalAngleZ * matrix[2][i];
	}
	cancelGravity(absoluteAcceleration);
	return absoluteAcceleration;
}

double Transformation::getAbsoluteVelocity(double kalAngleX, double kalAngleY, double kalAngleZ, double gyroXangle, double gyroYangle, double gyroZangle, double dt, int axis) {
	std::vector<double> absoluteAcceleration = transformToNormal(kalAngleX, kalAngleY, kalAngleZ, gyroXangle, gyroYangle, gyroZangle, dt);
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

double Transformation::getAbsolutePosition(double kalAngleX, double kalAngleY, double kalAngleZ, double gyroXangle, double gyroYangle, double gyroZangle, double dt, int axis) {
	std::vector<double> absoluteAcceleration = transformToNormal(kalAngleX, kalAngleY, kalAngleZ, gyroXangle, gyroYangle, gyroZangle, dt);
	velocityX += absoluteAcceleration[0] * dt / 1000;
	velocityY += absoluteAcceleration[1] * dt / 1000;
	velocityZ += absoluteAcceleration[2] * dt / 1000;
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
