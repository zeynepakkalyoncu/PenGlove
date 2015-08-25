#include "Transformation.h"
#include "Integration.h"
#include <cmath>

Transformation::Transformation(){
	
}

float Transformation::integrate(){
	
	
}

// TODO: check positive/negative orientations
double** Transformation::getTransformationMatrix(double gyroXrate, double gyroYrate, double gyroZrate, double dt){
	double alpha = getRotationAngle(gyroXrate, dt);
	double beta = getRotationAngle(gyroYrate, dt);
	double gamma = getRotationAngle(gyroZrate, dt);
	double[3][3] matrix = {{cos(beta)*cos(gamma), -sin(gamma), sin(beta)},
						{sin(gamma), cos(alpha)*cos(gamma), -sin(alpha)},
						{-sin(beta), sin(alpha), cos(alpha)*cos(beta)}};
	return matrix;
}

double Transformation::getRotationAngle(double angularVelocity double dt){
	double angle = integrate(angularVelocity, dt); // integrate parameters missing
	return angle;
}

double* Transformation::transformToNormal(double kalAngleX, double kalAngleY, double kalAngleZ, double gyroXrate, double gyroYrate, double gyroZrate, double dt){
	double** transformationMatrix = getTransformationMatrix(angularVelocity);
	double absoluteAcceleration[3];
	for (int i = 0; i < 3; i++){
		absoluteAcceleration[i] = kalAngleX * transformationMatrix[0][i]
								+ kalAngleY * transformationMatrix[1][i]
								+ kalAngleZ * transformationMatrix[2][i];	
	}
	cancelGravity(absoluteAcceleration);
	return absoluteAcceleration;
}

float Transformation::getAbsoluteVelocity(){
	float absoluteVelocity = integrate(transformToNormal(relativeAcceleration, angularVelocity));
	return absoluteVelocity;
	}
	
float Transformation::getAbsolutePosition(){
	float absolutePosition = integrate(getAbsoluteVelocity()); 
	return absolutePosition;
	}

void Transformation::cancelGravity(float* realAcceleration){
	realAcceleration[2] -= 9.81;
}

float* Transformation::getAngularVelocity(){
	return this->angularVelocity;
}

float Transformation::getGyrox(){
	return this->angularVelocity[0];
}

float Transformation::getGyroy(){
	return this->angularVelocity[1];
}

float Transformation::getGyroz(){
	return this->angularVelocity[2];
}
