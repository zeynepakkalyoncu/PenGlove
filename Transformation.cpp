#include "Transformation.h"
#include "Integration.h"
#include <cmath>

Transformation::Transformation(){
	
}

// TODO: check positive/negative orientations
float** Transformation::getTransformationMatrix(float* angularVelocity){
	float gyro_x = getGyrox(angularVelocity);
	float gyro_y = getGyroy(angularVelocity);
	float gyro_z = getGyroz(angularVelocity);
	float alpha = getRotationAngle(gyro_x);
	float beta = getRotationAngle(gyro_y);
	float gamma = getRotationAngle(gyro_z);
	float[3][3] matrix = {{cos(beta)*cos(gamma), -sin(gamma), sin(beta)},
						{sin(gamma), cos(alpha)*cos(gamma), -sin(alpha)},
						{-sin(beta), sin(alpha), cos(alpha)*cos(beta)}};
	return matrix;
}

float Transformation::getRotationAngle(float angularVelocity){
	float angle = integrate(angularVelocity, dt);
	return angle;
}

float* Transformation::transformToNormal(float* relativeAcceleration, float* angularVelocity){
	float** transformationMatrix = getTransformation(angularVelocity);
	float absoluteAcceleration[3];
	for (int i = 0; i < 3; i++){
		absoluteAcceleration[i] = absoluteAcceleration[0] * transformationMatrix[0][i]
								+ absoluteAcceleration[1] * transformationMatrix[1][i]
								+ absoluteAcceleration[2] * transformationMatrix[2][i];	
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
