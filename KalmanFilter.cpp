#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(){
	// taken from TJKElectronics
	// TODO: check values for precision
	Q_angle = 0.001f;
	Q_bias = 0.003f;
	R_measure = 0.03f;
	
	// calibration
	angle = 0.0f;
	bias = 0.0f;
	P[2][2] = {0};
};

// getter methods
float KalmanFilter::getAngle(float angleInput, float rateInput, float dt){
	float y; // innovation
	float S; // innovation covariance (i.e: how much to trust incoming measurement)
	float K[2]; // Kalman gain (i.e: how much to trust innovation)
	float temp1, temp2;
		
	// PREDICT
	
	// ...current state wrt previous states and bias
	unbiasedRate = rateInput - bias;
	angle += dt * unbiasedRate;
	// TODO: check matrix multiplication methods
	// ...current error covariance wrt its previous value
	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Q_bias * dt;
	
	// UPDATE 
	
	// ...innovation
	y = angleInput - angle;
	// ...innovation covariance
	S = P[0][0] + R_measure;	
	// ...Kalman gain
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;	
	// ...estimated measurements
	angle += K[0] * y;
	bias += K[1] * y;
	// ...error covariance
	temp1 = P[0][0]; temp2 = P[0][1];	
	P[0][0] -= K[0] * temp1;
	P[0][1] -= K[0] * temp2;
	P[1][0] -= K[1] * temp1;
	P[1][1] -= K[1] * temp2;
	
	return angle;
	
	// TODO: also apply high-pass filter to block out low frequency noise?
};

float KalmanFilter::getUnbiasedRate(){
	return this->unbiasedRate;
};

float KalmanFilter::getQangle(){
	return this->Q_angle;
};

float KalmanFilter::getQbias(){
	return this->Q_bias;
};

float KalmanFilter::getRmeasure(){
	return this->R_measure;
};

// setter methods
void KalmanFilter::setStartingAngle(float angle) {
	this->angle = angle;
};

// used to fine-tune the filter
void KalmanFilter::setQangle(float Q_angle){
	this->Q_angle = Q_angle;
};

void KalmanFilter::setQbias(float Q_bias){
	this->Q_bias = Q_bias;
};

void KalmanFilter::setRmeasure(float R_measure){
	this->R_measure = R_measure;
};
