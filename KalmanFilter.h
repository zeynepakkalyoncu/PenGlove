#ifndef _KalmanFilter_h_
#define _KalmanFilter_h_

class KalmanFilter {
private:
	// variables for the Kalman filter
	float Q_angle; // process noise var for accel
	float Q_bias; // process noise var for gyro
	float R_measure; // measurement noise var
	float P[2][2]; // error covariance P_2x2
	
	// estimated by the Kalman filter
	float angle; // x_k[0]
	float bias; // x_k[1]
	float unbiasedRate;
		
public:
	KalmanFilter();
	
	// getter methods
	float getAngle(float newAngle, float newRate, float dt);
	float getUnbiasedRate();
	float getQangle();
	float getQbias();
	float getRmeasure();
	
	// setter methods
	void setStartingAngle (float angle); 
	
	// used to fine-tune the filter
	void setQangle(float Q_angle);
	void setQbias(float Q_bias);
	void setRmeasure(float R_measure);
};

#endif
