#ifndef _TRANSFORMATION_H_
#define _TRANSFORMATION_H_

class Transformation {
private:
	float angularVelocity;
	float relativeAcceleration;
public:	
	Transformation();
	
	float getRotationalAngle(float angularVelocity);
	float integration(float data, float dt);
	float getAngularVelocity();
	
	
	float getGyrox(float angularVelocity);
	float getGyroy(float angularVelocity);
	float getGyroz(float angularVelocity);
	
	float getAccelx(float relativeAcceleration);
	float getAccely(float relativeAcceleration);
	float getAccelz(float relativeAcceleration);
	
	float transformToNormal(float** transformationMatrix, float relativeAcceleration);

};


#endif
