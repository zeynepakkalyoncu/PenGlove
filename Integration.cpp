#include <cmath>
#include <iostream>

using namespace std;

struct State{
	float position;
	float velocity;
};

struct Derivative{
	float dx;
	float dv;
};

float acceleration(const State &state){
	const float k = 10;
	const float b = 1;
	return - k*state.position - b*state.velocity;
}

Derivative evaluate(const State& initial, float time){
	Derivative output;
	output.dx = initial.velocity;
	output.dv = acceleration(initial, time);
	return output;
}

Derivative evaluate(const State& initial, float time, float dt, const Derivative& d){
	State state;
	state.position = initial.position + d.dx*dt;
	state.velocity = initial.velocity + d.dv*dt;
	Derivative output;
	output.dx = state.velocity;
	output.dv = acceleration(state, time + dt);
	return output;
}

void integrate(State& state, float t, float dt){
	Derivative a = evaluate(state, t);
	Derivative b = evaluate(state, t, dt*0.5f, a);
	Derivative c = evaluate(state, t, dt*0.5f, b);
	Derivative d = evaluate(state, t, dt, c);
	
	const float dxdt = 1.0f/6.0f * (a.dx + 2.0f*(b.dx + c.dx) + d.dx);
	const float dvdt = 1.0f/6.0f * (a.dv + 2.0f*(b.dv + c.dv) + d.dv);
	
	state.position += dxdt*dt;
	state.velocity += dvdt*dt;
}

int main(){
	State state;
	
	cin >> state.position;
	cin >> state.velocity;
	
	float t = 0;
	float dt = 0.1f;
	
	while (fabs(state.position) > 0.001f || fabs(state.velocity) > 0.001f){
		std::cout << state.position << " " << state.velocity << endl;
		integrate(state, t, dt);
		t += dt;
	}
	
	return 0;
	
}
