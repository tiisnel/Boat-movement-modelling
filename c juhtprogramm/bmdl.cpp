/*
 * author Johannes Ehala
 * 11.03.2019
 */

#include <math.h>
#include "bparams.h"

struct state_vector
{
	float x;	//x position
	float y;	//y position
	float t;	//theta position
	float vx;	//x speed
	float vy;	//y speed
	float w;	//theta speed
};

class bmodel
{
	bool init_done;
	float x,y,vx,vy,t,w;	//boat state
	float timestep; 	//time granualrity for simulation, seconds
	
	public:
	int init(state_vector init, float ts);

	//typically one might use reset in the middle of simulation to changes 
	//time granualarity, while not changing boat state, example:
	//
	//nstate = next_position(F1, F2);
	//reset(nstate, NEW_TIME_STEP);	
	int reset(state_vector init, float ts);

	state_vector next_position(float F1, float F2);
};

int bmodel::init(state_vector init, float ts)
{
	//can be called only once, then must use reset() function
	if(!init_done)
	{
		x = init.x;
		vx = init.vx;
		y = init.y;
		vy = init.vy;
		t = init.t;
		w = init.w;
		timestep = ts;
		init_done = true;
		return 0;
	}	
	return 1;
}

int bmodel::reset(state_vector init, float ts)
{
	//there is no guard or notification here, user must take care when using reset
	x = init.x;
	vx = init.vx;
	y = init.y;
	vy = init.vy;
	t = init.t;
	w = init.w;
	timestep = ts;
	return 0;
}

state_vector bmodel::next_position(float F1, float F2) 
{
	state_vector new_state;
	float F, Ft, b, c, d, e, f, h; //intermediary constants to simplyfy code and readability, not 
	
	//Runge Kutta 4 slopes for each curve
	float k1, k2, k3, k4; //theta
	float l1, l2, l3, l4; //thetadot
	float m1, m2, m3, m4; //xi
	float n1, n2, n3, n4; //xidot
	float o1, o2, o3, o4; //psi
	float p1, p2, p3, p4; //psidot

	/*****************************************************
	 *		Initialize constants and variables
	 *****************************************************/
	
	//if initial conditions are not set, can't do anything
	if(!init_done)	
	{
		//unhappy ending
		new_state.x = 0;
		new_state.vx = 0;
		new_state.y = 0;
		new_state.vy = 0;
		new_state.t = 0;
		new_state.w = 0;
		return new_state;
	}

	//can't exert more force on the system than is allowed!
	if(F1 > BP_MAX_INPUT)F1 = BP_MAX_INPUT;
	if(F2 > BP_MAX_INPUT)F2 = BP_MAX_INPUT;

	//to make code more compact, we find an intermediate value here
	F = F1 + F2 - fabs(F1-F2);

	/*****************************************************
	 *		Solve full system differential equations
	 *****************************************************/
	
	//Here we use Runge Kutta forth order method for solving a system of differential
	//equations. The equations are explained and displayed in the document ENTER_NAME_HERE.

	/////////////////////////////////////////////////////////////////////////
	//Slope estimations using initial values (x, xv, y, vy, theta, omega)
	/////////////////////////////////////////////////////////////////////////

	k1 = vx;
	l1 = F*cos(t*M_PI/180)/BP_MASS;
	if(l1 != 0)Ft = BP_CT*vx/BP_MASS;
	else
	{
		if(vx >= BP_STOP_SPEED)Ft = BP_CT*vx/BP_MASS;
		else Ft = vx;
	}
	l1 -= Ft;

	m1 = vy;
	n1 = F*sin(t*M_PI/180)/BP_MASS;
	if(n1 != 0)Ft = BP_CT*vy/BP_MASS;
	else
	{
		if(vy >= BP_STOP_SPEED)Ft = BP_CT*vy/BP_MASS;
		else Ft = vy;
	}
	n1 -= Ft;

	o1 = w;
	p1 = (F2 - F1)*BP_OAR_DIST/BP_BOAT_INER;
	if(p1 > 1)Ft = BP_CTN*w*BP_OAR_DIST/BP_BOAT_INER;
	else
	{
		if(w >= 1)Ft = BP_CTN*w*BP_OAR_DIST/BP_BOAT_INER;
		else Ft = w;
	}
	p1 -= Ft;

	///////////////////////////////////////////////
	//Curve estimations at halfway through timestep using previous 
	//slope estimations (k1, l1, m1, n1, o1, p1).
	///////////////////////////////////////////////

	b = x + k1*timestep/2; 		//new x
	c = vx + l1*timestep/2; 	//new vx
	d = y + m1*timestep/2; 		//new y
	e = vy + n1*timestep/2; 	//new vy
	f = t + o1*timestep/2; 		//new theta
	h = w + p1*timestep/2; 		//new angular speed omega

	///////////////////////////////////////////////
	//New slope estimation using previous halfway through
	//timestep curve estimations
	///////////////////////////////////////////////

	k2 = c;
	l2 = F*cos(f*M_PI/180)/BP_MASS;
	if(l2 != 0)Ft = BP_CT*c/BP_MASS;
	else
	{
		if(c >= BP_STOP_SPEED)Ft = BP_CT*c/BP_MASS;
		else Ft = c;
	}
	l2 -= Ft;

	m2 = e;
	n2 = F*sin(f*M_PI/180)/BP_MASS;
	if(n2 != 0)Ft = BP_CT*e/BP_MASS;
	else
	{
		if(e >= BP_STOP_SPEED)Ft = BP_CT*e/BP_MASS;
		else Ft = e;
	}
	n2 -= Ft;

	o2 = h;
	p2 = (F2 - F1)*BP_OAR_DIST/BP_BOAT_INER;
	if(p2 > 1)Ft = BP_CTN*h*BP_OAR_DIST/BP_BOAT_INER;
	else
	{
		if(h >= 1)Ft = BP_CTN*h*BP_OAR_DIST/BP_BOAT_INER;
		else Ft = h;
	}
	p2 -= Ft;

	///////////////////////////////////////////////
	//Curve estimations at halfway through timestep using previous 
	//slope estimations (k2, l2, m2, n2, o2, p2).
	///////////////////////////////////////////////

	b = x + k2*timestep/2; 		//new x
	c = vx + l2*timestep/2; 	//new vx
	d = y + m2*timestep/2; 		//new y
	e = vy + n2*timestep/2; 	//new vy
	f = t + o2*timestep/2; 		//new theta
	h = w + p2*timestep/2; 		//new angular speed omega

	///////////////////////////////////////////////
	//New slope estimation using previous halfway through
	//timestep curve estimations
	///////////////////////////////////////////////

	k3 = c;
	l3 = F*cos(f*M_PI/180)/BP_MASS;
	if(l3 != 0)Ft = BP_CT*c/BP_MASS;
	else
	{
		if(c >= BP_STOP_SPEED)Ft = BP_CT*c/BP_MASS;
		else Ft = c;
	}
	l3 -= Ft;

	m3 = e;
	n3 = F*sin(f*M_PI/180)/BP_MASS;
	if(n3 != 0)Ft = BP_CT*e/BP_MASS;
	else
	{
		if(e >= BP_STOP_SPEED)Ft = BP_CT*e/BP_MASS;
		else Ft = e;
	}
	n3 -= Ft;

	o3 = h;
	p3 = (F2 - F1)*BP_OAR_DIST/BP_BOAT_INER;
	if(p3 > 1)Ft = BP_CTN*h*BP_OAR_DIST/BP_BOAT_INER;
	else
	{
		if(h >= 1)Ft = BP_CTN*h*BP_OAR_DIST/BP_BOAT_INER;
		else Ft = h;
	}
	p3 -= Ft;

	///////////////////////////////////////////////
	//Curve estimations after full timestep using previous 
	//slope estimations (k3, l3, m3, n3, o3, p3).
	///////////////////////////////////////////////

	b = x + k3*timestep; 		//new x
	c = vx + l3*timestep; 		//new vx
	d = y + m3*timestep; 		//new y
	e = vy + n3*timestep; 		//new vy
	f = t + o3*timestep;		//new theta
	h = w + p3*timestep; 		//new angular speed omega

	///////////////////////////////////////////////
	//New slope estimation using previous halfway through
	//timestep curve estimations
	///////////////////////////////////////////////

	k4 = c;
	l4 = F*cos(f*M_PI/180)/BP_MASS;
	if(l4 != 0)Ft = BP_CT*c/BP_MASS;
	else
	{
		if(c >= BP_STOP_SPEED)Ft = BP_CT*c/BP_MASS;
		else Ft = c;
	}
	l4 -= Ft;

	m4 = e;
	n4 = F*sin(f*M_PI/180)/BP_MASS;
	if(n4 != 0)Ft = BP_CT*e/BP_MASS;
	else
	{
		if(e >= BP_STOP_SPEED)Ft = BP_CT*e/BP_MASS;
		else Ft = e;
	}
	n4 -= Ft;

	o4 = h;
	p4 = (F2 - F1)*BP_OAR_DIST/BP_BOAT_INER;
	if(p4 > 1)Ft = BP_CTN*h*BP_OAR_DIST/BP_BOAT_INER;
	else
	{
		if(h >= 1)Ft = BP_CTN*h*BP_OAR_DIST/BP_BOAT_INER;
		else Ft = h;
	}
	p4 -= Ft;

	/////////////////////////////////////////////////
	//Final curve estimations after timestep
	/////////////////////////////////////////////////

	x = x + (k1+2*k2+2*k3+k4)*timestep/6; 		//new x
	vx = vx + (l1+2*l2+2*l3+l4)*timestep/6; 	//new vx
	y = y + (m1+2*m2+2*m3+m4)*timestep/6; 		//new y
	vy = vy + (n1+2*n2+2*n3+n4)*timestep/6; 	//new vy
	t = t + (o1+2*o2+2*o3+o4)*timestep/6;		//new theta
	w = w + (p1+2*p2+2*p3+p4)*timestep/6; 		//new angular speed omega

	//happy ending
	new_state.x = x;
	new_state.vx = vx;
	new_state.y = y;
	new_state.vy = vy;
	new_state.t = t;
	new_state.w = w;

	return new_state;
}
