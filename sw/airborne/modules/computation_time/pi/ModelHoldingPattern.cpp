/*
 * ModelHoldingPattern.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: vgomez, Sep Thijssen
 */

#ifndef MODEL_HOLDINGPATTERN_H_
#define MODEL_HOLDINGPATTERN_H_

#include "PIController.h"

using namespace std;

// nested class Model used by the sampler to simulate dynamics
class ModelHoldingPattern: public PIController::Model {
protected:
	double x;		// coordinate x of the center of the holding pattern
	double y;		// coordinate y of the center of the holding pattern
	double radius;	// radious of the holding pattern
	double minv;	// determines min velocity
	double maxv;	// determines max velocity
	double hit;		// penalty constant for collision

public:

	// apply control A on the current state
	void step(const vec &A, const  int& flag) {

		if (flag == 0 ) {

			// active unit
			state_active[2] += A[0]*PIController::dh;		//  X velocity
			state_active[3] += A[1]*PIController::dh;		//  Y velocity
			state_active[0] += state_active[2]*PIController::dh;	//  X position
			state_active[1] += state_active[3]*PIController::dh;	//  Y position

			// for the rest of the units just keep constant speed
			for(int i=0; i<PIController::units-1; i++) {
				state_inactive[4*i+0] += state_inactive[4*i+2]*PIController::dh;	//  X position
				state_inactive[4*i+1] += state_inactive[4*i+3]*PIController::dh;	//  Y position
			}
		} else {
				//std::cout << "Controls" << A << endl;
				//std::cout << "States initial" << state << endl;
				for(int i=0; i<PIController::units; i++) {

					state[4*i+2] += A[2*i+0]*PIController::dt;		//  X velocity
					state[4*i+3] += A[2*i+1]*PIController::dt;		//  Y velocity

					state[4*i+0] += state[4*i+2]*PIController::dt;	//  X position
					state[4*i+1] += state[4*i+3]*PIController::dt;	//  Y position
				}
				//std::cout << "States after" << state << endl;
		}

	}

	// Output is the immediate state reward. Input is a state.
	double immediateStateReward() const {
		//cout << "in ModelHoldingPattern::inmediateStateReward()" << endl;
		// Immediate state reward of the state X.
		// Returns the negative of the cost c in this case.
		double c=0;
		// cost per unit
		//for(int i=0; i<PIController::units; i++)
		//{
			double speed = sqrt(state_active[2]*state_active[2] + state_active[3]*state_active[3]);
			//double speed = sqrt(state[2]*state[2] + state[3]*state[3]);
			// penalty for low or high speeds
			c += exp(speed-maxv); 		// determines max allowed speed
			c += exp(-speed+minv);		// determines min allowed speed

			// penalty for going to far away from the center (x, y)
			double d;
			d = sqrt( (state_active[0]-x)*(state_active[0]-x) + (state_active[1]-y)*(state_active[1]-y) );
			//std::cout << "distance from RADIUS" <<  d << "cost " << exp(d - radius) << endl;
			c += 100*exp(d - radius);				// max allowed distance ~= radius


			//for(int j=u+1; j<PIController::units; j++)
			//{	d=0;
			//	d+=(state[4*u+0]-state[4*j+0])*(state[4*u+0]-state[4*j+0]);
			///	d+=(state[4*u+1]-state[4*j+1])*(state[4*u+1]-state[4*j+1]);
			//	if (0.00001 > d) d=0.00001;
			//	c += hit*1/d;
			//}
			for (int i = 0; i < state_inactive.size()/4; i++) {
				d = 0;
				d+=(state_active[0]-state_inactive[4*i])*(state_active[0]-state_inactive[4*i]);
				d+=(state_active[1]-state_inactive[4*i+1])*(state_active[1]-state_inactive[4*i+1]);
				if (0.00001 > d) d=0.00001;
			}
				c += (hit*1)/sqrt(d);
				//std::cout << "collision penalty" << (hit*1)/sqrt(d) << endl;
		//}

		return c; //return the cost  //-c;
	}

	virtual void setProperties()
	{
		cout << " setting properties" << endl;
		x = ctrl->getProperties().get<double>("holdingp.x");
		y = ctrl->getProperties().get<double>("holdingp.y");
		minv = ctrl->getProperties().get<double>("holdingp.minv");
		maxv = ctrl->getProperties().get<double>("holdingp.maxv");
		radius = ctrl->getProperties().get<double>("holdingp.radius");
		hit = ctrl->getProperties().get<double>("holdingp.hit");
	}

};

void PIController::Sampler::setModel()
{
	cout << "setting model for ModelHoldingPattern" << endl;
	Model *m = new ModelHoldingPattern();
	model = m;
}

void PIController::Sampler::unsetModel()
{
	cout << "unsetting model for ModelHoldingPattern" << endl;
	delete model;
}

#endif /* MODEL_HOLDINGPATTERN_H_ */
