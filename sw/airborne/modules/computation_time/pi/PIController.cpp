/*
 * PIController.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: vgomez, Sep Thijssen
 */


#include "PIController.h"
#include <math.h>

//#define OLD_CONTROL 1
#define PI 3.14159265
using namespace std;

// static members initialization
int PIController::T = 0;
double PIController::iT = .0;
double PIController::dt = .0;
int PIController::units = 0;
double PIController::R = .0;
double PIController::nu = .0;
double PIController::lambda = .0;
int PIController::dtperstep = 0;
int PIController::H = 0;
double PIController::dh = .0;
double PIController::iH = 0.;
double PIController::dS = 0.;
double PIController::stdv = 0.;
int PIController::dimUAVx = 0.;
int PIController::dimUAVu = 0.;
int PIController::seed = 0;

////////////////////////////////////////////////
// Model methods
//	step() and immediateStateReward() to be defined in the concrete Model class
//

double PIController::Model::immediateControlCost( const vec& A) const
{
	// Immediate control cost of the control action A.
	double v=0;
	v += R*A[0]*A[1];
	return v*.5;
}

//for(unsigned int i=0; i<A.size(); i++) {
		// v += R[i]*A[i]*A[i];
		//v += R*A[i]*A[i];
	//}
////////////////////////////////////////////////
// Sampler methods
//

double PIController::Sampler::runningStateReward(
		const vec& X0_active,
		const vec& X0_inactive,
		const vvec& control
) {
	// Returns state based reward of a rollout.
	model->setStateActive(X0_active);
	model->setStateInactive(X0_inactive);

	double v=0;
	int flag = 0;
	//for (int s=0; s<H; s++) {
		//for (int i=0; i<dtperstep; i++){
	for (int i = 0; i < iH; i++) {
		v += model->immediateStateReward();//*dt;
		model->step(control[i], flag);
	}
	//}
	//}
	//v += model->endStateReward();

	return v;
}

vec PIController::Sampler::predictState(const vec& X, const vec& A)
{
	// Predicts the new state after a control step with A starting in X.
	model->setState(X);
	//std::cout << model->getState() << '\n';
	//int u = 1000;
	//for (int i=0; i<dtperstep; i++)
	int flag = 1;
	model->step(A, flag);
	return model->getState();
}

double PIController::Sampler::runningControlCost( const vvec& control ) const
{
	// Returns the cost of a control sequence.
	double c=0;
	for (int s=0; s<H; s++)
		c += model->immediateControlCost(control[s]);
	return c*(dh);
}

////////////////////////////////////////////////
// PIController methods
//

void PIController::Init(const boost::property_tree::ptree &_pt)
{
	pt = _pt;

	T = pt.get<int>("T");
	dt =pt.get<double>("dt");
	R =pt.get<double>("R");
	dtperstep =pt.get<int>("dtperstep");
	H =pt.get<int>("H");
	dh =pt.get<double>("dh");
	nu =pt.get<double>("nu");
	units = pt.get<int>("units");
	dimUAVx =pt.get<int>("dimUAVx");
	dimUAVu =pt.get<int>("dimUAVu");
	seed =pt.get<int>("seed");
	N = pt.get<int>("N");

	iT = T/dt;
	iH = H/dh;
	lambda = R * nu;
	dS = 3.333;// como 1/dh dt * dtperstep;
	stdv = sqrt(nu*dh);

	dimX = (dimUAVx*units);//dimUAVx;//
	dimU = (dimUAVu*units);//dimUAVu;//
	u_exp.resize(iH, vec(dimU,0));

	gsl_rng_default_seed = seed;
	r = gsl_rng_alloc (gsl_rng_default);
	time_t timer;
	time(&timer);
	char name[50];
	sprintf(name,"test_results/experiment%d.m",(int)timer);
	outfile = name;

	plotSetup();

	// Initialize the sampler object
	sampl.Init(this);
}

PIController::~PIController() {
	// TODO Auto-generated destructor stub
}

vec PIController::InitState(){

	std::vector<double> joint_state;
	for (int j = 1; j <= units; j++) {
		double t = (double(j - 1)/units)*2* PI;
  	// x y pos
		joint_state.push_back(cos(t));
		joint_state.push_back(sin(t));
		// vx vy vel
		joint_state.push_back(cos(t));
		joint_state.push_back(sin(t));
	}
	return joint_state;
}

vec PIController::computeControl(const vec &state_active, const vec &state_inactive, vvec &u_exp_active) {

	// Input is vector of (posx, posy, velx, vely)
	vec action(dimUAVu,0);

	// Move to horizon with exploring controls
	//for (int s=1; s<H; s++) {
	//	u_exp[s-1] = u_exp[s];
	//}
	//u_exp[H-1] = vec(dimU,0);

	ofstream fout("log",ios::app);

	// Set value of exploring controls
	//double v_exp = sampl.runningStateReward(state_active, state_inactive, u_exp_active);
  //fout << v_exp << endl;
	//v_exp -= sampl.runningControlCost(u_exp_active);
	//fout << v_exp << endl;

	// define some algorithm variables:
	//double v_max = -1e100;
	double v_min = 1e100;
	vec  v_roll(N);
	vvec u_roll(iH, vec(dimUAVu));
	vvec noise(iH, vec(dimUAVu));
	vvec u_init(N, vec(dimUAVu));
	cout << "number of samples" << N << endl;
	for (int n=0; n<N; n++) {

		// set exploring noise and perturb control.
		for (int s=0; s<iH; s++) {
			for(int i=0; i<dimUAVu; i++) {
				noise[s][i] = gsl_ran_gaussian(r,stdv);
				u_roll[s][i] = u_exp_active[s][i] + noise[s][i];
			}
		}
		// save initial direction
		u_init[n] = u_roll[0];

		// set value of random control
		v_roll[n] = sampl.runningStateReward(state_active, state_inactive,u_roll);
		v_roll[n] = sampl.runningControlCost(u_roll);

		// improve exploring control if possible
		//if ( v_roll[n] > v_exp ) {
		//	v_exp = v_roll[n];
		//	u_exp = u_roll;
		//}

		// correct value of rollout for to get correct importance sampling.
		//v_roll[n] += sampl.runningControlCost(noise);

		// save max for rescaling weights
		if (v_roll[n] <  v_min) v_min = v_roll[n];
	}
	//std::cout << "V_min is " << v_min << endl;
	// PI update
	double sum1 = 0;	// sum of weights
	//double sum2 = 0;	// sum of square weights
	std::vector<double>  W(N);
	for (int n=0; n<N; n++)
	{
		//W = v_roll[n] - v_min;
		//if (W >= -20*lambda) {
		//if (lambda == 0.) W = 1;
		//else W = exp((-1*W)/lambda);
		W[n] = exp(-(v_roll[n] - v_min)/lambda);
		sum1 += W[n];

	//		for (int i=0; i<dimUAVu; i++){
	//			action[i] += u_exp_active[0][i] + W*u_init[n][i]*dS;
	//		}
		//else W = 0;
	}
	//std::cout << "sum W " << sum1 << endl;

	// Importance sampling
	for (int h = 0; h < iH; h++) {
		for (int n = 0; n < N; n++) {
			u_exp_active[h][0] += (W[n]/sum1)*noise[h][0]/dh;
			u_exp_active[h][1] += (W[n]/sum1)*noise[h][1]/dh;
		}
	}

	//std::cout << "Importance sampling " << u_exp_active << endl;
	// normalization
	for (int i=0; i<dimUAVu; i++)
	{
		action[i] = u_exp_active[1][i]/sum1;
		//action[i] /= sum1;
	}
//	cout << " end " << endl;
//	printTime();
//plotCurrent(state_active,action);

	//Importance sampling



	return action;
}


void PIController::printTime() const       // GENERAL
{
	clock_t t=clock();
	clock_t s=t/CLOCKS_PER_SEC; // Get the number of seconds
	t=t%CLOCKS_PER_SEC;
	t=t*100;
	t=t/CLOCKS_PER_SEC;
	//cout << "time: "<<s<<".";
	if (t<10) cout<<0;
	//cout<<t<<endl;
}


void PIController::plotSetup() const 	// GENERAL
// For export and plotting in matlab.
{
	// id number of experiment
	ofstream fout(outfile.c_str(),ios::trunc);
	fout << "%%matlab" << endl;
	fout << "%%This file is generated by pi_qrsim.cpp" << endl << endl;
	fout << "%%Moving Horizon stochastic PI control." << endl << endl;
	fout << "%%PARAMETERS:" << endl << endl;
	fout << "%%Horizon steps,\n H=" << H << ";"<< endl;
	fout << "%%Plant Precision" << endl << " dtperstep=" << dtperstep << ";"<< endl;
	fout << "%%Infinitessimal time, " << endl << "dt=" << dt << ";"<< endl;
	fout << "%%Time of a step, " << endl << "dS=dtperstep*dt;" << endl;
	fout << "%%Samples, " << endl << "N=" << N << ";"<< endl;
	fout << "%%DATA:" << endl;
	fout << "X = [];\n U = [];" << endl;
	fout.close();
}

void PIController::plotCurrent(const vec& state, const vec& action) const
// For export and plotting in matlab
{
	// id number of experiment
	ofstream fout(outfile.c_str(),ios::app);
	fout << "X = [X; [";
	for(size_t i=0; i<state.size(); i++) //dimX*units
		fout << state[i] << " ";
	fout << "]];" << endl;
	fout << "U = [U; [" << endl;
	for(size_t i=0; i<action.size(); i++) //dimU*units
		fout << action[i] << " ";
	fout << "]];" << endl;
	fout.close();
}

void PIController::shiftControls()
{
	// Move to horizon with exploring controls
	for (int s=1; s<iH; s++) {
		u_exp[s-1] = u_exp[s];
	}
	u_exp[iH-1] = vec(dimU,0);
	//cout <<  "Shifted control " << u_exp << endl;
}

vvec PIController::selectControls(const int& u)
{
	vvec u_exp_active(iH,vec(dimUAVu,0));
	//u_exp_active.resize(H, vec(dimUAVu,0));
	// select the active unit controls dim H,dimUAVu
	for (int s=0; s<iH; s++) {
		u_exp_active[s] = {u_exp[s][2*u],u_exp[s][2*u+1]};
	}

	return u_exp_active;
}

void PIController::updateControls(const vvec& u_exp_active, const int& u){

	for (int s=0; s<iH; s++) {
		u_exp[s][2*u] = u_exp_active[s][0];
		u_exp[s][2*u+1] = u_exp_active[s][1];
	}
	//std::cout << "5. Update controls unit" <<  u << "is" << u_exp << endl;
}
