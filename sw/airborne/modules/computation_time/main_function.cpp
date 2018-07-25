
#include "main_function.h"
#include "pi/PIController.h"
#include "pi/global.h"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>
#include <chrono>


// TODO: a better solution than global var?
//vector<double> joint_state;

// Main entry point of application
int main_function(const char *file)
{
		PIController pi;
		int units;
		vector<double> action;
		vector<double> new_state;
		//ifstream is("pi_UAV_joint.xml");
		// read parameters from xml file
		using boost::property_tree::ptree;
		ptree pt;

		pt.put("T",6);
		pt.put("R",1);
		pt.put("dt",0.04);
		pt.put("dtperstep",4);
		pt.put("H",1);
		pt.put("dh",0.25);
		pt.put("nu",1);
		pt.put("seed",0);
		pt.put("N",100);
		pt.put("dimUAVx",4);
		pt.put("dimUAVu",2);
		pt.put("units",3);
		pt.put("holdingp.x",0.0);
		pt.put("holdingp.y",0.0);
		pt.put("holdingp.radius",4.0);
		pt.put("holdingp.minv",1.0);
		pt.put("holdingp.maxv",3.0);
		pt.put("holdingp.hit",100);


		// initialize Joint State variable
		units = pt.get<int>("units");
		//joint_state.resize( units*pt.get<int>("dimUAVx") );

		// Initialize PI controller
		pi.Init(pt);

 	// END XML
	// Define initial positions and velocities for the 3 units manually
	vector<double> joint_state = pi.InitState();
	//cout << "Initial Joint state " << joint_state << endl;
	//joint_state = {0, 0, 1, 1, 2, 2, 1, 1, 4, 4, 1, 1};
	//mat_state.push_back(joint_state);
	std::vector<double> state_active;
	std::vector<double> state_inactive;
	std::vector<vector<double>> u_exp_active;
	double elapsed_time_unit;
	double elapsed_time_init;
	double elapsed_time_predict;
	double elapsed_time_compute;
	auto start_units = std::chrono::high_resolution_clock::now();


	for (int i = 0; i < pi.iT; i++) {		// Loop over simulation time
		auto start_init = std::chrono::high_resolution_clock::now();
		vector<double> joint_action;

		// Print iteration number
		if (i % 10 == 0 ) {
			cout << i << "/" << pi.iT << endl;
		}

		//pi.printTime();
		// Move to horizon with exploring controls
		pi.shiftControls();
		auto end_init= std::chrono::high_resolution_clock::now();
		elapsed_time_init += double(std::chrono::duration_cast <std::chrono::nanoseconds> (end_init-start_init).count());

		// select only the controls of the active unit
		for (int u = 0; u < units; u++) {
				if (u == 0 ) {
						start_units = std::chrono::high_resolution_clock::now();
				}

				u_exp_active = pi.selectControls(u);
				state_active = {joint_state[4*u], joint_state[4*u+1],joint_state[4*u+2], joint_state[4*u+3]};
			if (u == 0) {
				 state_inactive = {joint_state[4], joint_state[5],joint_state[6], joint_state[7],joint_state[8],joint_state[9],joint_state[10],joint_state[11]};
			} else if (u == 1) {
				 state_inactive = {joint_state[0], joint_state[1],joint_state[2], joint_state[3],joint_state[8],joint_state[9],joint_state[10],joint_state[11]};
			} else {
			 state_inactive = {joint_state[0],joint_state[1],joint_state[2],joint_state[3],joint_state[4], joint_state[5],joint_state[6], joint_state[7]};
			}



			//std::cout << "1.Active controls unit" <<  u << "is" << u_exp_active <<endl;
			//std::cout << "2.State active unit" <<  u << "is" << state_active << endl;
			auto start_compute = std::chrono::high_resolution_clock::now();
			action = pi.computeControl(state_active,state_inactive,u_exp_active);
			//std::cout << "4. computed controls unit" <<  u << "is" << action << endl;
			auto end_compute = std::chrono::high_resolution_clock::now();
			elapsed_time_compute += double(std::chrono::duration_cast <std::chrono::nanoseconds> (end_compute-start_compute).count());

			//save new action for each unit
			joint_action.push_back(action[0]);
			joint_action.push_back(action[1]);

			//update controls
			pi.updateControls(u_exp_active,u);

			if (u == 0 ) {
					auto end_units = std::chrono::high_resolution_clock::now();
					elapsed_time_unit += double(std::chrono::duration_cast <std::chrono::nanoseconds> (end_units-start_units).count());
			}

		}
		//cout <<  " combined control" << joint_action << endl;
		auto start_predict= std::chrono::high_resolution_clock::now();
		new_state = pi.predictState(joint_state, joint_action);
		auto end_predict= std::chrono::high_resolution_clock::now();
		elapsed_time_predict += double(std::chrono::duration_cast <std::chrono::nanoseconds> (end_predict-start_predict).count());

		//cout <<  " new_state" << new_state[0] <<" "<< new_state[1] <<" "<< new_state[4] <<" "<< new_state[5] <<" "<< new_state[8] <<" "<< new_state[9] << endl;
		joint_state = new_state;
		pi.plotCurrent(joint_state, joint_action);
		// Save the states of each simulation step
		// mat_state.push_back(new_state);
	}

	std::cout << "FNISHED!" << endl;
	std::cout << "elapsed_time init (s) :" << (elapsed_time_init/pi.iT)/1e9 << endl;
	std::cout << "elapsed_time unit (s) :" << (elapsed_time_unit/pi.iT)/1e9 << endl;
	std::cout << "elapsed_time compute (s) :" << (elapsed_time_compute/(pi.iT*3))/1e9 << endl;
	std::cout << "elapsed_time predict (s) :" << (elapsed_time_predict/pi.iT)/1e9 << endl;
	std::cout << "elapsed_time total (s) :" << (elapsed_time_init/pi.iT)/1e9+(elapsed_time_unit/pi.iT)/1e9+(elapsed_time_predict/pi.iT)/1e9 << endl;
	double elapsed_time_compute_s = (elapsed_time_compute/(pi.iT*3))/1e9;

	printf("elapsed_time compute (s): %f\n",elapsed_time_compute_s);

	// handling of environment variable
	// const char* bus = 0;
	// if (!bus) bus = getenv ("IVYBUS");
	// IvyStart (bus);

	// 	MESSAGES
	// IvyBindMsg (Callback, NULL, "^ground FLIGHT_PARAM(.*)$");

	// IVY LOOP RUNS IN THE MAIN THREAD
	// IvyMainLoop();
	return 0;
}

