#pragma once
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Core/Console.hpp>
#include "OpenWrist.hpp"

using namespace mel;

class Airplane{
public:
	Airplane(Q8Usb& ow_daq,
		OpenWrist& ow,
		ctrl_bool& stop_flag);

	void play();

private:
	//-------------------------------------------------------------------------
	// PRIVATE VARIABLES
	//-------------------------------------------------------------------------
	MelShare msstates;
	MelShare msunity;

	std::vector<double> states;
	std::vector<double> torque;
	std::vector<double> gravitycomp;

	// HARDWARE TIMER
	Timer timer_;

	// HARDWARE
	Q8Usb& ow_daq_;
	OpenWrist& ow_;

	// STOP FLAG
	ctrl_bool& stop_flag_;


	///underdamped pd controllers for air flying
	std::array<PdController, 3> underdamp = std::array<PdController, 3>{
			PdController(5, 1.15), // joint 0 ( Nm/rad , Nm-s/rad )
			PdController(2, 1.5), // joint 1 ( Nm/rad , Nm-s/rad )
			PdController(3, 0.25)  // joint 2 ( Nm/rad , Nm-s/rad )
	};

	//double px_per_rad_x_ = 1920.0 / (100 * DEG2RAD);
	//double px_per_rad_y_ = 1080.0 / (50 * DEG2RAD);


};
