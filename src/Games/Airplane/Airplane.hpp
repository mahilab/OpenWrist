#pragma once
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Core/Console.hpp>
#include "OpenWrist.hpp"
#include "Cuff/Cuff.hpp"

using namespace mel;

class Airplane{
public:
	Airplane(Q8Usb& ow_daq,
		OpenWrist& ow,
		Cuff& cuff,
		ctrl_bool& stop_flag,
		bool cuff_active );

	void play();
	void update_ow_torques();
	void update_cuff_torques();
	void initialize();
	void cinch_cuff();
	void release_cuff();

private:
	//-------------------------------------------------------------------------
	// PRIVATE VARIABLES
	//-------------------------------------------------------------------------
	MelShare msstates;
	MelShare msunity;


	std::vector<double> states;
	std::vector<double> torque;
	std::vector<double> gravitycomp;
	std::vector<double> unitydata;

	// HARDWARE TIMER
	Timer timer_;

	// HARDWARE
	Q8Usb& ow_daq_;
	OpenWrist& ow_;
	Cuff& cuff_;

	// STOP FLAG
	ctrl_bool& stop_flag_;

	 // CUFF
	bool cuff_active_;
    const short int cuff_normal_force_ = 4;
	const short int cuff_ff_gain_ = 250;
    const short int cuff_fb_gain_ = 350;
    short int offset[2];
    short int scaling_factor[2];

	short int cuff_ref_pos_1_;
    short int cuff_ref_pos_2_;


	///underdamped pd controllers for air flying
	std::array<PdController, 3> underdamp = std::array<PdController, 3>{
			PdController(5, 1.15), // joint 0 ( Nm/rad , Nm-s/rad )
			PdController(2, 1.5), // joint 1 ( Nm/rad , Nm-s/rad )
			PdController(3, 0.25)  // joint 2 ( Nm/rad , Nm-s/rad )
	};

	//double px_per_rad_x_ = 1920.0 / (100 * DEG2RAD);
	//double px_per_rad_y_ = 1080.0 / (50 * DEG2RAD);


};
