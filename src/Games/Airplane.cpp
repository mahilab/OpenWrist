#include  "Airplane.hpp"
#include <MEL/Devices/Windows/Keyboard.hpp>
#include <MEL/Core.hpp>


Airplane::Airplane(Q8Usb& ow_daq, OpenWrist& ow, ctrl_bool& stop_flag) :
	timer_(hertz(1000)),
	ow_daq_(ow_daq),
	ow_(ow),
	stop_flag_(stop_flag),
	msstates("states"),
	msunity("unity"),
	states(6),
	gravitycomp(3),
	torque(3)
{

}


void Airplane::play() {
	
	//startup
	ow_daq_.enable();
	
	// enable OpenWrist
	timer_.restart();
	ow_.enable();

	//enable cuff
	//cuff.enable();

	ow_daq_.watchdog.set_timeout(milliseconds(100));
	ow_daq_.watchdog.start();
	
	// loop
	while (true) {
		// read OpenWrist DAQ
		ow_daq_.update_input();
		ow_daq_.watchdog.kick();

		//get position and velocity state data
		states[0] = ow_[0].get_position();
		states[1] = ow_[1].get_position();
		states[2] = ow_[2].get_position();
		states[3] = ow_[0].get_velocity();
		states[4] = ow_[1].get_velocity();
		states[5] = ow_[2].get_velocity();
		msstates.write_data(states);

		update_ow_torques();

		// check for stop conditions
		if (stop_flag_ ||
			ow_.any_limit_exceeded() ||
			Keyboard::is_key_pressed(Key::Escape)) {
			break;
		}
		// write OpenWrist DAQ
		ow_daq_.update_output();
		
		timer_.wait();
	}//end loop

	//Turn off everything

	// disable OpenWrist
	ow_.disable();
	// disable OpenWrist DAQ
	ow_daq_.watchdog.stop();
	ow_daq_.disable();
}

void Airplane::initialize(){

}

void Airplane::update_ow_torques(){
		gravitycomp[0] = ow_.compute_gravity_compensation(0);
		gravitycomp[1] = ow_.compute_gravity_compensation(1);
		gravitycomp[2] = ow_.compute_gravity_compensation(2);

		torque[0] = gravitycomp[0] + underdamp[0].calculate(0, states[0], 0, 0);
		torque[1] = gravitycomp[1] + underdamp[1].calculate(0, states[1], 0, 0);
		torque[2] = gravitycomp[2] + underdamp[2].calculate(0, states[2], 0, 0);

		ow_[0].set_torque(torque[0]);
		ow_[1].set_torque(torque[1]);
		ow_[2].set_torque(torque[2]);
}
void Airplane::update_cuff_torques(){

}