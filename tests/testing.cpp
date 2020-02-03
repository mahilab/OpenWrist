#include <MEL/Core/Console.hpp>
#include "OpenWrist.hpp"
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Core.hpp>
#include <MEL/Communications.hpp>
#include <MEL/Math.hpp>

using namespace mel;

ctrl_bool ctrlc(false);
bool handler(CtrlEvent event) {
	print("Ctrl+C Pressed");
	ctrlc = true;
	return true;
}

int main(int argc, char* argv[]) {

	// register ctrl-c handler
	register_ctrl_handler(handler);

	// make options
	Options options("openwrist_q8usb.exe", "OpenWrist Q8 USB Demo");
	options.add_options()
		("c,calibrate", "Calibrates the OpenWrist")
		("t,transparency", "Puts the OpenWrist in transparency mode")
		("d,debug", "Debug Mode (No Power)")
		("s,sine", "Hold one joint to a sine wave and lock the others using standard PD controller")
		("h,help", "Prints this help message");

	auto result = options.parse(argc, argv);
	if (result.count("help") > 0) {
		print(options.help());
		return 0;
	}


	// enable Windows realtime
	//enable_realtime();


	// make Q8 USB that's configured for current control with VoltPAQ-X4
// Hardware
	QuanserOptions qoptions;
	//qoptions.set_update_rate(QuanserOptions::UpdateRate::Fast);
	qoptions.set_analog_output_mode(0, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
	qoptions.set_analog_output_mode(1, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
	qoptions.set_analog_output_mode(2, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
	Q8Usb q8(qoptions);

	if (!q8.open()) {
		LOG(Fatal) << "Unable to open Q8-USB. Aborting OpenWrist demo application.";
	}

	VoltPaqX4 vpx4(q8.DO[{ 0, 1, 2 }], q8.AO[{ 0, 1, 2 }], q8.DI[{0, 1, 2}], q8.AI[{ 0, 1, 2 }]);

	// create OpenWrist and bind Q8 channels to it
	OwConfiguration config(q8, q8.watchdog, q8.encoder[{0, 1, 2}], vpx4.amplifiers);
	OpenWrist ow(config);

	// run calibration script
	if (result.count("calibrate") > 0) {
		ow.calibrate(ctrlc);
		disable_realtime();
		return 0;
	}

	// enter transparency mode
	if (result.count("transparency") > 0) {
		ow.transparency_mode(ctrlc);
		disable_realtime();
		return 0;
	}

	// enter debug mode
	if (result.count("debug") > 0) {
		q8.enable();
		MelShare ms_ow_state("ow_state");
		std::vector<double> ow_state_data(6);
		Timer timer(milliseconds(1));
		while (!ctrlc) {
			q8.update_input();
			ow_state_data[0] = ow[0].get_position();
			ow_state_data[1] = ow[1].get_position();
			ow_state_data[2] = ow[2].get_position();
			ow_state_data[3] = ow[0].get_velocity();
			ow_state_data[4] = ow[1].get_velocity();
			ow_state_data[5] = ow[2].get_velocity();
			ms_ow_state.write_data(ow_state_data);
			timer.wait();
		}
		return 0;
	}

	if (result.count("sine") > 0) {
		q8.enable();
		q8.watchdog.start();
		ow.enable();

		std::vector<double> jointref(3);//position reference values
		std::vector<double> ow_state_data(7);//0pos 1pos 2pos 0vel 1vel 2vel sinereference
		std::vector<double> ow_torques(3);//0torq 1torq 2torq
		std::array<double, 3> sat_torques = { 5.0, 5.0, 5.0 }; // temporary saturation torques
		double t = 0;
		
		MelShare ms_ow_state("ow_state");
		Timer timer(milliseconds(1));
		
		while (!ctrlc) {

			t = timer.get_elapsed_time().as_seconds();
			jointref[0] = 0.5*mel::sin(t);
			jointref[1] = 0.5*mel::sin(t*1.25);
			jointref[2] = 0.5*mel::sin(t*0.75);


			q8.update_input();
			q8.watchdog.kick();

			ow_state_data[0] = ow[0].get_position();
			ow_state_data[1] = ow[1].get_position();
			ow_state_data[2] = ow[2].get_position();
			ow_state_data[3] = ow[0].get_velocity();
			ow_state_data[4] = ow[1].get_velocity();
			ow_state_data[5] = ow[2].get_velocity();
			ow_state_data[6] = jointref[1];
			ms_ow_state.write_data(ow_state_data);

			//calculate necessary torques
			ow_torques[0] = ow.pd_controllers_[0].calculate(jointref[0], ow_state_data[0], 0, ow_state_data[3]);
			ow_torques[1] = ow.pd_controllers_[1].calculate(jointref[1], ow_state_data[1], 0, ow_state_data[4]);
			ow_torques[2] = ow.pd_controllers_[2].calculate(jointref[2], ow_state_data[2], 0, ow_state_data[5]);

			// saturate torques by maximum limits
			ow_torques[0] = saturate(ow_torques[0], sat_torques[0]);
			ow_torques[1] = saturate(ow_torques[1], sat_torques[1]);
			ow_torques[2] = saturate(ow_torques[2], sat_torques[2]);

			// check joint limits
			if (ow.any_torque_limit_exceeded() || ow.any_velocity_limit_exceeded()) {
				ctrlc = true;
				break;
			}
			//set torques for motors
			ow.set_joint_torques(ow_torques);
			q8.update_output();

			timer.wait();
		}
		ow.disable();
		q8.disable();
		return 0;
	}
}

