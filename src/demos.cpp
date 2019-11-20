#include <MEL/Communications/MelShare.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Math/Functions.hpp>
#include "Simulations/Pendulum.hpp"
#include "Games/Jedi/Jedi.hpp"
#include <atomic>
#include "Games/Jedi/Jedi.hpp"
#include "Games/OctagonSqueeze/OctagonSqueeze.hpp"
#include "Games/Airplane/Airplane.hpp"
#include "Simulations/Pendulum.hpp"
#include "Simulations/BallAndBeam.hpp"
#include "OpenWrist.hpp"
#include "Cuff/Cuff.hpp"

ctrl_bool ctrlc(false);
bool handler(CtrlEvent event) {
    print("Ctrl+C Pressed");
    ctrlc = true;
    return true;
}

using namespace mel;

int main(int argc, char* argv[]) {

    // register ctrl-c handler
    register_ctrl_handler(handler);

    // make options
    Options options("openwrist_q8usb.exe", "OpenWrist Q8 USB Demo");
    options.add_options()
        ("c,calibrate", "Calibrates the OpenWrist")
        ("t,transparency", "Puts the OpenWrist in transparency mode")
        ("s,setpoint", "Runs OpenWrist MelScope set-point demo")
        ("j,jedi", "Runs A Jedi's Last Stand demo")
        ("o,octagon", "Runs OctagonSqueeze")
        ("p,pendulum", "Runs OpenWrist Pendulum demo")
        ("b,ballbeam", "Runs OpenWrist Ball and Beam demo")
		("a,airplane", "Runs the airplane target game demo")
        ("d,debug", "Debug Mode (No Power)")
        ("k,cuff","Tests CUFF Power and Output")
        ("h,help","Prints this help message");

    auto result = options.parse(argc, argv);
    if (result.count("help") > 0) {
        print(options.help());
        return 0;
    }

    // enable Windows realtime
    enable_realtime();

    // make Q8 USB that's configured for current control with VoltPAQ-X4
    // Hardware
    QuanserOptions qoptions;
    //qoptions.set_update_rate(QuanserOptions::UpdateRate::Fast);
    qoptions.set_analog_output_mode(0, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    qoptions.set_analog_output_mode(1, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    qoptions.set_analog_output_mode(2, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    qoptions.set_analog_output_mode(3, QuanserOptions::AoMode::VoltageMode,0,0,0,0,0,0,0);
    Q8Usb q8(qoptions);

    if (!q8.open()) {
        LOG(Fatal) << "Unable to open Q8-USB. Aborting OpenWrist demo application.";
    }

    VoltPaqX4 vpx4(q8.DO[{ 0, 1, 2 }], q8.AO[{ 0, 1, 2 }], q8.DI[{0, 1, 2}], q8.AI[{ 0, 1, 2 }]);

    // create OpenWrist and bind Q8 channels to it
    OwConfiguration config(q8, q8.watchdog, q8.encoder[{0, 1, 2}], vpx4.amplifiers);
    OpenWrist ow(config);

    
     // run cuff script
    if (result.count("k") > 0) {
        CuffVoltPaq cuff("cuff",3,q8.DO[3],q8.AO[3]);
        q8.enable();
        cuff.enable();
        sleep(seconds(10));
        cuff.disable();

        return 0;
    }

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

    // enter Jedi demo
    if (result.count("jedi") > 0) {
        Jedi game(Timer(hertz(1000), Timer::Hybrid), q8, ow, ctrlc);
        game.execute();
        disable_realtime();
        return 0;
    }

    // enter OctagonSqueeze demo
    if (result.count("octagon")) {
        OctagonSqueeze game(q8, ow, ctrlc);
        game.execute();
        disable_realtime();
        return 0;
    }

	// enter Airplane Demo
	if (result.count("airplane")) {
        Cuff cuff("cuff",3);
		Airplane game(q8, ow, cuff, ctrlc,false);
		game.play();
		disable_realtime();
		return 0;
	}

    // enter Pendulum demo
    if (result.count("pendulum") > 0) {
        Pendulum pendulum;
        mel::PdController pd1(60, 1);    // OpenWrist Joint 1 (FE)
        mel::PdController pd2(40, 0.5);  // OpenWrist Joint 2 (RU)
        q8.enable();
        ow.enable();
        q8.watchdog.start();
        Timer timer(milliseconds(1));
        while (!ctrlc) {
            q8.watchdog.kick();
            q8.update_input();

            pendulum.step_simulation(timer.get_elapsed_time(),
                                     ow[0].get_position(),
                                     ow[0].get_velocity());

            double ps_comp_torque_ = ow.compute_gravity_compensation(0) +
                                     0.75 * ow.compute_friction_compensation(0);
            double ps_total_torque_ = ps_comp_torque_ - pendulum.Tau[0];
            ow[0].set_torque(ps_total_torque_);

            ow[1].set_torque(pd1.move_to_hold(
                0, ow[1].get_position(), 60 * mel::DEG2RAD,
                ow[1].get_velocity(), 0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));

            ow[2].set_torque(pd2.move_to_hold(
                mel::DEG2RAD * 0, ow[2].get_position(), 60 * mel::DEG2RAD,
                ow[2].get_velocity(), 0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));

            if (ow.any_limit_exceeded())
                ctrlc = true;

            q8.update_output();
            timer.wait();
        }
        return 0;
    }

    // enter Ball and Beam demo
    if (result.count("ballbeam") > 0) {
        MelShare ms("ballbeam");
        BallAndBeam bb;
        mel::PdController pd1(60, 1);    // OpenWrist Joint 1 (FE)
        mel::PdController pd2(40, 0.5);  // OpenWrist Joint 2 (RU)
        std::vector<double> state_data(3);
        q8.enable();
        ow.enable();
        q8.watchdog.start();
        Timer timer(hertz(1000), Timer::Hybrid);
        while (!ctrlc) {
            q8.watchdog.kick();
            q8.update_input();

            bb.step_simulation(timer.get_elapsed_time(), ow[0].get_position(),
                               ow[0].get_velocity());

            state_data[0] = bb.r;
            state_data[1] = bb.q;
            state_data[2] = bb.tau;
            ms.write_data(state_data);

            double ps_comp_torque_ = ow.compute_gravity_compensation(0) +
                                     0.75 * ow.compute_friction_compensation(0);
            double ps_total_torque_ = ps_comp_torque_ - bb.tau / 2.0;
            ow[0].set_torque(ps_total_torque_);

            ow[1].set_torque(pd1.move_to_hold(
                0, ow[1].get_position(), 60 * mel::DEG2RAD,
                ow[1].get_velocity(), 0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));

            ow[2].set_torque(pd2.move_to_hold(
                mel::DEG2RAD * 0, ow[2].get_position(), 60 * mel::DEG2RAD,
                ow[2].get_velocity(), 0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));

            if (ow.any_limit_exceeded())
                ctrlc = true;

            q8.update_output();
            timer.wait();
        }
        return 0;
    }

    // setpoint control with MelScope
    if (result.count("setpoint") > 0) {
        MelShare ms("ow_setpoint");
        MelShare state("ow_state");
        std::vector<double> state_data(12, 0.0);
        std::vector<double> positions(3, 0.0);
        std::vector<double> torques(3, 0.0);
        ms.write_data(positions);
        q8.enable();
        ow.enable();
        q8.watchdog.start();
        Timer timer(milliseconds(1), Timer::Hybrid);
        while (!ctrlc) {
            q8.update_input();
            positions    = ms.read_data();
            positions[0] = saturate(positions[0], 80);
            positions[1] = saturate(positions[1], 60);
            positions[2] = saturate(positions[2], 30);
            torques[0]   = ow.pd_controllers_[0].move_to_hold(
                positions[0] * DEG2RAD, ow[0].get_position(), 30 * DEG2RAD,
                ow[0].get_velocity(), 0.001, DEG2RAD, 10 * DEG2RAD);
            torques[1] = ow.pd_controllers_[1].move_to_hold(
                positions[1] * DEG2RAD, ow[1].get_position(), 30 * DEG2RAD,
                ow[1].get_velocity(), 0.001, DEG2RAD, 10 * DEG2RAD);
            torques[2] = ow.pd_controllers_[2].move_to_hold(
                positions[2] * DEG2RAD, ow[2].get_position(), 30 * DEG2RAD,
                ow[2].get_velocity(), 0.001, DEG2RAD, 10 * DEG2RAD);

            state_data[0]  = ow[0].get_position();
            state_data[1]  = ow[1].get_position();
            state_data[2]  = ow[2].get_position();
            state_data[3]  = positions[0];
            state_data[4]  = positions[1];
            state_data[5]  = positions[2];
            state_data[6]  = ow.motors_[0].get_torque_sense();
            state_data[7]  = ow.motors_[1].get_torque_sense();
            state_data[8]  = ow.motors_[2].get_torque_sense();
            state_data[9]  = torques[0];
            state_data[10] = torques[1];
            state_data[11] = torques[2];

            state.write_data(state_data);

            ow.set_joint_torques(torques);
            if (!q8.watchdog.kick() || ow.any_limit_exceeded())
                ctrlc = true;
            q8.update_output();
            timer.wait();
        }
    }

    // disable Windows realtime
    disable_realtime();
    return 0;
}
