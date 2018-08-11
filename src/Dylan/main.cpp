#include <MEL/Communications/MelShare.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Utility/System.hpp>
#include "OpenWrist.hpp"

using namespace mel;

// ctrl-c signal handler
ctrl_bool ctrlc(false);
bool handler(CtrlEvent event) {
    print("Ctrl+C Pressed");
    ctrlc = true;
    return true;
}

int main(int argc, char* argv[]) {



    // make options
    Options options("openwrist_q8usb.exe", "OpenWrist Q8 USB Demo");
    options.add_options()
        ("c,calibrate", "Calibrates the OpenWrist")
        ("t,transparency", "Puts the OpenWrist in active transparency mode")
        ("d,dylan", "Runs Dylan's code")
        ("d,debug", "Debug Mode - No power,, position and velocity written to MelShare \"ow_state\"")
        ("h,help", "Prints this help message");

    auto result = options.parse(argc, argv);
    if (result.count("help") > 0) {
        print(options.help());
        return 0;
    }

    // register ctrl-c handler
    register_ctrl_handler(handler);

    // make Q8 USB that's configured for current control with VoltPAQ-X4
    QuanserOptions qoptions;
    qoptions.set_update_rate(QuanserOptions::UpdateRate::Fast);
    qoptions.set_analog_output_mode(0, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    qoptions.set_analog_output_mode(1, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    qoptions.set_analog_output_mode(2, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    Q8Usb q8(qoptions);

    VoltPaqX4 vpx4(q8.DO[{ 0, 1, 2 }], q8.AO[{ 0, 1, 2 }], q8.DI[{0, 1, 2}], q8.AI[{ 0, 1, 2 }]);

    // create OpenWrist and bind Q8 channels to it
    OwConfiguration config(
        q8,
        q8.watchdog,
        q8.encoder[{ 0, 1, 2 }],
        vpx4.amplifiers
    );
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

    // enter Dylan's code
    if (result.count("dylan") > 0) {
        // create critically damped PD controller for each exis
        PdController pd0(25, 1.15); // joint 0 ( Nm/rad , Nm-s/rad )
        PdController pd1(20, 1.00); // joint 1 ( Nm/rad , Nm-s/rad )
        PdController pd2(20, 0.25); // joint 2 ( Nm/rad , Nm-s/rad )
        // enable hardware
        q8.enable();
        ow.enable();
        // start watchdog and timer
        q8.watchdog.start();
        Timer timer(milliseconds(1));
        // enter control loop
        while (!ctrlc) {
            // kick watchdog
            q8.watchdog.kick();
            // update inputs
            q8.update_input();

            // BEGIN DYLAN'S CODE
            double torque0 = pd0.calculate(0.0, ow[0].get_position(), 0.0, ow[0].get_velocity());
            double torque1 = pd1.calculate(0.0, ow[1].get_position(), 0.0, ow[1].get_velocity());
            double torque2 = pd2.calculate(0.0, ow[2].get_position(), 0.0, ow[2].get_velocity());
            ow[0].set_torque(torque0);
            ow[1].set_torque(torque1);
            ow[2].set_torque(torque2);

            // END DYLAN'S CODE

            // check limits
            if (ow.any_limit_exceeded())
                ctrlc = true;
            // update outputs
            q8.update_output();
            // wait timer
            timer.wait();
        }
        return 0;
    }

    // disable Windows realtime
    disable_realtime();
    return 0;

}
