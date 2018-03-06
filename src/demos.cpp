#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Exoskeletons/OpenWrist/OpenWrist.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Utility/Timer.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include <MEL/Math/Functions.hpp>
#include "Jedi.hpp"
#include <atomic>

static bool stop = false;
static void handler(int var) {
    stop = true;
}

using namespace mel;

int main(int argc, char* argv[]) {

    // register ctrl-c handler
    register_ctrl_c_handler(handler);

    // make options
    Options options("openwrist_q8usb.exe", "OpenWrist Q8 USB Demo");
    options.add_options()
        ("c,calibrate", "Calibrates the OpenWrist")
        ("t,transparency", "Puts the OpenWrist in transparency mode")
        ("s,setpoint", "Runs OpenWrist MelScope set-point demo")
        ("j,jedi", "Runs A Jedi's Last Stand demo")
        ("h,help", "Prints this help message")
        ("x","x");

    auto result = options.parse(argc, argv);
    if (result.count("help") > 0) {
        print(options.help());
        return 0;
    }

    // enable Windows realtime
    enable_realtime();

    // make Q8 USB that's configured for current control with VoltPAQ-X4
    QOptions qoptions;
    qoptions.set_update_rate(QOptions::UpdateRate::Fast);
    qoptions.set_analog_output_mode(0, QOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    qoptions.set_analog_output_mode(1, QOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    qoptions.set_analog_output_mode(2, QOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    Q8Usb q8(qoptions);

    VoltPaqX4 vpx4(q8.digital_output[{ 0, 1, 2 }], q8.analog_output[{ 0, 1, 2 }], q8.digital_input[{0, 1, 2}], q8.analog_input[{ 0, 1, 2 }]);

    // create OpenWrist and bind Q8 channels to it
    OwConfiguration config(
        q8,
        q8.watchdog,
        q8.encoder[{ 0, 1, 2 }],
        q8.velocity[{ 0, 1, 2 }],
        vpx4.amplifiers
    );
    OpenWrist ow(config);

    // run calibration script
    if (result.count("calibrate") > 0) {
        ow.calibrate(stop);
        disable_realtime();
        return 0;
    }

    // enter transparency mode
    if (result.count("transparency") > 0) {
        ow.transparency_mode(stop);
        disable_realtime();
        return 0;
    }

    // enter Jedi demo
    if (result.count("jedi") > 0) {
        Jedi game(Timer(milliseconds(1), Timer::Hybrid), q8, ow, stop);
        game.execute();
        disable_realtime();
        return 0;
    }

    if (result.count("x") > 0) {
        MelNet x(55001, 55002, "10.66.64.67");
        std::string message = x.receive_message();
        print(message);
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
        while (!stop) {
            q8.update_input();
            positions = ms.read_data();
            positions[0] = saturate(positions[0], 80);
            positions[1] = saturate(positions[1], 60);
            positions[2] = saturate(positions[2], 30);
            torques[0] = ow.pd_controllers_[0].move_to_hold(positions[0] * DEG2RAD, ow[0].get_position(), 30 * DEG2RAD, ow[0].get_velocity(), 0.001, DEG2RAD, 10 * DEG2RAD);
            torques[1] = ow.pd_controllers_[1].move_to_hold(positions[1] * DEG2RAD, ow[1].get_position(), 30 * DEG2RAD, ow[1].get_velocity(), 0.001, DEG2RAD, 10 * DEG2RAD);
            torques[2] = ow.pd_controllers_[2].move_to_hold(positions[2] * DEG2RAD, ow[2].get_position(), 30 * DEG2RAD, ow[2].get_velocity(), 0.001, DEG2RAD, 10 * DEG2RAD);

            state_data[0] = ow[0].get_position();
            state_data[1] = ow[1].get_position();
            state_data[2] = ow[2].get_position();
            state_data[3] = positions[0];
            state_data[4] = positions[1];
            state_data[5] = positions[2];
            state_data[6] = ow.motors_[0].get_torque_sense();
            state_data[7] = ow.motors_[1].get_torque_sense();
            state_data[8] = ow.motors_[2].get_torque_sense();
            state_data[9] = torques[0];
            state_data[10] = torques[1];
            state_data[11] = torques[2];

            state.write_data(state_data);

            ow.set_joint_torques(torques);
            if (!q8.watchdog.kick() || ow.check_all_joint_limits())
                stop = true;
            q8.update_output();
            timer.wait();
        }
    }

    // disable Windows realtime
    disable_realtime();
    return 0;

}
