#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include "OpenWrist.hpp"
#include <MEL/Core/Console.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Logging/Log.hpp>
#include <Games/XWing/XWing.hpp>

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
    Options options("xwing.exe", "OpenWrist XWing Demo");
    options.add_options()
        ("c,calibrate", "Calibrates the OpenWrist")
        ("x,xwing", "Runs A Jedi's Last Stand demo")
        ("h,help", "Prints this help message");

    auto result = options.parse(argc, argv);
    if (result.count("help") > 0) {
        print(options.help());
        return 0;
    }

    // enable Windows realtime
    enable_realtime();

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

    // enter Jedi demo
    if (result.count("xwing") > 0) {
        XWing game(Timer(hertz(1000), Timer::Hybrid), q8, ow, ctrlc);
        game.execute();
        disable_realtime();
        return 0;
    }

    // disable Windows realtime
    disable_realtime();
    return 0;

}
