#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Utility/Options.hpp>
#include "Cuff/Cuff.hpp"
#include "HapticGuidance.hpp"
#include "OpenWrist.hpp"
#include <MEL/Utility/Windows/Keyboard.hpp> 

int main(int argc, char* argv[]) {
    // init logger
    init_logger();

    // set up options
    mel::Options options("haptic_guidance.exe", "Haptic Guidance Experiment");
    options.add_options()("s,subj", "Subject Number", value<int>())(
        "c,cond", "Condition Number", value<int>())(
        "t,trial", "Start Trial Tag Name", value<std::string>())
            ("d,debug", "Debug")

            (
        "h,help", "Print Help Message");

    auto input = options.parse(argc, argv);
    if (input.count("help") > 0) {
        print(options.help());
        return 0;
    }

    int subject_number      = 1;
    int condition           = 1;
    std::string start_trial = "F1-1";

    if (input.count("subj"))
        subject_number = input["subj"].as<int>();
    if (input.count("cond"))
        condition = input["cond"].as<int>();
    if (input.count("trial"))
        start_trial = input["trial"].as<std::string>();

    // Hardware
    QOptions qoptions;
    qoptions.set_update_rate(QOptions::UpdateRate::Fast);
    qoptions.set_analog_output_mode(0, QOptions::AoMode::CurrentMode1, 0, 2.0,
                                    20.0, 0, -1, 0, 1000);
    qoptions.set_analog_output_mode(1, QOptions::AoMode::CurrentMode1, 0, 2.0,
                                    20.0, 0, -1, 0, 1000);
    qoptions.set_analog_output_mode(2, QOptions::AoMode::CurrentMode1, 0, 2.0,
                                    20.0, 0, -1, 0, 1000);
    Q8Usb q8(qoptions);

    VoltPaqX4 vpx4(q8.digital_output[{0, 1, 2}], q8.analog_output[{0, 1, 2}],
                   q8.digital_input[{0, 1, 2}], q8.analog_input[{0, 1, 2}]);

    // create OpenWrist and bind Q8 channels to it
    OwConfiguration config(q8, q8.watchdog, q8.encoder[{0, 1, 2}],
                           q8.velocity[{0, 1, 2}], vpx4.amplifiers);
    OpenWrist ow(config);
    Cuff cuff("cuff", 4);

    if (input.count("debug")) {
        Timer timer(hertz(1000));
        int pos = 0;
        cuff.enable();
        while (!Keyboard::is_key_pressed(Key::Escape)) {

            if (Keyboard::is_key_pressed(Key::Up))
                pos += 1;
            else if (Keyboard::is_key_pressed(Key::Down))
                pos -= 1;
            print(pos);
            cuff.set_motor_positions(-pos, pos, true);
            timer.wait();
        }
        cuff.disable();
        return 0;
    }

    HapticGuidance experiment(q8, ow, cuff, subject_number, condition,
                              start_trial);
    experiment.execute();
}
