#include "Cuff/Cuff.hpp"
#include <MEL/Utility/Console.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include "FurutaPendulum.hpp"
#include <MEL/Core/PdController.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include "OpenWrist.hpp"
#include <MEL/Utility/Windows/Keyboard.hpp>
#include <MEL/Utility/RingBuffer.hpp>
#include <MEL/Math/Waveform.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Utility/Windows/XboxController.hpp>
#include <MEL/Math/Differentiator.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <fstream>
#include "HapticTraining.hpp"

using namespace mel;

int main(int argc, char* argv[]) {

    // init logger
    init_logger(Verbose,Verbose);

    // set up options
    mel::Options options("haptic_guidance.exe", "Haptic Guidance Experiment");
    options.add_options()
        ("s,subj","Subject Number", value<int>())
        ("c,cond","Condition Number", value<int>())
        ("t,trial","Start Trial Tag Name", value<std::string>())
        ("h,help","Print Help Message");

    auto input = options.parse(argc, argv);
    if (input.count("help") > 0) {
        print(options.help());
        return 0;
    }

    int subject_number = 1;
    int condition = 1;
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
    Cuff cuff("cuff", 4);

    HapticTraining experiment(q8, ow, cuff, subject_number, condition, start_trial);
    experiment.execute();

}



int main2(int argc, char* argv[]) {

    // set up options
    mel::Options options("haptic_guidance.exe", "Haptic Guidance Experiment");
    options.add_options()
        ("x,test", "Test Code")
        ("u,up", "Pendulum Starts Upright")
        ("d,down", "Pendulum Starts Downright")
        ("p,pos", "Use position input instead of torque")
        ("c,cuff", "Enables CUFF")
        ("h,help", "Prints Help Message");

    auto result = options.parse(argc, argv);
    if (result.count("help") > 0) {
        print(options.help());
        return 0;
    }

    // initialize MEL logger
    init_logger();

    // enable Windows realtime
    // enable_realtime();

    std::vector<std::vector<double>> recorded_data;
    //read_csv("best", "./recordings/", 1, recorded_data);
    std::vector<double> recorded_torque; //= get_column(recorded_data, 6);
    std::vector<double> recorded_position; //= get_column(recorded_data, 0);

    // test code
    if (result.count("test") > 0) {

        MelShare ms("p_tau");
        FurutaPendulum fp;

        if (result.count("up") > 0) {
            fp.reset(0, 0, 0, 0);
        }
        else if (result.count("down")) {
            fp.reset(0, mel::PI, 0, 0);
        }
        fp.update(Time::Zero, 0);

        double tau = 0.0;
        Timer timer(hertz(1000));

        double u_ref = fp.c2 * fp.g * fp.m2;
        bool lqr = false;
        int i = 0;
        while (true) {


            if (result.count("up") > 0) {
                tau = -(-1.0 * fp.q1 + 4.6172 * fp.q2 - 0.9072 * fp.q1d + 1.2218 * fp.q2d);
            }
            else if (result.count("down")) {
                if (i < recorded_torque.size())
                    tau = recorded_torque[i];
                else
                    tau = 0.0;
            }

            if (Keyboard::is_key_pressed(Key::Right))
                fp.tau2 = 0.25;
            else if (Keyboard::is_key_pressed(Key::Left))
                fp.tau2 = -0.25;
            else
                fp.tau2 = 0.0;

            tau = mel::saturate(tau, -3.0, 3.0);
            if (Keyboard::is_key_pressed(Key::Space))
                fp.update(timer.get_elapsed_time(), 0);
            else
                fp.update(timer.get_elapsed_time(), tau);
            ++i;
            timer.wait();
        }
        return 0;
    }


    // OpenWrist setup
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

    // init cuff
    short int cuff_ref_pos_1_;
    short int cuff_ref_pos_2_;
    const short int cuff_normal_force_ = 2.5;
    const short int cuff_ff_gain_ = 250;
    const short int cuff_fb_gain_ = 175;
    short int offset[2];
    short int scaling_factor[2];
    double cuff_angle = 0.0;
    Cuff cuff("cuff", 4);

    DataLogger log(WriterType::Buffered, false);
    log.set_header({ "q1", "q2","q1d", "q2d", "q1dd", "q2dd", "tau1", "tau2", "k1", "k2", "u1", "u2" });

    if (result.count("cuff")) {
        prompt("Press ENTER to tension CUFF");
        cuff.enable();
        cuff.pretension(cuff_normal_force_, offset, scaling_factor);
        cuff.set_motor_positions(offset[0], offset[1], true);
    }

    FurutaPendulum fp;

    if (result.count("up") > 0) {
        fp.reset(0, 0, 0, 0);
    }
    else if (result.count("down")) {
        fp.reset(0, mel::PI, 0, 0);
    }
    fp.update(Time::Zero, 0);

    mel::PdController pd1(60, 1);   // OpenWrist Joint 1 (FE)
    mel::PdController pd2(40, 0.5); // OpenWrist Joint 2 (RU)

    double K_player = 25;                      ///< [N/m]
    double B_player = 1;                       ///< [N-s/m]
    double tau;

    prompt("Press Enter to Start");

    if (result.count("up") > 0) {
        fp.reset(0, 0, 0, 0);
    }
    else if (result.count("down")) {
        fp.reset(0, mel::PI, 0, 0);
    }

    q8.enable();
    ow.enable();
    q8.watchdog.start();

    MelShare ms("haptics");
    std::vector<double> data(2);

    double wall = 50 * mel::DEG2RAD;
    double k_wall = 50;
    double b_wall = 1;
    bool recording = false;

    int r_index = 0;

    MelShare ms_up("uptime");
    Time curr_up_time = Time::Zero;
    Time best_up_time = Time::Zero;
    Clock up_time_clock;

    Timer timer(milliseconds(1));
    while (true) {
        q8.watchdog.kick();
        q8.update_input();

        // reset on R
        if (Keyboard::is_key_pressed(Key::R)) {
            if (result.count("up")) {
                fp.reset(ow[1].get_position(), 0, 0.0, 0.0);
                up_time_clock.restart();
            }
            else if (result.count("down")) {
                r_index = 0;
                fp.reset(ow[1].get_position(), mel::PI ,0.0, 0.0);
            }
        }

        // update pendulum
        tau = K_player * (ow[1].get_position() - fp.q1) + B_player * (ow[1].get_velocity() - fp.q1d);
        fp.update(timer.get_elapsed_time(), tau);

        data[0] = fp.tau1;
        data[1] = fp.tau2;
        ms.write_data(data);

        // lock other joints
        double fe_total_torque = 0.0;
        if (ow[1].get_position() >= wall) {
            if (ow[1].get_velocity() > 0)
                fe_total_torque += k_wall * (wall - ow[1].get_position()) + b_wall * (0 - ow[1].get_velocity());
            else
                fe_total_torque += k_wall * (wall - ow[1].get_position());
        }
        else if (ow[1].get_position() <= -wall) {
            if (ow[1].get_velocity() < 0)
                fe_total_torque += k_wall * (-wall - ow[1].get_position()) + b_wall * (0 - ow[1].get_velocity());
            else
                fe_total_torque += k_wall * (-wall - ow[1].get_position());
        }

        // set FE torque due to pendulum
        fe_total_torque += -fp.tau1;
        ow[1].set_torque(fe_total_torque);

        // render walls
        ow[0].set_torque(pd1.move_to_hold(0, ow[0].get_position(),
            60 * mel::DEG2RAD, ow[0].get_velocity(),
            0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));

        ow[2].set_torque(pd2.move_to_hold(mel::DEG2RAD * 0, ow[2].get_position(),
            60 * mel::DEG2RAD, ow[2].get_velocity(),
            0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));


        // record user data when D held
        if (Keyboard::is_key_pressed(Key::D) && !recording) {
            log.clear_data();
            log.buffer(fp.data_state_);
            recording = true;
        }
        else if (Keyboard::is_key_pressed(Key::D) && recording) {
            log.buffer(fp.data_state_);
        }
        else if (recording) {
            log.save_data("data", ".", true);
            recording = false;
        }

        // CUFF control
        if (result.count("cuff")) {
            if (result.count("up")) {
                if (fp.upright) {
                    cuff_angle = -(-1.0 * fp.q1 + 4.6172 * fp.q2 - 0.9072 * fp.q1d + 1.2218 * fp.q2d) * 5000;
                }
            }
            else if (result.count("down")) {
                if (r_index < recorded_torque.size()) {
                    if (result.count("pos"))
                        cuff_angle = recorded_position[r_index] * RAD2DEG * cuff_ff_gain_;
                    else
                        cuff_angle = recorded_torque[r_index] * 5000;
                }
                else
                    cuff_angle = 0.0;
            }
            else {
                cuff_angle = ow[1].get_position() * RAD2DEG * cuff_ff_gain_;
            }
            cuff_ref_pos_1_ = offset[0] + cuff_angle;
            cuff_ref_pos_2_ = offset[1] + cuff_angle;
            cuff.set_motor_positions(cuff_ref_pos_1_, cuff_ref_pos_2_, true);
        }

        // Track time
        if (result.count("up")) {
            if (fp.upright) {
                curr_up_time = up_time_clock.get_elapsed_time();
                if (curr_up_time > best_up_time) {
                    best_up_time = curr_up_time;
                }
            }
            else {
                up_time_clock.restart();
            }
        }
        ms_up.write_data({ curr_up_time.as_seconds(), best_up_time.as_seconds() });

        // interement r
        r_index++;

        // check limtis
        //if (ow.any_torque_limit_exceeded())
        //    stop = true;

        // update Q8 output
        q8.update_output();

        // wait timer
        timer.wait();
    }
    return 0;
}
