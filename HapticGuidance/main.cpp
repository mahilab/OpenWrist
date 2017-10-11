#include "Clock.h"
#include "Q8Usb.h"
#include "OpenWrist.h"
#include "Cuff.h"
#include <boost/program_options.hpp>
#include "Input.h"
#include "HapticGuidanceV2.h"

using namespace mel;

int main(int argc, char * argv[]) {

    //-------------------------------------------------------------------------
    // PROGRAM OPTIONS
    //-------------------------------------------------------------------------
    boost::program_options::options_description desc("Available Options");
    desc.add_options()
        ("help", "produces help message")
        ("subject", boost::program_options::value<int>(), "the subject number, 1-40")
        ("condition", boost::program_options::value<int>(), "tbd")
        ("trial", boost::program_options::value<std::string>(), "the trial to start at, e.g. F1-1, T3-5, G2-12, etc")
        ("calibrate-ow", "calibrates OpenWrist")
        ("calibrate-meii", "calibrates MAHI-Exo II")
        ("calibrate-cuff", "calibrates CUFF");

    boost::program_options::variables_map var_map;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).allow_unregistered().run(), var_map);
    boost::program_options::notify(var_map);

    if (var_map.count("help")) {
        util::print(desc);
        return 0;
    }

    //-------------------------------------------------------------------------
    // ENABLE REALTIME
    //-------------------------------------------------------------------------

    util::Input::ignore_ctrl_c();
    util::enable_realtime();

    //-------------------------------------------------------------------------
    // IDENTIFY Q8 USBs
    //-------------------------------------------------------------------------

    util::print("\nIdentifying connected Q8 USBs ...");

    bool ow_connected = false;
    bool meii_connected = false;

    int q8_count = dev::Q8Usb::get_q8_usb_count();

    util::print("Q8 USBs Detected: " + std::to_string(q8_count));

    uint32 id_ow;
    uint32 id_meii;

    if (q8_count > 0) {
        // check if ME-II Q8-USB is connected
        if (dev::Q8Usb::check_digital_loopback(0, 7)) {
            util::print("The ME-II Q8 USB (w/ digital loopback) was identified as 0.");
            meii_connected = true;
            id_meii = 0;
        }
        if (dev::Q8Usb::check_digital_loopback(1, 7)) {
            if (meii_connected) {
                util::print("The ME-II Q8 USB was identified again as 1. Something is wrong, aborting program.");
                return -1;
            }
            util::print("The ME-II Q8 USB (w/ digital loopback) was identified as 1.");
            meii_connected = true;
            id_meii = 1;
        }

        // infer if OW ME-II is connected
        if (meii_connected && q8_count == 2) {
            ow_connected = true;
            id_ow = 1 - id_meii;
            util::print("The OpenWrist Q8 USB (no digital loopback) was identified as " + std::to_string(id_ow) + ".");
        }
        else if (!meii_connected && q8_count == 1) {
            ow_connected = true;
            id_ow = 0;
            util::print("The OpenWrist Q8 USB (no digital loopback) was identified as 0.");
        }
        else if (!meii_connected && q8_count == 2) {
            util::print("Two Q8 USBs were detected, but neither have a digital loopback. Aborting program.");
            return -1;
        }
    }
    else {
        util::print("No Q8 USBs were detected. Aborting program.");
        //return -1;
    }

    util::print(""); // blank line

    if (ow_connected)
        util::print("OpenWrist Status:   Connected (Q8 USB ID " + std::to_string(id_ow) + ")");
    else 
        util::print("OpenWrist Status:   Not Connected");
    if (meii_connected)
        util::print("MAHI-Exo II Status: Connected (Q8 USB ID " + std::to_string(id_meii) + ")");
    else
        util::print("MAHI-Exo II Status: Not Connected");

    util::Input::acknowledge("\nPress Space to continue.", util::Input::Space);

    //-------------------------------------------------------------------------
    // SETUP HARDWARE
    //-------------------------------------------------------------------------

    // create OpenWrist
    mel::exo::OpenWrist::Config ow_config;
    mel::core::Daq* q8_ow;
    //if (ow_connected) {
        channel_vec  ai_channels = { 0, 1, 2, 3 };
        channel_vec  ao_channels = { 0, 1, 2, 3 };
        channel_vec  di_channels = { 0, 1, 2, 3 };
        channel_vec  do_channels = { 0, 1, 2, 3 };
        channel_vec enc_channels = { 0, 1, 2, 3 };

        dev::Q8Usb::Options options_q8;
        options_q8.update_rate_ = dev::Q8Usb::Options::UpdateRate::Fast_8kHz;
        options_q8.decimation_ = 1;
        options_q8.ao_modes_[0] = dev::Q8Usb::Options::AoMode(dev::Q8Usb::Options::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
        options_q8.ao_modes_[1] = dev::Q8Usb::Options::AoMode(dev::Q8Usb::Options::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
        options_q8.ao_modes_[2] = dev::Q8Usb::Options::AoMode(dev::Q8Usb::Options::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);

        q8_ow = new mel::dev::Q8Usb(id_ow, ai_channels, ao_channels, di_channels, do_channels, enc_channels, options_q8, true);

        for (int i = 0; i < 3; i++) {
            ow_config.enable_[i] = q8_ow->do_(i);
            ow_config.command_[i] = q8_ow->ao_(i);
            ow_config.sense_[i] = q8_ow->ai_(i);
            ow_config.encoder_[i] = q8_ow->encoder_(i);
            ow_config.encrate_[i] = q8_ow->encrate_(i);
            ow_config.amp_gains_[i] = 1;
        }
    //}
    mel::exo::OpenWrist ow(ow_config);

    // create MAHI-EXO II
    mel::exo::MahiExoII::Config meii_config;
    mel::core::Daq* q8_meii;
    if (meii_connected) {
        channel_vec  ai_channels = { 0, 1, 2, 3, 4, 5, 6, 7 };
        channel_vec  ao_channels = { 1, 2, 3, 4, 5 };
        channel_vec  di_channels = { 0, 1, 2, 3, 4, 5, 6, 7 };
        channel_vec  do_channels = { 0, 1, 2, 3, 4, 5, 6, 7 };
        channel_vec enc_channels = { 1, 2, 3, 4, 5 };
        dev::Q8Usb::Options options;
        for (int i = 0; i < 8; ++i) {
            options.do_initial_signals_[i] = 1;
            options.do_final_signals_[i] = 1;
            options.do_expire_signals_[i] = 1;
        }

        q8_meii = new dev::Q8Usb(id_meii, ai_channels, ao_channels, di_channels, do_channels, enc_channels, options, true);

        for (int i = 0; i < 5; ++i) {
            meii_config.enable_[i] = q8_meii->do_(i + 1);
            meii_config.command_[i] = q8_meii->ao_(i + 1);
            meii_config.encoder_[i] = q8_meii->encoder_(i + 1);
            meii_config.encrate_[i] = q8_meii->encrate_(i + 1);
        }

    }
    mel::exo::MahiExoII meii(meii_config);

    // create CUFF
    Cuff cuff("cuff", 4);    

    // create Clock
    mel::util::Clock clock(1000);

    //-------------------------------------------------------------------------
    // CALIBRATION
    //-------------------------------------------------------------------------

    if (ow_connected && var_map.count("calibrate-ow"))
    {
        ow.calibrate();
        return 0;
    }

    if (meii_connected && var_map.count("calibrate-meii"))
    {
        meii.zero_encoders(q8_meii);
        return 0;
    }

    if (var_map.count("calibrate-cuff")) {
        util::print("Calibrating CUFF");
        const short int cuff_normal_force_ = 3;
        short int offset[2];
        short int scaling_factor[2];

        cuff.enable();
        if (!cuff.is_enabled()) {
            util::print("cuff not enabled");
            return -1;
        }
        cuff.pretension(cuff_normal_force_, offset, scaling_factor);
        cuff.disable();
        return 0;
    }

    //-------------------------------------------------------------------------
    // INITIALIZE EXPERIMENT
    //-------------------------------------------------------------------------

    int subject, condition;
    std::string start_trial = "F1-1";

    if (var_map.count("subject") && var_map.count("condition")) {
        subject = var_map["subject"].as<int>();
        condition = var_map["condition"].as<int>();
        if (var_map.count("trial"))
            start_trial = var_map["trial"].as<std::string>();

        HapticGuidanceV2 haptic_guidance(clock, q8_ow, ow, q8_meii, meii, cuff, subject, condition, start_trial);

        // check if we can proceed with the experiment
        bool proceed = true;

        if (!ow_connected)
            proceed = false;

        if (condition == 4 && !meii_connected)
            proceed = false;

        if (proceed)
            haptic_guidance.execute();
        else
            util::print("Could not safely proceed with experiment");
    }
    else {
        mel::util::print("Not enough input parameters were provided to run the experiment.");
    }

    util::disable_realtime();
    delete q8_ow;
    return 0;   
}