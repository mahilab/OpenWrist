#include "Clock.h"
#include "Q8Usb.h"
#include "OpenWrist.h"
#include <boost/program_options.hpp>
#include "Input.h"

using namespace mel;

int main(int argc, char * argv[]) {

    //-------------------------------------------------------------------------
    // PROGRAM OPTIONS
    //-------------------------------------------------------------------------
    boost::program_options::options_description desc("Available Options");
    desc.add_options()
        ("help",        "produces help message")
        ("calibrate",   "calibrate OpenWrist zero position")
        ("transparent", "puts the OpenWrist in transparency mode indefinitely");

    boost::program_options::variables_map var_map;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).allow_unregistered().run(), var_map);
    boost::program_options::notify(var_map);

    if (var_map.count("help")) {
        util::print(desc);
        return 0;
    }

    //-------------------------------------------------------------------------
    // DAQ SETUP
    //-------------------------------------------------------------------------
    util::Input::ignore_ctrl_c();
    util::enable_realtime();

    // ensure that only one Q8 USB is connected
    if (dev::Q8Usb::get_q8_usb_count() == 0) {
        util::print("No Q8 USB detected! Aborting program.");
        return -1;
    }
    else if (dev::Q8Usb::get_q8_usb_count() > 1) {
        util::print("More than one Q8 USB detected! Aborting program.");
        return -1;
    }

    // create Q8Usb
    uint32 id = 0;

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

    core::Daq* q8 = new dev::Q8Usb(id, ai_channels, ao_channels, di_channels, do_channels, enc_channels, options_q8);

    //-------------------------------------------------------------------------
    // OPENWRIST SETUP
    //-------------------------------------------------------------------------
    exo::OpenWrist::Config ow_config;
    for (int i = 0; i < 3; i++) {
        ow_config.enable_[i] = q8->do_(i);
        ow_config.command_[i] = q8->ao_(i);
        ow_config.sense_[i] = q8->ai_(i);
        ow_config.encoder_[i] = q8->encoder_(i);
        ow_config.encrate_[i] = q8->encrate_(i);
        ow_config.amp_gains_[i] = 1;
    }
    exo::OpenWrist open_wrist(ow_config);

    // perform calibration commands if requested by user
    if (var_map.count("calibrate")) {
        open_wrist.calibrate();
        delete q8;
        return 0;
    }

    // put the OpenWrist in transparency mode if requested by user
    if (var_map.count("transparent")) {
        open_wrist.transparency_mode();
        delete q8;
        return 0;
    }
}