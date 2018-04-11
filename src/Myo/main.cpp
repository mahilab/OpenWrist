#include "MyoBand.hpp"  
#include <MEL/Logging/Log.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>
#include <MEII/EMG/MesArray.hpp>

#include "MEL/Daq/Quanser/Q8Usb.hpp"
#include "MEL/Utility/System.hpp"
#include "MEL/Logging/Log.hpp"
#include "MEL/Utility/Console.hpp"
#include "MEII/EMG/MesArray.hpp"
#include "MEL/Communications/Windows/MelShare.hpp"
#include "MEL/Utility/Windows/Keyboard.hpp"
#include "MEII/Classification/EmgDirClassifier.hpp"
#include <MEL/Core/Clock.hpp>
#include <MEL/Logging/DataLogger.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEII/EMG/EmgDataCapture.hpp>

using namespace mel;

using namespace mel;
using namespace meii;

ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

int main(int argc, char *argv[]) {

    // handle inputs 
    std::vector<uint32> emg_channel_numbers = { 0,1,2,3,4,5,6,7 };

    mel::MyoBand myo("my_myo");
    MelShare ms("myo");

    // initialize logger
    mel::init_logger(mel::Verbose);

    // register ctrl-c handler
    // register_ctrl_handler(handler);

    // construct array of Myoelectric Signals    
    MesArray mes(myo.get_channels(emg_channel_numbers));

    // make MelShares
    MelShare ms_mes_env("mes_env");
    MelShare ms_mes_dm("mes_dm");
    MelShare ms_pred_label("pred_label");

    // initialize testing conditions
    Time Ts = milliseconds(1); // sample period
    std::size_t num_classes = 5; // number of active classes
    std::vector<Key> active_keys = { Key::Num1, Key::Num2, Key::Num3, Key::Num4, Key::Num5, Key::Num6 };
    Time mes_active_capture_period = seconds(0.2);
    std::size_t mes_active_capture_window_size = (std::size_t)((unsigned)(mes_active_capture_period.as_seconds() / Ts.as_seconds()));
    mes.resize_buffer(mes_active_capture_window_size);
    std::size_t pred_label = 0;


    // initialize classifier
    bool RMS = true;
    bool MAV = false;
    bool WL = false;
    bool ZC = false;
    bool SSC = false;
    bool AR1 = false;
    bool AR2 = false;
    bool AR3 = false;
    bool AR4 = false;
    EmgDirClassifier dir_classifier(num_classes, emg_channel_numbers.size(), Ts, RMS, MAV, WL, ZC, SSC, AR1, AR2, AR3, AR4, seconds(1.0), seconds(0.2), seconds(0.9));

    myo.enable();


    // construct clock to regulate interaction
    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds((double)((signed)mes.get_buffer_capacity()) * Ts.as_seconds());

    // construct timer in hybrid mode to avoid using 100% CPU
    Timer timer(Ts, Timer::Hybrid);

    // prompt the user for input
    print("Press 'A + target #' to add training data for one target.");
    print("Press 'C + target #' to clear training data for one target.");
    print("Number of targets/classes is:");
    print(num_classes);
    print("Press 'T' to train classifier and begin real-time classification.");
    print("Press 'Escape' to exit.");

    while (!stop) {

        // update all DAQ input channels
        myo.update();

        // emg signal processing
        mes.update_and_buffer();

        // predict state
        if (dir_classifier.update(mes.get_demean())) {
            pred_label = dir_classifier.get_class();
        }

        // clear active data
        for (std::size_t k = 0; k < num_classes; ++k) {
            if (Keyboard::are_all_keys_pressed({ Key::C, active_keys[k] })) {
                if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                    if (dir_classifier.clear_training_data(k)) {
                        LOG(Info) << "Cleared active data for target " + stringify(k + 1) + ".";
                    }
                    keypress_refract_clock.restart();
                }
            }
        }




        // capture active data
        for (std::size_t k = 0; k < num_classes; ++k) {
            if (Keyboard::are_all_keys_pressed({ Key::A, active_keys[k] })) {
                if (mes.is_buffer_full()) {
                    if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                        if (dir_classifier.add_training_data(k, mes.get_dm_buffer_data(mes_active_capture_window_size))) {
                            LOG(Info) << "Added active data for target " + stringify(k + 1) + ".";
                        }
                        keypress_refract_clock.restart();
                    }
                }
            }
        }

        // train the active/rest classifiers
        if (Keyboard::is_key_pressed(Key::T)) {
            if (keypress_refract_clock.get_elapsed_time() > keypress_refract_time) {
                if (dir_classifier.train()) {
                    LOG(Info) << "Trained new active/rest classifier based on given data.";
                }
                keypress_refract_clock.restart();
            }
        }

        // write to MelShares
        ms_mes_env.write_data(mes.get_envelope());
        ms_mes_dm.write_data(mes.get_demean());
        ms_pred_label.write_data({ (double)((signed)(pred_label + 1)) });

        // check for exit key
        if (Keyboard::is_key_pressed(Key::Escape)) {
            stop = true;
        }

        // wait for remainder of sample period
        timer.wait();

    } // end control loop

    return 0;
}

