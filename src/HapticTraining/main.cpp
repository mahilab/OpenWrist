#include "Cuff/Cuff.hpp"
#include <MEL/Core/Console.hpp>
#include <MEL/Communications/MelShare.hpp>
#include "Simulations/FurutaPendulum.hpp"
#include <MEL/Mechatronics/PdController.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include "OpenWrist.hpp"
#include <MEL/Devices/Windows/Keyboard.hpp>
#include <MEL/Utility/RingBuffer.hpp>
#include <MEL/Math/Waveform.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Devices/Windows/XboxController.hpp>
#include <MEL/Math/Differentiator.hpp>
// #include <MEL/Logging/DataLogger.hpp>
#include <fstream>
#include "HapticTraining.hpp"
#include "PerlinNoise.hpp"

using namespace mel;

int main(int argc, char* argv[]) {

    // set up options
    mel::Options options("haptic_guidance.exe", "Haptic Guidance Experiment");
    options.add_options()
        ("s,subj","Subject Number", value<int>())
        ("c,cond","Condition Number", value<int>())
        ("t,trial","Start Trial Tag Name", value<std::string>())
        ("d,debug", "Debug")
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
    QuanserOptions qoptions;
    qoptions.set_update_rate(QuanserOptions::UpdateRate::Fast);
    qoptions.set_analog_output_mode(0, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    qoptions.set_analog_output_mode(1, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    qoptions.set_analog_output_mode(2, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    Q8Usb q8(qoptions);

    q8.open();

    VoltPaqX4 vpx4(q8.DO[{ 0, 1, 2 }], q8.AO[{ 0, 1, 2 }], q8.DI[{0, 1, 2}], q8.AI[{ 0, 1, 2 }]);

    // create OpenWrist and bind Q8 channels to it
    OwConfiguration config(
        q8,
        q8.watchdog,
        q8.encoder[{ 0, 1, 2 }],
        vpx4.amplifiers
    );
    OpenWrist ow(config);
    Cuff cuff("cuff", 4);

    if (input.count("debug")) {
        Timer timer(hertz(1000));
    
       //std::vector<int> list1 = {1,1,1,1};
       //std::vector<int> list2 = {2,2,2,2,2};
       //std::vector<int> list3 = {3,3,3,3,3};
       //std::vector<int> list4;

       //list4.insert(list4.end(),list1.begin(),list1[2]);
       //list4.insert(list4.end(),list2.begin(),5);
       //list4.insert(list4.end(),list3.begin(),5);
       //std::shuffle(std::begin(list4),std::end(list4),std::default_random_engine(46768));
       //print(list4);
       

        //while (!Keyboard::is_key_pressed(Key::Escape)) {  
        //    timer.wait();
        //}
       
       
    }

    if(!input.count("debug")){
    HapticTraining experiment(q8, ow, cuff, subject_number, condition, start_trial);
    experiment.execute();
    }

}