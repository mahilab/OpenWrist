#pragma once

#include <MEL/Utility/Timer.hpp>
#include <MEL/Utility/StateMachine.hpp>
#include <MEL/Exoskeletons/OpenWrist/OpenWrist.hpp>
#include <MEL/Communications/MelNet.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>

using namespace mel;

class Jedi : public StateMachine {

public:

    Jedi(Timer timer, Q8Usb& ow_daq, OpenWrist& ow, bool& stop_flag);

private:

    //-------------------------------------------------------------------------
    // STATE MACHINE SETUP
    //-------------------------------------------------------------------------

    // STATES
    enum States {
        ST_START,
        ST_PLAY,
        ST_STOP,
        ST_NUM_STATES
    };

    // STATE FUNCTIONS
    void sf_start(const NoEventData*);
    void sf_play(const NoEventData*);
    void sf_stop(const NoEventData*);

    // STATE ACTIONS
    StateAction<Jedi, NoEventData, &Jedi::sf_start> sa_start;
    StateAction<Jedi, NoEventData, &Jedi::sf_play>  sa_play;
    StateAction<Jedi, NoEventData, &Jedi::sf_stop>  sa_stop;

    // STATE MAP
    virtual const StateMapRow* get_state_map() {
        static const StateMapRow STATE_MAP[] = {
            &sa_start,
            &sa_play,
            &sa_stop,
        };
        return &STATE_MAP[0];
    }

    //-------------------------------------------------------------------------
    // PRIVATE VARIABLES
    //-------------------------------------------------------------------------

    // HARDWARE TIMER
    Timer timer_;

    // HARDWARE
    Q8Usb& ow_daq_;
    OpenWrist& ow_;

    // STOP FLAG
    bool& stop_flag_;

    std::vector<double> state_;

    Clock pulse_clock_;

    // MELSHARES
    MelNet ow_state_;
    MelNet impulse_;

};