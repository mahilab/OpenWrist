#pragma once

#include <MEL/Core/Timer.hpp>
#include <MEL/Utility/StateMachine.hpp>
#include "OpenWrist.hpp"
#include <MEL/Communications/MelNet.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Communications/MelShare.hpp>


using namespace mel;

class XWing : public StateMachine {

public:

	XWing(Timer timer, Q8Usb& ow_daq, OpenWrist& ow, ctrl_bool& stop_flag);

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
	StateAction<XWing, NoEventData, &XWing::sf_start> sa_start;
    StateAction<XWing, NoEventData, &XWing::sf_play>  sa_play;
    StateAction<XWing, NoEventData, &XWing::sf_stop>  sa_stop;

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
    ctrl_bool& stop_flag_;

    std::vector<double> state_;
	std::vector<double> impulse_data_;
    std::vector<double> impulse_data_temp_;

    Clock pulse_clock_;

    // MELSHARES
	MelShare ow_state_;
	MelShare impulses_;

    double tau0, tau1, tau2;

    PdController pd0, pd1, pd2;

};
