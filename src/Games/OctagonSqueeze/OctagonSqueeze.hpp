#pragma once
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Utility/StateMachine.hpp>
#include "OpenWrist.hpp"

using namespace mel;

class OctagonSqueeze : public StateMachine {
public:
    OctagonSqueeze(Q8Usb& ow_daq,
                   OpenWrist& ow,
                   ctrl_bool& stop_flag);

private:
    //-------------------------------------------------------------------------
    // STATE MACHINE SETUP
    //-------------------------------------------------------------------------

    // STATES
    enum States { ST_START, ST_PLAY, ST_STOP, ST_NUM_STATES };

    // STATE FUNCTIONS
    void sf_start(const NoEventData*);
    void sf_play(const NoEventData*);
    void sf_stop(const NoEventData*);

    // STATE ACTIONS
    StateAction<OctagonSqueeze, NoEventData, &OctagonSqueeze::sf_start> sa_start;
    StateAction<OctagonSqueeze, NoEventData, &OctagonSqueeze::sf_play> sa_play;
    StateAction<OctagonSqueeze, NoEventData, &OctagonSqueeze::sf_stop> sa_stop;

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

    Clock pulse_clock_;

    double K_couple = 10.0;
    double B_couble = 1.0;

    double px_per_rad_x_ = 1920.0 / (100 * DEG2RAD);
    double px_per_rad_y_ = 1080.0 / (50  * DEG2RAD);

    // MELSHARES
    MelShare ms_real_state_;
    MelShare ms_virt_state_;
    MelShare ms_force_torque_;

    // DATA
    std::vector<double> real_state_;
    std::vector<double> virt_state_;
    std::vector<double> force_torque_data_;
};
