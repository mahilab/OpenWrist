#include "Jedi.hpp"
#include <MEL/Utility/System.hpp>
#include <MEL/Devices/Windows/Keyboard.hpp>

Jedi::Jedi(Timer timer, Q8Usb& ow_daq, OpenWrist& ow, ctrl_bool& stop_flag) :
    StateMachine(4),
    timer_(timer),
    ow_daq_(ow_daq),
    ow_(ow),
    stop_flag_(stop_flag),
    state_({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }),
    ow_state_(55002, 55001, "127.0.0.1", false),
    impulse_(55004, 55003, "127.0.0.1", false)
    //impulse_("impulse")
{
    sleep(seconds(0.1));
}

void Jedi::sf_start(const NoEventData*) {
    // enable OpenWrist DAQ
    ow_daq_.enable();
    if (!ow_daq_.is_enabled()) {
        event(ST_STOP);
        return;
    }
    // enable OpenWrist
    timer_.restart();
    ow_.enable();
    ow_daq_.watchdog.start();
    // transition state
    event(ST_PLAY);
}

void Jedi::sf_play(const NoEventData*) {
    // loop
    while (true) {
        // read OpenWrist DAQ
        ow_daq_.update_input();
        ow_daq_.watchdog.kick();
        // calculate and set compensation torques
        ow_[0].set_torque(ow_.compute_gravity_compensation(0) + ow_.compute_friction_compensation(0));
        ow_[1].set_torque(ow_.compute_gravity_compensation(1) + ow_.compute_friction_compensation(1));
        ow_[2].set_torque(ow_.compute_friction_compensation(2) * 0.5);
        // check for impulse torque
        if (impulse_.receive_message() == "pulse")
            pulse_clock_.restart();
        if (pulse_clock_.get_elapsed_time() < seconds(0.05))
            ow_[2].add_torque(0.75);
        // update positions/velocities
        state_[0] = ow_[0].get_position();
        state_[1] = ow_[1].get_position();
        state_[2] = ow_[2].get_position();
        state_[3] = ow_[0].get_velocity();
        state_[4] = ow_[1].get_velocity();
        state_[5] = ow_[2].get_velocity();
        if (ow_state_.receive_message() == "data")
            ow_state_.send_data(state_);
        // check for stop conditions
        if (stop_flag_ ||
            ow_.any_torque_limit_exceeded() ||
            Keyboard::is_key_pressed(Key::Escape)) {
            event(ST_STOP);
            return;
        }
        // write OpenWrist DAQ
        ow_daq_.update_output();
        // wait Clock
        timer_.wait();
    }
}

void Jedi::sf_stop(const NoEventData*) {
    // disable OpenWrist
    ow_.disable();
    // disable OpenWrist DAQ
    ow_daq_.watchdog.stop();
    ow_daq_.disable();
}