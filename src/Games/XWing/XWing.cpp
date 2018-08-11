#include "XWing.hpp"
#include <MEL/Utility/System.hpp>
#include <MEL/Devices/Windows/Keyboard.hpp>
#include <MEL/Math/Functions.hpp>

XWing::XWing(Timer timer, Q8Usb& ow_daq, OpenWrist& ow, ctrl_bool& stop_flag) :
    StateMachine(4),
    timer_(timer),
    ow_daq_(ow_daq),
    ow_(ow),
    stop_flag_(stop_flag),
    state_(6),
    ow_state_("ow_state"),
    impulses_("impulses"),
    impulse_data_(13),
    impulse_data_temp_(13),
    pd0(5, 0.5),
    pd1(5, 0.5),
    pd2(5, 0.125)
{
    sleep(seconds(0.1));
}

void XWing::sf_start(const NoEventData*) {
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

void XWing::sf_play(const NoEventData*) {
    // loop
    while (true) {
        // read OpenWrist DAQ
        ow_daq_.update_input();
        ow_daq_.watchdog.kick();



        // calculate and set compensation torques
        //ow_[0].set_torque(ow_.compute_gravity_compensation(0) + ow_.compute_friction_compensation(0));
        //      ow_[1].set_torque(ow_.compute_gravity_compensation(1) + ow_.compute_friction_compensation(1));
        //      ow_[2].set_torque(ow_.compute_friction_compensation(2) * 0.5);


        tau0 = pd0.move_to_hold(0, ow_[0].get_position(), 30 * DEG2RAD, ow_[0].get_velocity(), 0.001, DEG2RAD, 30 * DEG2RAD);
        tau1 = pd1.move_to_hold(0, ow_[1].get_position(), 30 * DEG2RAD, ow_[1].get_velocity(), 0.001, DEG2RAD, 30 * DEG2RAD);
        tau2 = pd2.move_to_hold(0, ow_[2].get_position(), 30 * DEG2RAD, ow_[2].get_velocity(), 0.001, DEG2RAD, 30 * DEG2RAD);



        ow_[0].set_torque(tau0);
        ow_[1].set_torque(tau1);
        ow_[2].set_torque(tau2);


        impulse_data_ = impulses_.read_data();
        double sum_impulse = sum(impulse_data_);


        double k1 = 4;
        double k2 = 2;
        double k3 = 2;
        double F1 = 0.25;
        double F2 = 0.5;
        double F3 = 0.75;
        double F4 = 1;
        double d1 = 1;
        double d2 = 0.5;
        double d3 = 0.25;

        if (sum_impulse > 0.0) {
            impulse_data_temp_ = impulse_data_;
            pulse_clock_.restart();
        }

        if (pulse_clock_.get_elapsed_time() < seconds(0.05)) {
            if (impulse_data_temp_[0] == 1.0) {
                ow_[0].add_torque(-k1*F1*d1);
                ow_[1].add_torque(k2*F1);
                ow_[2].add_torque(-k3*F1);
            }
            if (impulse_data_temp_[1] == 1.0) {
                ow_[0].add_torque(-k1*F2*d2);
                ow_[1].add_torque(k2*F2);
                ow_[2].add_torque(-k3*F2);
            }
            if (impulse_data_temp_[2] == 1.0) {
                ow_[0].add_torque(-k1*F3*d3);
                ow_[1].add_torque(k2*F3);
                ow_[2].add_torque(-k3*F3);
            }
            if (impulse_data_temp_[3] == 1.0) {
                ow_[0].add_torque(k1*F1*d1);
                ow_[1].add_torque(k2*F1);
                ow_[2].add_torque(k3*F1);
            }
            if (impulse_data_temp_[4] == 1.0) {
                ow_[0].add_torque(k1*F2*d2);
                ow_[1].add_torque(k2*F2);
                ow_[2].add_torque(k3*F2);
            }
            if (impulse_data_temp_[5] == 1.0) {
                ow_[0].add_torque(k1*F3*d3);
                ow_[1].add_torque(k2*F3);
                ow_[2].add_torque(k3*F3);
            }
            if (impulse_data_temp_[6] == 1.0) {
                ow_[0].add_torque(k1*F1*d1);
                ow_[1].add_torque(-k2*F1);
                ow_[2].add_torque(-k3*F1);
            }
            if (impulse_data_temp_[7] == 1.0) {
                ow_[0].add_torque(k1*F2*d2);
                ow_[1].add_torque(-k2*F2);
                ow_[2].add_torque(-k3*F2);
            }
            if (impulse_data_temp_[8] == 1.0) {
                ow_[0].add_torque(k1*F3*d3);
                ow_[1].add_torque(-k2*F3);
                ow_[2].add_torque(-k3*F3);
            }
            if (impulse_data_temp_[9] == 1.0) {
                ow_[0].add_torque(-k1*F1*d1);
                ow_[1].add_torque(-k2*F1);
                ow_[2].add_torque(k3*F1);
            }
            if (impulse_data_temp_[10] == 1.0) {
                ow_[0].add_torque(-k1*F2*d2);
                ow_[1].add_torque(-k2*F2);
                ow_[2].add_torque(k3*F2);
            }
            if (impulse_data_temp_[11] == 1.0) {
                ow_[0].add_torque(-k1*F3*d3);
                ow_[1].add_torque(-k2*F3);
                ow_[2].add_torque(k3*F3);
            }
            if (impulse_data_temp_[12] == 1.0) {
                ow_[2].add_torque(k3*F4);
            }
        }

        for (int i = 0; i<13; i = i + 1) {
            impulses_.write_data({0,0,0,0,0,0,0,0,0,0,0,0,0});
        }


        // update positions/velocities
        state_[0] = ow_[0].get_position();
        state_[1] = ow_[1].get_position();
        state_[2] = ow_[2].get_position();
        state_[3] = ow_[0].get_velocity();
        state_[4] = ow_[1].get_velocity();
        state_[5] = ow_[2].get_velocity();
        ow_state_.write_data(state_);
        // check for stop conditions
        if (stop_flag_ ||
            ow_.any_limit_exceeded() ||
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

void XWing::sf_stop(const NoEventData*) {
    // disable OpenWrist
    ow_.disable();
    // disable OpenWrist DAQ
    ow_daq_.watchdog.stop();
    ow_daq_.disable();
}