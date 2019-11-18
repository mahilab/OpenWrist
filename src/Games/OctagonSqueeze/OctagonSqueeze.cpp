#include  "OctagonSqueeze.hpp"
#include <MEL/Devices/Windows/Keyboard.hpp>

OctagonSqueeze::OctagonSqueeze(Q8Usb& ow_daq, OpenWrist& ow, ctrl_bool& stop_flag) :
    StateMachine(4),
    timer_(hertz(1000)),
    ow_daq_(ow_daq),
    ow_(ow),
    stop_flag_(stop_flag),
    ms_real_state_("real_state"),
    ms_virt_state_("virt_state"),
    ms_force_torque_("force_torque"),
    real_state_(4),
    virt_state_(4),
    force_torque_data_(2),
    filters_(4, Butterworth(2, hertz(25), hertz(1000) ))
{

}

void OctagonSqueeze::sf_play(const NoEventData*) {
    // loop
    Time t;
    while (true) {
        // read OpenWrist DAQ
        ow_daq_.update_input();
        ow_daq_.watchdog.kick();
        // lock PS joint
        ow_[0].set_torque(ow_.pd_controllers_[0].move_to_hold(0, ow_[0].get_position(),
            60 * mel::DEG2RAD, ow_[0].get_velocity(),
            0.001, mel::DEG2RAD, 10 * mel::DEG2RAD));
        // calculate and set compensation torques
        ow_[1].set_torque(0);
        ow_[2].set_torque(0);
        // read virtual state
        virt_state_ = ms_virt_state_.read_data();

        // calculate real state
        real_state_[0] = -ow_[1].get_position() * px_per_rad_x_;
        real_state_[1] = ow_[2].get_position() * px_per_rad_y_;
        real_state_[2] = -ow_[1].get_velocity() * px_per_rad_x_;
        real_state_[3] = ow_[2].get_velocity() * px_per_rad_y_;
        // calculate force/torque
        if (virt_state_.size() == 4) {
            for (std::size_t i = 0; i < 4; ++i) 
                virt_state_[i] = filters_[i].update(virt_state_[i], t);
            force_torque_data_[0] = K_couple * (real_state_[0] - virt_state_[0]) + B_couble * (real_state_[2] - virt_state_[2]);
            force_torque_data_[1] = K_couple * (real_state_[1] - virt_state_[1]) + B_couble * (real_state_[3] - virt_state_[3]);
            ow_[1].add_torque(force_torque_data_[0]*0.001);
            ow_[2].add_torque(-force_torque_data_[1]*0.001);
        }


        ms_force_torque_.write_data(force_torque_data_);
        // write real state
        ms_real_state_.write_data(real_state_);
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
        t = timer_.wait();
    }
}

void OctagonSqueeze::sf_start(const NoEventData*) {
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


void OctagonSqueeze::sf_stop(const NoEventData*) {
    // disable OpenWrist
    ow_.disable();
    // disable OpenWrist DAQ
    ow_daq_.watchdog.stop();
    ow_daq_.disable();
}
