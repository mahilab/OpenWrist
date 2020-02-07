#include "OpenWristSim.hpp"
#include <MEL/Math/Constants.hpp>
#include <MEL/Core.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Logging/Log.hpp>

using namespace mel;

OpenWristSim::OpenWristSim() :
    ms_in("openwrist_sim_in"),
    ms_out("openwrist_sim_out"),
    ms_in_data(3,0),
    ms_out_data(12,0)
{
}

OpenWristSim::~OpenWristSim() {
    set_enabled(false);
}

void OpenWristSim::set_enabled(bool _enabled) {
    if (!_enabled) {
        set_torques({0,0,0});
    }
    enabled = _enabled;
}

double OpenWristSim::get_position(int i) {
    ms_out_data = ms_out.read_data();
    if (ms_out_data.size() == 12)
        return ms_out_data[i];
    return 0;
}

double OpenWristSim::get_velocity(int i) {
    ms_out_data = ms_out.read_data();
    if (ms_out_data.size() == 12)
        return ms_out_data[i+3];
    return 0;
}

double OpenWristSim::get_acceleration(int i) {
    ms_out_data = ms_out.read_data();
    if (ms_out_data.size() == 12)
        return ms_out_data[i+6];
    return 0;
}

double OpenWristSim::get_torque(int i) {
    ms_out_data = ms_out.read_data();
    if (ms_out_data.size() == 12)
        return ms_out_data[i+9];
    return 0;
}

void OpenWristSim::set_torque(int i, double tau) {
    if (!enabled)
        return;
    ms_in_data[i] = tau;
    ms_in.write_data(ms_in_data);
}

std::vector<double> OpenWristSim::get_positions() {
    ms_out_data = ms_out.read_data();
    if (ms_out_data.size() == 12)
        return {ms_out_data[0], ms_out_data[1], ms_out_data[2]};
    return {0,0,0};
}

std::vector<double> OpenWristSim::get_velocities() {
    ms_out_data = ms_out.read_data();
    if (ms_out_data.size() == 12)
        return {ms_out_data[3], ms_out_data[4], ms_out_data[5]};
    return {0,0,0};
}

std::vector<double> OpenWristSim::get_accelerations() {
    ms_out_data = ms_out.read_data();
    if (ms_out_data.size() == 12)
        return {ms_out_data[6], ms_out_data[7], ms_out_data[8]};
    return {0,0,0};
}

std::vector<double> OpenWristSim::get_torques() {
    ms_out_data = ms_out.read_data();
    if (ms_out_data.size() == 12)
        return {ms_out_data[9], ms_out_data[10], ms_out_data[11]};
    return {0,0,0};
}

void OpenWristSim::set_torques(const std::vector<double>& Tau) {
    if (!enabled)
        return;
    ms_in.write_data(Tau);
}

void OpenWristSim::calibrate() {

    // create needed variables
    std::array<double, 3> zeros = { 0, 0, 0 }; // determined zero positions for each joint
    std::array<int, 3> dir = { -1 , 1, -1 };   // direction to rotate each joint
    mel::uint32 calibrating_joint = 0;         // joint currently calibrating
    bool returning = false;                    // bool to track if calibrating joint is return to zero
    double pos_ref = 0;                        // desired position
    double vel_ref = 90 * DEG2RAD;             // desired velocity
    std::vector<double> stored_positions;      // stores past positions
    stored_positions.reserve(100000);

    std::array<double, 3> sat_torques = { 2.0, 0.5, 1.0 }; // temporary saturation torques

    Time timeout = seconds(10); // max amout of time we will allow calibration to occur for

    std::array<double,3> pos_limits_neg { -86.1123 * DEG2RAD, -63.2490 * DEG2RAD, -42.0321 * DEG2RAD }; // [rad]
    std::array<double,3> pos_limits_pos { +86.1123 * DEG2RAD, +68.2490 * DEG2RAD, +30.9087 * DEG2RAD }; // [rad]

    // start the clock
    Timer timer(milliseconds(1), Timer::Hybrid);

    // start the calibration control loop
    while (timer.get_elapsed_time() < timeout) {

        // iterate over all joints
        for (std::size_t i = 0; i < 3; i++) {

            // get positions and velocities
            double pos_act = std::round( get_position(i) * 1000 ) / 1000;
            double vel_act = get_velocity(i);

            double torque = 0;
            if (i == calibrating_joint) {

                if (!returning) {

                    // calculate torque req'd to move the calibrating joint forward at constant speed
                    pos_ref += dir[i] * vel_ref * timer.get_period().as_seconds();
                    torque = pd[i].calculate(pos_ref, pos_act, 0, vel_act);
                    torque = saturate(torque, sat_torques[i]);

                    // check if the calibrating joint is still moving
                    stored_positions.push_back(pos_act);
                    bool moving = true;
                    if (stored_positions.size() > 500) {
                        moving = false;
                        for (size_t j = stored_positions.size() - 500; j < stored_positions.size(); j++) {
                            moving = stored_positions[j] != stored_positions[j - 1];
                            if (moving)
                                break;
                        }
                    }

                    // if it's not moving, it's at a hardstop so record the position and deduce the zero location
                    if (!moving) {
                        if (dir[i] > 0)
                            zeros[i] = pos_act - pos_limits_pos[i];
                        else if (dir[i] < 0)
                            zeros[i] = pos_act - pos_limits_neg[i];
                        returning = true;
                    }
                }

                else {
                    // calculate torque req'd to retur the calibrating joint back to zero
                    pos_ref -= dir[i] * vel_ref *  timer.get_period().as_seconds();
                    torque = pd[i].calculate(pos_ref, pos_act, 0, vel_act);
                    torque = saturate(torque, sat_torques[i]);


                    if (dir[i] * pos_ref <= dir[i] * zeros[i]) {
                        // reset for the next joint
                        calibrating_joint += 1;
                        pos_ref = 0;
                        returning = false;
                        LOG(Info) << "Joint " << i << " calibrated";
                    }
                }

            }
            else {
                // lock all other joints at their zero positions
                torque = pd[i].calculate(zeros[i], pos_act, 0, vel_act);
                torque = saturate(torque, sat_torques[i]);

            }
            set_torque(i, torque);
        }
        // wait the clock
        timer.wait();
    }
    set_torques({0,0,0});
}