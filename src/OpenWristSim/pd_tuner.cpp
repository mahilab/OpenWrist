#include <MEL/Core.hpp>
#include <MEL/Math.hpp>
#include "OpenWristSim.hpp"

using namespace mel;

ctrl_bool g_stop(false);

bool my_handler(CtrlEvent event) {
    if (event == CtrlEvent::CtrlC)
        g_stop = true;
    return 1;
}

// Demostates how to use OpenWristSim to interface the OpenWristSim Unity visualization
int main(int argc, char *argv[]) {
    register_ctrl_handler(my_handler);
    int j = 0;                        // joint we will tune (0,1,2)
    double kp = 10;                   // proportional gain
    double kd = 1;                    // derivative gain
    OpenWristSim ow;                  // OpenWrist interface
    ow.set_enabled(true);             // enable it
    Timer timer = Timer(hertz(1000)); // loop timer @ 1000 Hz
    Time t = Time::Zero;              // loop elapsed time
    // trajectory we will tune against
    Waveform traj = Waveform(Waveform::Square, seconds(2), 30 * DEG2RAD);
    // control loop
    while (!g_stop) {
        double q  = ow.get_position(j);      // current position
        double qd = ow.get_velocity(j);      // current velocity
        double q_ref = traj(t);              // target position
        std::vector<double> Tau  ={0,0,0};   // axis torques {0,0,0}
        Tau[j] = kp * (q_ref - q) - kd * qd; // PD torque
        ow.set_torques(Tau);                 // set all joint torques
        t = timer.wait();                    // wait the end of the loop
    }
    return 0;    
}
