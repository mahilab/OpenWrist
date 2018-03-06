#include "BallAndBeam.hpp"

double alpha = 2.0 / 5.0;

void BallAndBeam::step_simulation(mel::Time time, double position_ref, double velocity_ref) {
    // compute torque applied to beam
    tau = K_player * (position_ref - q) + B_player * (velocity_ref - qd);

    // evaluate equations of motion
    rdd =
    qdd = -((r*qd*qd - g*sin(q))/(alpha + 1) - (2*m*qd*r*rd - tau + R*g*m*sin(q) + g*m*r*cos(q))/(R*alpha*m))/(R/(alpha + 1) - (alpha*m*R^2 + m*r^2 + I)/(R*alpha*m))

}
