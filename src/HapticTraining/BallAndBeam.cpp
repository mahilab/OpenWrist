#include "BallAndBeam.hpp"

double alpha = 2.0 / 5.0;

void BallAndBeam::step_simulation(mel::Time time, double position_ref, double velocity_ref) {
    // compute torque applied to beam
    tau = K_player * (position_ref - q) + B_player * (velocity_ref - qd);

    // evaluate equations of motion
    rdd = -((2 * m*qd*r*rd - tau + R*g*m*sin(q) + g*m*r*cos(q)) / (alpha*m*R*R + m*r *r + I) - (r*qd *qd - g*sin(q)) / R) / ((alpha + 1) / R - (R*alpha*m) / (alpha*m*R *R + m*r*r + I));
    qdd = -((r*qd*qd - g*sin(q)) / (alpha + 1) - (2 * m*qd*r*rd - tau + R*g*m*sin(q) + g*m*r*cos(q)) / (R*alpha*m)) / (R / (alpha + 1) - (alpha*m*R*R + m*r*r + I) / (R*alpha*m));

    // springy hardstops
    if (r > L / 2.0)
        rdd -= abs(r - L / 2.0) * 400 / m + rd * 30;
    else if (r < (-L / 2.0))
        rdd += abs(r - (-L / 2.0)) * 400 / m - rd * 30;

    // integrate acclerations to find velocities
    rd = rdd2rd.update(rdd, time);
    qd = qdd2qd.update(qdd, time);

    // integrate velocities to find positions
    r = rd2r.update(rd, time);
    q = qd2q.update(qd, time);
}

void BallAndBeam::reset() {
    r = 0;
    rd = 0;
    rdd = 0;
    q = 0;
    qd = 0;
    qdd = 0;
    rdd2rd.reset();
    rd2r.reset();
    qdd2qd.reset();
    qd2q.reset();
}
