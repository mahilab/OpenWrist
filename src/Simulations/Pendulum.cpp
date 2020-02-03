#include "Pendulum.hpp"


Pendulum::Pendulum() :
    ms_state_("pendulum"),
    state_data_(2) 
{

}

void Pendulum::step_simulation(mel::Time time, double position_ref, double velocity_ref) {

    // compute torque of first joint given reference position and velocity
    Tau[0] = K_player * (position_ref - mel::PI / 2 - Q[0]) + B_player * (velocity_ref - Qd[0]);

    // evaluate the equations of motion
    Qdd[0] = -((L[0] * L[1] * M[1] * sin(Q[1])*pow(Qd[0], 2) - Tau[1] + Fk[1] * tanh(10 * Qd[1]) + B[1] * Qd[1] + g*L[1] * M[1] * cos(Q[0] + Q[1])) / (pow(L[1], 2) * M[1]) - (-L[0] * L[1] * M[1] * sin(Q[1])*pow(Qd[1], 2) - 2 * L[0] * L[1] * M[1] * Qd[0] * sin(Q[1])*Qd[1] - Tau[0] + Fk[0] * tanh(10 * Qd[0]) + B[0] * Qd[0] + g*L[1] * M[1] * cos(Q[0] + Q[1]) + g*L[0] * M[0] * cos(Q[0]) + g*L[0] * M[1] * cos(Q[0])) / (L[1] * M[1] * (L[1] + L[0] * cos(Q[1])))) / ((M[1] * pow(L[1], 2) + L[0] * M[1] * cos(Q[1])*L[1]) / (pow(L[1], 2) * M[1]) - (pow(L[0], 2) * M[0] + pow(L[0], 2) * M[1] + pow(L[1], 2) * M[1] + 2 * L[0] * L[1] * M[1] * cos(Q[1])) / (L[1] * M[1] * (L[1] + L[0] * cos(Q[1]))));
    Qdd[1] = ((-L[0] * L[1] * M[1] * sin(Q[1])*pow(Qd[1], 2) - 2 * L[0] * L[1] * M[1] * Qd[0] * sin(Q[1])*Qd[1] - Tau[0] + Fk[0] * tanh(10 * Qd[0]) + B[0] * Qd[0] + g*L[1] * M[1] * cos(Q[0] + Q[1]) + g*L[0] * M[0] * cos(Q[0]) + g*L[0] * M[1] * cos(Q[0])) / (pow(L[0], 2) * M[0] + pow(L[0], 2) * M[1] + pow(L[1], 2) * M[1] + 2 * L[0] * L[1] * M[1] * cos(Q[1])) - (L[0] * L[1] * M[1] * sin(Q[1])*pow(Qd[0], 2) - Tau[1] + Fk[1] * tanh(10 * Qd[1]) + B[1] * Qd[1] + g*L[1] * M[1] * cos(Q[0] + Q[1])) / (L[1] * M[1] * (L[1] + L[0] * cos(Q[1])))) / (L[1] / (L[1] + L[0] * cos(Q[1])) - (M[1] * pow(L[1], 2) + L[0] * M[1] * cos(Q[1])*L[1]) / (pow(L[0], 2) * M[0] + pow(L[0], 2) * M[1] + pow(L[1], 2) * M[1] + 2 * L[0] * L[1] * M[1] * cos(Q[1])));

    // integrate acclerations to find velocities
    Qd[0] = Qdd2Qd[0].update(Qdd[0], time);
    Qd[1] = Qdd2Qd[1].update(Qdd[1], time);

    // integrate velocities to find positions
    Q[0] = Qd2Q[0].update(Qd[0], time);
    Q[1] = Qd2Q[1].update(Qd[1], time);

    state_data_[0] = Q[0];
    state_data_[1] = Q[1];
    ms_state_.write_data(state_data_);
}

double Pendulum::natural_frequency(int mode) {
    if (mode == 0) // this should be the fast mode
        return sqrt((g*(sqrt((M[0] + M[1])*((L[0] * L[0])*M[0] + (L[0] * L[0])*M[1] + (L[1] * L[1])*M[0] + (L[1] * L[1])*M[1] - L[0] * L[1] * M[0] * 2.0 + L[0] * L[1] * M[1] * 2.0)) + L[0] * M[0] + L[0] * M[1] + L[1] * M[0] + L[1] * M[1])*(0.5)) / (L[0] * L[1] * M[0]));
    else if (mode == 1) // this should be the slow mode
        return sqrt((g*(-sqrt((M[0] + M[1])*((L[0] * L[0])*M[0] + (L[0] * L[0])*M[1] + (L[1] * L[1])*M[0] + (L[1] * L[1])*M[1] - L[0] * L[1] * M[0] * 2.0 + L[0] * L[1] * M[1] * 2.0)) + L[0] * M[0] + L[0] * M[1] + L[1] * M[0] + L[1] * M[1])*(0.5)) / (L[0] * L[1] * M[0]));
}

void Pendulum::reset() {
    Qdd = { 0,0 };
    Qd = { 0,0 };
    Q = { -mel::PI / 2  ,0 };
    Tau = { 0, 0 };
    Qdd2Qd = { mel::Integrator(Qd[0]), mel::Integrator(Qd[1]) };
    Qd2Q = { mel::Integrator(Q[0]), mel::Integrator(Q[1]) };
}
