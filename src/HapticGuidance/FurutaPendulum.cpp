#include "FurutaPendulum.hpp"

#define m1 M[0]
#define m2 M[1]
#define l1 L[0]
#define l2 L[1]
#define b1 B[0]
#define b2 B[1]
#define q1d Qd[0]
#define q2d Qd[1]
#define q1 Q[0]
#define q2 Q[1]
#define tau1 Tau[0]
#define tau2 Tau[1]

FurutaPendulum::FurutaPendulum() :
    ms_props_("pendulum_props"),
    ms_state_("pendulum_state"),
    data_props_(10),
    data_state_(2)
{
    if (ms_props_.read_data().size() != 9) {
        data_props_[0] = K_player;
        data_props_[1] = B_player;
        data_props_[2] = g;
        data_props_[3] = M[0];
        data_props_[4] = M[1];
        data_props_[5] = L[0];
        data_props_[6] = L[1];
        data_props_[7] = B[0];
        data_props_[8] = B[1];
        ms_props_.write_data(data_props_);
    }        
}

void FurutaPendulum::update(mel::Time time, double position_ref, double velocity_ref) {

    // read in properties
    data_props_ = ms_props_.read_data();
    K_player = data_props_[0];
    B_player = data_props_[1];
    g        = data_props_[2]; 
    M[0]     = data_props_[3];
    M[1]     = data_props_[4];
    L[0]     = data_props_[5];
    L[1]     = data_props_[6];
    B[0]     = data_props_[7];
    B[1]     = data_props_[8];

    // compute torque of first joint given reference position and velocity
    Tau[0] = K_player * (position_ref - Q[0]) + B_player * (velocity_ref - Qd[0]);

    // evaluate the equations of motion
    Qdd[0] = (l2*tau1-l1*tau2*cos(q2)-b1*l2*q1d+b2*l1*q2d*cos(q2)+g*l1*l2*m2*sin(q2*2.0)*(1.0/2.0)+l1*(l2*l2)*m2*(q2d*q2d)*sin(q2)-l1*(l2*l2)*m2*(q1d*q1d)*(sin(q2)-pow(sin(q2),3.0))-(l2*l2*l2)*m2*q1d*q2d*sin(q2*2.0))/(l2*((l1*l1)*m1+(l1*l1)*m2+(l2*l2)*m2-(l1*l1)*m2*pow(cos(q2),2.0)-(l2*l2)*m2*pow(cos(q2),2.0)));
    Qdd[1] = (1.0 / (l2*l2)*((l1*l1)*m1*tau2*8.0 + (l1*l1)*m2*tau2*8.0 + (l2*l2)*m2*tau2*4.0 + (l2*l2*l2*l2)*(m2*m2)*(q1d*q1d)*sin(q2*2.0)*2.0 - (l2*l2*l2*l2)*(m2*m2)*(q1d*q1d)*sin(q2*4.0) - (l2*l2)*m2*tau2*cos(q2*2.0)*4.0 - g*(l2*l2*l2)*(m2*m2)*sin(q2)*6.0 - b2*(l1*l1)*m1*q2d*8.0 - b2*(l1*l1)*m2*q2d*8.0 - b2*(l2*l2)*m2*q2d*4.0 + g*(l2*l2*l2)*(m2*m2)*sin(q2*3.0)*2.0 + b2*(l2*l2)*m2*q2d*cos(q2*2.0)*4.0 - g*(l1*l1)*l2*(m2*m2)*sin(q2)*8.0 + (l1*l1)*(l2*l2)*(m2*m2)*(q1d*q1d)*sin(q2*2.0)*4.0 - (l1*l1)*(l2*l2)*(m2*m2)*(q2d*q2d)*sin(q2*2.0)*4.0 - l1*l2*m2*tau1*cos(q2)*8.0 + l1*(l2*l2*l2)*(m2*m2)*q1d*q2d*sin(q2)*4.0 + b1*l1*l2*m2*q1d*cos(q2)*8.0 + l1*(l2*l2*l2)*(m2*m2)*q1d*q2d*sin(q2*3.0)*4.0 - g*(l1*l1)*l2*m1*m2*sin(q2)*8.0 + (l1*l1)*(l2*l2)*m1*m2*(q1d*q1d)*sin(q2*2.0)*4.0)*(1.0 / 4.0)) / (m2*((l1*l1)*m1*2.0 + (l1*l1)*m2 + (l2*l2)*m2 - (l1*l1)*m2*cos(q2*2.0) - (l2*l2)*m2*cos(q2*2.0)));

    // integrate acclerations to find velocities
    Qd[0] = Qdd2Qd[0].update(Qdd[0], time);
    Qd[1] = Qdd2Qd[1].update(Qdd[1], time);

    // integrate velocities to find positions
    Q[0] = Qd2Q[0].update(Qd[0], time);
    Q[1] = Qd2Q[1].update(Qd[1], time);

    // write out state
    data_state_[0] = Q[0];
    data_state_[1] = Q[1];
    ms_state_.write_data(data_state_);
}

void FurutaPendulum::reset() {
    Qdd = { 0,0 };
    Qd =  { 0,0 };
    Q =   { 0, mel::PI };
    Tau = { 0, 0 };
    Qdd2Qd = { mel::Integrator(Qd[0]), mel::Integrator(Qd[1]) };
    Qd2Q = { mel::Integrator(Q[0]), mel::Integrator(Q[1]) };
}
