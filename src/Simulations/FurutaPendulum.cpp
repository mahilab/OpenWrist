#include "FurutaPendulum.hpp"

FurutaPendulum::FurutaPendulum() :
    ms_props_("pendulum_props"),
    ms_state_("pendulum_state"),
    data_props_(9),
    data_state_(12)
{
    if (ms_props_.read_data().size() != 9) {
        data_props_[0] = g;
        data_props_[1] = rho_link;
        data_props_[2] = rho_mass;
        data_props_[3] = r_link;
        data_props_[4] = r_mass;
        data_props_[5] = l1;
        data_props_[6] = l2;
        data_props_[7] = b1;
        data_props_[8] = b2;
        ms_props_.write_data(data_props_);
    }
    else {
        read_properties();
    }
    reset();
    update_properties();
    std::cout << "g = " << g << std::endl;
    std::cout << "rho_link = " << rho_link << std::endl;
    std::cout << "rho_mass = " << rho_mass << std::endl;
    std::cout << "r_link = " << r_link << std::endl;
    std::cout << "r_mass = " << r_mass << std::endl;
    std::cout << "m1 = " << m1 << std::endl;
    std::cout << "m2 = " << m2 << std::endl;
    std::cout << "l1 = " << l1 << std::endl;
    std::cout << "l2 = " << l2 << std::endl;
    std::cout << "b1 = " << b1 << std::endl;
    std::cout << "b2 = " << b2 << std::endl;
}

void FurutaPendulum::update(mel::Time time, double tau) {

    // read in properties
    // read_properties();

    // update mass properties
    update_properties();

    // set tau1
    tau1 = tau;

    // evaluate the equations of motion
    q1dd = -(-Ixx2*tau1 - (c2*c2)*m2*tau1 + Ixx2*b1*q1d + (Ixx2*Ixx2)*q1d*q2d*sin(q2*2.0) + b1*(c2*c2)*m2*q1d - (c2*c2)*g*l1*(m2*m2)*sin(q2*2.0)*(1.0 / 2.0) + (c2*c2*c2)*l1*(m2*m2)*(q2d*q2d)*sin(q2) - (c2*c2*c2)*l1*(m2*m2)*(q1d*q1d)*(sin(q2) - pow(sin(q2), 3.0)) + (c2*c2*c2*c2)*(m2*m2)*q1d*q2d*sin(q2*2.0) - Ixx2*Iyy2*q1d*q2d*sin(q2*2.0) - c2*l1*m2*tau2*cos(q2) + b2*c2*l1*m2*q2d*cos(q2) + Ixx2*c2*l1*m2*(q2d*q2d)*sin(q2) - Ixx2*c2*l1*m2*(q1d*q1d)*(sin(q2) - pow(sin(q2), 3.0)) + Iyy2*c2*l1*m2*(q1d*q1d)*(sin(q2) - pow(sin(q2), 3.0)) + Ixx2*(c2*c2)*m2*q1d*q2d*sin(q2*2.0)*2.0 - Iyy2*(c2*c2)*m2*q1d*q2d*sin(q2*2.0)) / (-(Ixx2*Ixx2)*pow(cos(q2), 2.0) + Ixx2*Ixx2 + (c2*c2*c2*c2)*(m2*m2) + Ixx1*Ixx2 - (c2*c2*c2*c2)*(m2*m2)*pow(cos(q2), 2.0) + Ixx2*Iyy2*pow(cos(q2), 2.0) + (c2*c2)*(l1*l1)*(m2*m2) + Ixx2*(c1*c1)*m1 + Ixx1*(c2*c2)*m2 + Ixx2*(c2*c2)*m2*2.0 + Ixx2*(l1*l1)*m2 - Ixx2*(c2*c2)*m2*pow(cos(q2), 2.0)*2.0 + Iyy2*(c2*c2)*m2*pow(cos(q2), 2.0) + (c1*c1)*(c2*c2)*m1*m2 - (c2*c2)*(l1*l1)*(m2*m2)*pow(cos(q2), 2.0));
    q2dd = ((tau2 - b2*q2d + c2*g*m2*sin(q2) + (q1d*q1d)*cos(q2)*sin(q2)*(Ixx2 - Iyy2 + (c2*c2)*m2))*(Ixx1 + Ixx2 + (c1*c1)*m1 + (c2*c2)*m2 + (l1*l1)*m2 - Ixx2*pow(cos(q2), 2.0) + Iyy2*pow(cos(q2), 2.0) - (c2*c2)*m2*pow(cos(q2), 2.0))) / (-(Ixx2*Ixx2)*pow(cos(q2), 2.0) + Ixx2*Ixx2 + (c2*c2*c2*c2)*(m2*m2) + Ixx1*Ixx2 - (c2*c2*c2*c2)*(m2*m2)*pow(cos(q2), 2.0) + Ixx2*Iyy2*pow(cos(q2), 2.0) + (c2*c2)*(l1*l1)*(m2*m2) + Ixx2*(c1*c1)*m1 + Ixx1*(c2*c2)*m2 + Ixx2*(c2*c2)*m2*2.0 + Ixx2*(l1*l1)*m2 - Ixx2*(c2*c2)*m2*pow(cos(q2), 2.0)*2.0 + Iyy2*(c2*c2)*m2*pow(cos(q2), 2.0) + (c1*c1)*(c2*c2)*m1*m2 - (c2*c2)*(l1*l1)*(m2*m2)*pow(cos(q2), 2.0)) - (c2*l1*m2*cos(q2)*(-tau1 + b1*q1d + Ixx2*q1d*q2d*sin(q2*2.0) - Iyy2*q1d*q2d*sin(q2*2.0) + c2*l1*m2*(q2d*q2d)*sin(q2) + (c2*c2)*m2*q1d*q2d*sin(q2*2.0))) / (-(Ixx2*Ixx2)*pow(cos(q2), 2.0) + Ixx2*Ixx2 + (c2*c2*c2*c2)*(m2*m2) + Ixx1*Ixx2 - (c2*c2*c2*c2)*(m2*m2)*pow(cos(q2), 2.0) + Ixx2*Iyy2*pow(cos(q2), 2.0) + (c2*c2)*(l1*l1)*(m2*m2) + Ixx2*(c1*c1)*m1 + Ixx1*(c2*c2)*m2 + Ixx2*(c2*c2)*m2*2.0 + Ixx2*(l1*l1)*m2 - Ixx2*(c2*c2)*m2*pow(cos(q2), 2.0)*2.0 + Iyy2*(c2*c2)*m2*pow(cos(q2), 2.0) + (c1*c1)*(c2*c2)*m1*m2 - (c2*c2)*(l1*l1)*(m2*m2)*pow(cos(q2), 2.0));

    // integrate acclerations to find velocities
    q1d = q1dd_q1d.update(q1dd, time);
    q2d = q2dd_q2d.update(q2dd, time);

    // integrate velocities to find positions
    q1 = q1d_q1.update(q1d, time);
    q2 = q2d_q2.update(q2d, time);

    // wrap angles to [-pi, pi]
    q1 = mel::wrap_to_pi(q1);
    q2 = mel::wrap_to_pi(q2);

    // compute kinetic energies
    k1 = (q1d*q1d)*(Ixx1 + (c1*c1)*m1)*(1.0 / 2.0);
    k2 = Ixx2*(q1d*q1d)*(1.0 / 2.0) + Ixx2*(q2d*q2d)*(1.0 / 2.0) + (c2*c2)*m2*(q1d*q1d)*(1.0 / 2.0) + (c2*c2)*m2*(q2d*q2d)*(1.0 / 2.0) + (l1*l1)*m2*(q1d*q1d)*(1.0 / 2.0) - Ixx2*(q1d*q1d)*pow(cos(q2), 2.0)*(1.0 / 2.0) + Iyy2*(q1d*q1d)*pow(cos(q2), 2.0)*(1.0 / 2.0) - (c2*c2)*m2*(q1d*q1d)*pow(cos(q2), 2.0)*(1.0 / 2.0) - c2*l1*m2*q1d*q2d*cos(q2);

    // compute potential energies
    u1 = 0;
    u2 = c2*g*m2*cos(q2);

    // determine if upright
    determine_upright();

    // write out state
    write_state();
}

void FurutaPendulum::read_properties() {
    data_props_ = ms_props_.read_data();
    g = data_props_[0];
    rho_link = data_props_[1];
    rho_mass = data_props_[2];
    r_link = data_props_[3];
    r_mass = data_props_[4];
    l1 = data_props_[5];
    l2 = data_props_[6];
    b1 = data_props_[7];
    b2 = data_props_[8];
}

void FurutaPendulum::write_state() {
    data_state_[0] = q1;
    data_state_[1] = q2;
    data_state_[2] = q1d;
    data_state_[3] = q2d;
    data_state_[4] = q1dd;
    data_state_[5] = q2dd;
    data_state_[6] = tau1;
    data_state_[7] = tau2;
    data_state_[8] = k1;
    data_state_[9] = k2;
    data_state_[10] = u1;
    data_state_[11] = u2;
    ms_state_.write_data(data_state_);
}

void FurutaPendulum::determine_upright() {
    if (u2 > 0.9 *c2*g*m2 && k2 < 0.05) {
        upright = true;
    }
    else {
        upright = false;
    }
}

void FurutaPendulum::update_properties() {
    m1 = mel::PI * r_link * r_link * l1*rho_link;
    m2_link = mel::PI * r_link * r_link * l2 * rho_link;
    m2_mass = 4.0 / 3.0 * mel::PI * r_mass * r_mass * r_mass * rho_mass;
    m2 = m2_link + m2_mass;
    c1 = l1 * 0.5;
    c2_link = l2 * 0.5;
    c2_mass = l2 + r_mass;
    c2 = (m2_link * c2_link + m2_mass * (c2_mass)) / m2;
    Ixx1 = 1.0 / 12.0 * m1*(3.0 * r_link * r_link + l1 * l1);
    Iyy1 = 0.5 * m1 * r_link * r_link;
    Ixx2_link = 1 / 12 * m2_link*(3.0 * r_link * r_link + l2 *l2);
    Ixx2_mass = 0.4 * m2_mass * r_mass * r_mass;
    Ixx2 = (Ixx2_link + m2_link*(c2_link - c2) * (c2_link - c2)) + (Ixx2_mass + m2_mass*(c2_mass - c2) * (c2_mass - c2));
    Iyy2 = 0.5 * m2_link*r_link *r_link + 0.5 * m2_mass*r_mass * r_mass;
}

void FurutaPendulum::reset(double q1_0, double q2_0, double q1d_0, double q2d_0) {
    q1       = q1_0;
    q2       = q2_0;
    q1d      = q1d_0;
    q2d      = q2d_0;
    q1dd     = 0.0;
    q2dd     = 0.0;
    tau1     = 0.0;
    tau2     = 0.0;
    q1dd_q1d = mel::Integrator(q1d);
    q2dd_q2d = mel::Integrator(q2d);
    q1d_q1   = mel::Integrator(q1);
    q2d_q2   = mel::Integrator(q2);
}
