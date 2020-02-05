#pragma once
#include <MEL/Core/Time.hpp>
#include <MEL/Math/Constants.hpp>
#include <MEL/Math/Integrator.hpp>
#include <MEL/Mechatronics/Limiter.hpp>

class OpenWristSim {
public:

    OpenWristSim();

    void update(mel::Time t);
    void set_torques(double tau1, double tau2, double tau3);
    void set_positions(double q1, double q2, double q3);
    void set_velocities(double qd1, double qd2, double qd3);
    void reset();

public:

    // Joint torques [Nm]
    double tau1, tau2, tau3;
    // Joint Positions [rad]
    double q1, q2, q3;
    // Joint Velocities [rad/s]
    double q1d, q2d, q3d;
    // Joint Accelerations [rad/s^2]
    double q1dd, q2dd, q3dd;

    // Hardstops
    const double q1min = -86.1123 * mel::DEG2RAD;
    const double q2min = -63.2490 * mel::DEG2RAD;
    const double q3min = -42.0321 * mel::DEG2RAD;
    const double q1max = +86.1123 * mel::DEG2RAD;
    const double q2max = +68.2490 * mel::DEG2RAD;
    const double q3max = +30.9087 * mel::DEG2RAD;

    const double Khard = 50; // hardstop stiffness
    const double Bhard = 0.5;  // hardstop damping

    // Joint Mass [kg]
    const double m1 = 1.79265300000000;
    const double m2 = 0.891430000000000;
    const double m3 = 0.192063000000000;

    // Joint Moments of Inertia [kg*m^2]
    const double Ic1xx = 0.00530900000000000;  
    const double Ic1xy = -0.000758000000000000; 
    const double Ic1xz = -0.000687000000000000; 
    const double Ic1yy = 0.00899500000000000; 
    const double Ic1yz = -0.000368000000000000; 
    const double Ic1zz = 0.0103780000000000; 
    const double Ic2xx = 0.00375400000000000; 
    const double Ic2xy = -0.000527000000000000; 
    const double Ic2xz = 7.10000000000000e-05; 
    const double Ic2yy = 0.00151900000000000; 
    const double Ic2yz = -0.000324000000000000; 
    const double Ic2zz = 0.00451600000000000; 
    const double Ic3xx = 0.000419000000000000; 
    const double Ic3xy = -2.60000000000000e-05; 
    const double Ic3xz = -0.000140000000000000; 
    const double Ic3yy = 0.000470000000000000; 
    const double Ic3yz = -2.90000000000000e-05; 
    const double Ic3zz = 0.000287000000000000; 

    // Joint Center of Mass [m]
    const double Pc1x = 0.0224710000000000; 
    const double Pc1y = 0.0404220000000000;
    const double Pc1z = 0.115783000000000;
    const double Pc2x = -0.00945400000000000;
    const double Pc2y = -0.0271490000000000;
    const double Pc2z = -0.0781620000000000;
    const double Pc3x = 0.0493680000000000;
    const double Pc3y = -0.0211170000000000;
    const double Pc3z = 0.0607280000000000;

    // Motor Rotor Inertia [kg-m^2]
    const double Jm1 = 1.37000000000000e-05;
    const double Jm2 = 1.37000000000000e-05;
    const double Jm3 = 3.47000000000000e-06;

    /// Motor continous and max torque limits [Nm]
    const double tau1_mot_cont = 0.187; 
    const double tau1_mot_max  = 2.560;
    const double tau2_mot_cont = 0.187; 
    const double tau2_mot_max  = 2.560;
    const double tau3_mot_cont = 0.0897; 
    const double tau3_mot_max  = 1.050;

    // Transmission Rations [in/in]
    const double eta1 = 8.75 / 0.468;
    const double eta2 = 9.00 / 0.468;
    const double eta3 = 6.00 / 0.234;

    // Damping Coefficients [Nm*s/rad]
    const double b1 = 0.5 * 0.0252;  
    const double b2 = 0.5 * 0.0019;  
    const double b3 = 0.5 * 0.0029; 

    // Kinetic Friction [Nm]
    const double fk1 = 0.5 * 0.1891; 
    const double fk2 = 0.5 * 0.0541;
    const double fk3 = 0.5 * 0.1339; 

    // Gravity Constant [m/s^2]
    const double g = 9.80665;

private:
    // torque limiters
    mel::Limiter lim1, lim2, lim3;

    // Integrators
    mel::Integrator q1dd_q1d, q2dd_q2d, q3dd_q3d, q1d_q1, q2d_q2, q3d_q3;
};