#pragma once
#include <array>
#include <vector>
#include <MEL/Core/Time.hpp>
#include <MEL/Math/Constants.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Math/Integrator.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>

class FurutaPendulum {

public:

    /// Constructor
    FurutaPendulum();

    /// Steps the pendulum simulation
    void update(mel::Time time, double tau);

    /// Resets the pendulum integrators
    void reset(double q1_0 = 0.0, double q2_0 = 0.0, double q1d_0 = 0.0, double q2d_0 = 0.0);

private:

    /// Reads properties from external applications (Unity)
    void read_properties();

    /// Updates computed properties
    void update_properties();

public:

    // Default Pendulum Properties
    double g        = 9.81;   ///< [m/s^2]
    double rho_link = 10;   ///< [kg/m^3]
    double rho_mass = 10;   ///< [kg/m^3]
    double r_link   = 0.025;  ///< [m]
    double r_mass   = 0.1;    ///< [m]
    double l1       = 1.0;    ///< [m]
    double l2       = 1.0;    ///< [m]
    double b1       = 0.01;   ///< [Nm-s/rad]
    double b2       = 0.01;   ///< [Nm-s/rad]

    // Computed Properties
    double m1, m2, m2_link, m2_mass, c1, c2, c2_link, c2_mass, Ixx1, Iyy1, Ixx2, Ixx2_link, Ixx2_mass, Iyy2;

    // Wall Properties
    double k_wall = 20;
    double b_wall = 1.0;
    double wall   = 40 * mel::DEG2RAD;

    // State Variables
    double q1, q2, q1d, q2d, q1dd, q2dd, tau1, tau2;

private:    

    // Integrators
    mel::Integrator q1dd_q1d, q2dd_q2d, q1d_q1, q2d_q2;

    // MELShare and Data
    mel::MelShare ms_props_, ms_state_;
    std::vector<double> data_props_, data_state_;
};
