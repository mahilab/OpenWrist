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
    void reset(double q1_0 = 0.0, double q2_0 = mel::PI, double q1d_0 = 0.0, double q2d_0 = 0.0);

    /// Reads properties from external applications (Unity)
    void read_properties();

    /// Write state
    void write_state();

    /// Determiens if upright
    void determine_upright();

    /// Updates computed properties
    void update_properties();


public:

    // Default Pendulum Properties
    double g        = 9.81;   ///< [m/s^2]
    double rho_link = 10;     ///< [kg/m^3]
    double rho_mass = 10;     ///< [kg/m^3]
    double r_link   = 0.025;  ///< [m]
    double r_mass   = 0.1;    ///< [m]
    double l1       = 1.0;    ///< [m]
    double l2       = 1.0;    ///< [m]
    double b1       = 0.01;   ///< [Nm-s/rad]
    double b2       = 0.01;   ///< [Nm-s/rad]

    // Computed Properties
    double m1, m2, m2_link, m2_mass, c1, c2, c2_link, c2_mass, Ixx1, Iyy1, Ixx2, Ixx2_link, Ixx2_mass, Iyy2;

    // State Variables
    double q1, q2, q1d, q2d, q1dd, q2dd, tau1, tau2, k1, k2, u1, u2;

    std::vector<double> data_props_, data_state_;

    bool upright;

private:

    // Integrators
    mel::Integrator q1dd_q1d, q2dd_q2d, q1d_q1, q2d_q2;

    // MELShare and Data
    mel::MelShare ms_props_, ms_state_, ms_up_;
};
