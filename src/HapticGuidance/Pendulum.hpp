#pragma once
#include <array>
#include <MEL/Core/Time.hpp>
#include <MEL/Math/Constants.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Math/Integrator.hpp>
#include <MEL/Communications/Windows/MelShare.hpp>

class Pendulum {

public:

    /// Constructor
    Pendulum();

    /// Steps the pendulum simulation
    void step_simulation(mel::Time time, double position_ref, double velocity_ref);

    /// Computes the natural frequency of the pendulum give a mode (0 or 1)
    double natural_frequency(int mode);

    /// Resets the pendulum integrators
    void reset();

public:

    double K_player = 15;                         ///< [N/m]
    double B_player = 1;                          ///< [N-s/m]
    double g = 9.81;                              ///< [m/s^2]
    std::array<double, 2> M  = { 0.01, 0.30 };    ///< [kg]
    std::array<double, 2> L  = { 0.45, 0.3 };     ///< [m]
    std::array<double, 2> B  = { 0.001,0.001 };   ///< [N-s/m]
    std::array<double, 2> Fk = { 0.001,0.001 };   ///< [Nm]

    std::array<double, 2> Qdd = { 0,0 };
    std::array<double, 2> Qd = { 0,0 };
    std::array<double, 2> Q = { -mel::PI / 2  ,0 };
    std::array<double, 2> Tau = { 0, 0 };

private:

    std::array<mel::Integrator, 2> Qdd2Qd = { mel::Integrator(Qd[0]), mel::Integrator(Qd[1]) };
    std::array<mel::Integrator, 2> Qd2Q   = { mel::Integrator(Q[0]),  mel::Integrator(Q[1]) };

    mel::MelShare ms_state_;
    std::vector<double> state_data_;

};
