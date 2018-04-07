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
    void update(mel::Time time, double position_ref, double velocity_ref);

    /// Resets the pendulum integrators
    void reset();

public:

    double K_player = 25;                        ///< [N/m]
    double B_player = 1;                       ///< [N-s/m]
    double g = 9.81;                             ///< [m/s^2]
    std::array<double, 2> M = { 0.01, 0.05};    ///< [kg]
    std::array<double, 2> L = { 1.5, 1 };     ///< [m]
    std::array<double, 2> B = { 0.001,0.001 };   ///< [N-s/m]

    std::array<double, 2> Qdd = { 0,0 };
    std::array<double, 2> Qd = { 0,0 };
    std::array<double, 2> Q =  { 0 , mel::PI };
    std::array<double, 2> Tau = { 0, 0 };

private:

    std::array<mel::Integrator, 2> Qdd2Qd = { mel::Integrator(Qd[0]), mel::Integrator(Qd[1]) };
    std::array<mel::Integrator, 2> Qd2Q = { mel::Integrator(Q[0]),  mel::Integrator(Q[1]) };

    mel::MelShare ms_props_;
    mel::MelShare ms_state_;

    std::vector<double> data_props_;
    std::vector<double> data_state_;

};
