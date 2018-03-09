#ifndef MEL_BALLANDBEAM_HPP
#define MEL_BALLANDBEAM_HPP

#include <MEL/Utility/Time.hpp>
#include <MEL/Math/Constants.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Math/Integrator.hpp>

class BallAndBeam {
public:

    /// Steps the ball and beam simulation
    void step_simulation(mel::Time time, double position_ref, double velocity_ref);

    /// Resets the ball and beam inegrators
    void reset();

public:

    double K_player = 30;  ///< stiffness between user and beam [N/m]
    double B_player = 1;    ///< damping between user and beam   [N-s/m]
    double g = 9.81;       ///< accerlation due to gravity      [m/s^2]
    double I = 0.025;      ///< beam moment of inertia          [kg*m^2]
    double m = 0.25;       ///< ball mass                       [kg]
    double R = 0.03;       ///< ball radius                     [m]
    double L = 0.8;        ///< beam length                     [m]

    double tau;            ///< torque acting on beam           [Nm]
    double r, rd, rdd;     ///< ball state                      [m, m/s m/s^2]
    double q, qd, qdd;     ///< beam state                      [rad, rad/s, rad/s^2]

private:

    mel::Integrator rdd2rd; ///< integrates r'' to r'
    mel::Integrator rd2r;   ///< integrates r'  to r
    mel::Integrator qdd2qd; ///< integrates q'' to r'
    mel::Integrator qd2q;   ///< integrates q' to q

};


#endif // MEL_BALLANDBEAM_HPP
