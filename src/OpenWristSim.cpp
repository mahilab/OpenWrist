#include "OpenWristSim.hpp"
#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::Vector3d;

void OpenWristSim::update(mel::Time, double tau1, double tau2, double tau3)
{

    const double sq1 = sin( q1);
    const double sq2 = sin( q2);
    const double sq3 = sin( q3);
    const double cq1 = cos( q1);
    const double cq2 = cos( q2);
    const double cq3 = cos( q3);

}
