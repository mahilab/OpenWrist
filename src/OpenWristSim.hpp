#pragma once
#include <MEL/Core/Time.hpp>
#include <MEL/Math/Constants.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Math/Integrator.hpp>
#include <MEL/Communications/MelShare.hpp>

class OpenWristSim {
public:

    void update(mel::Time, double tau1, double tau2, double tau3);

private:

    double q1, q2, q3, q1d, q2d, q3d, q1dd, q2dd, q3dd;

    double m1 = 1.79265300000000;
    double m2 = 0.891430000000000;
    double m3 = 0.192063000000000;

    double Jm1 = 1.37000000000000e-05;
    double Jm2 = 1.37000000000000e-05;
    double Jm3 = 3.47000000000000e-06;

    double Ic1xx = 0.00530900000000000;  
    double Ic1xy = -0.000758000000000000; 
    double Ic1xz = -0.000687000000000000; 
    double Ic1yy = 0.00899500000000000; 
    double Ic1yz = -0.000368000000000000; 
    double Ic1zz = 0.0103780000000000; 
    double Ic2xx = 0.00375400000000000; 
    double Ic2xy = -0.000527000000000000; 
    double Ic2xz = 7.10000000000000e-05; 
    double Ic2yy = 0.00151900000000000; 
    double Ic2yz = -0.000324000000000000; 
    double Ic2zz = 0.00451600000000000; 
    double Ic3xx = 0.000419000000000000; 
    double Ic3xy = -2.60000000000000e-05; 
    double Ic3xz = -0.000140000000000000; 
    double Ic3yy = 0.000470000000000000; 
    double Ic3yz = -2.90000000000000e-05; 
    double Ic3zz = 0.000287000000000000; 

    double Pc1x = 0.0224710000000000; 
    double Pc1y = 0.0404220000000000;
    double Pc1z = 0.115783000000000;
    double Pc2x = -0.00945400000000000;
    double Pc2y = -0.0271490000000000;
    double Pc2z = -0.0781620000000000;
    double Pc3x = 0.0493680000000000;
    double Pc3y = -0.0211170000000000;
    double Pc3z = 0.0607280000000000;

    double g = 9.80665;

    double eta1 = 8.75 / 0.468;
    double eta2 = 9.00 / 0.468;
    double eta3 = 6.00 / 0.234;

    double B1 = 0.0252;  
    double B2 = 0.0019;  
    double B3 = 0.0029; 

    double fk1 = 0.1891; 
    double fk2 = 0.0541;
    double fk3 = 0.1339; 

    double fs1 = 0.2250;
    double fs2 = 0.0720;
    double fs3 = 0.1180;

};