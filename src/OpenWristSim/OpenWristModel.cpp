#include "OpenWristModel.hpp"
#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::Vector3d;

using namespace mel;

inline double hardstop_torque(double q, double qd, double qmin, double qmax, double K, double B) {
    if (q < qmin)
        return K * (qmin - q) - B * qd;
    else if (q > qmax)
        return K * (qmax - q) - B * qd;
    else
        return 0;
}

OpenWristModel::OpenWristModel() : 
    lim1(tau1_mot_cont * eta1, tau1_mot_max * eta1, seconds(2)),
    lim2(tau2_mot_cont * eta2, tau2_mot_max * eta2, seconds(2)),
    lim3(tau3_mot_cont * eta3, tau3_mot_max * eta3, seconds(2))
{
    reset();
}

void OpenWristModel::update(mel::Time t)
{
    // limit torques
    tau1 = lim1.limit(tau1);
    tau2 = lim2.limit(tau2);
    tau3 = lim3.limit(tau3);

    const double sq1 = std::sin( q1 );
    const double sq2 = std::sin( q2 );
    const double sq3 = std::sin( q3 );
    const double cq1 = std::cos( q1 );
    const double cq2 = std::cos( q2 );
    const double cq3 = std::cos( q3 );

    // Mass matrix
    Matrix3d M;
    M(0,0) = Ic1xx+Ic3xx+Ic2yy+(Pc1x*Pc1x)*m1+(Pc1y*Pc1y)*m1+(Pc2x*Pc2x)*m2+(Pc3x*Pc3x)*m3+(Pc2z*Pc2z)*m2+(Pc3y*Pc3y)*m3+Ic2xx*pow(cos(q2),2.0)-Ic3xx*pow(cos(q2),2.0)-Ic2yy*pow(cos(q2),2.0)+Ic3yy*pow(cos(q2),2.0)+Ic2xy*sin(q2*2.0)+Ic3xx*pow(cos(q2),2.0)*pow(cos(q3),2.0)-Ic3yy*pow(cos(q2),2.0)*pow(cos(q3),2.0)-(Pc2x*Pc2x)*m2*pow(cos(q2),2.0)+(Pc2y*Pc2y)*m2*pow(cos(q2),2.0)-(Pc3y*Pc3y)*m3*pow(cos(q2),2.0)+(Pc3z*Pc3z)*m3*pow(cos(q2),2.0)-Ic3xz*cos(q2)*cos(q3)*sin(q2)*2.0+Ic3yz*cos(q2)*sin(q2)*sin(q3)*2.0+Ic3xy*pow(cos(q2),2.0)*cos(q3)*sin(q3)*2.0+Pc2x*Pc2y*m2*sin(q2*2.0)-(Pc3x*Pc3x)*m3*pow(cos(q2),2.0)*pow(cos(q3),2.0)+(Pc3y*Pc3y)*m3*pow(cos(q2),2.0)*pow(cos(q3),2.0)+Pc3y*Pc3z*m3*cos(q2)*sin(q2)*sin(q3)*2.0+Pc3x*Pc3y*m3*pow(cos(q2),2.0)*cos(q3)*sin(q3)*2.0-Pc3x*Pc3z*m3*cos(q2)*cos(q3)*sin(q2)*2.0;
    M(0,1) = Ic2xz*cos(q2)-Ic3xy*cos(q2)-Ic2yz*sin(q2)+Ic3yz*cos(q3)*sin(q2)+Ic3xz*sin(q2)*sin(q3)+Ic3xy*cos(q2)*pow(cos(q3),2.0)*2.0-Ic3xx*cos(q2)*cos(q3)*sin(q3)+Ic3yy*cos(q2)*cos(q3)*sin(q3)+Pc2x*Pc2z*m2*cos(q2)-Pc3x*Pc3y*m3*cos(q2)-Pc2y*Pc2z*m2*sin(q2)+(Pc3x*Pc3x)*m3*cos(q2)*cos(q3)*sin(q3)-(Pc3y*Pc3y)*m3*cos(q2)*cos(q3)*sin(q3)+Pc3y*Pc3z*m3*cos(q3)*sin(q2)+Pc3x*Pc3z*m3*sin(q2)*sin(q3)+Pc3x*Pc3y*m3*cos(q2)*pow(cos(q3),2.0)*2.0;
    M(0,2) = -Ic3xx*sin(q2)+Ic3xz*cos(q2)*cos(q3)-(Pc3x*Pc3x)*m3*sin(q2)-(Pc3y*Pc3y)*m3*sin(q2)-Ic3yz*cos(q2)*sin(q3)+Pc3x*Pc3z*m3*cos(q2)*cos(q3)-Pc3y*Pc3z*m3*cos(q2)*sin(q3);
    M(1,0) = Ic2xz*cos(q2)-Ic3xy*cos(q2)-Ic2yz*sin(q2)+Ic3yz*cos(q3)*sin(q2)+Ic3xz*sin(q2)*sin(q3)+Ic3xy*cos(q2)*pow(cos(q3),2.0)*2.0-Ic3xx*cos(q2)*cos(q3)*sin(q3)+Ic3yy*cos(q2)*cos(q3)*sin(q3)+Pc2x*Pc2z*m2*cos(q2)-Pc3x*Pc3y*m3*cos(q2)-Pc2y*Pc2z*m2*sin(q2)+(Pc3x*Pc3x)*m3*cos(q2)*cos(q3)*sin(q3)-(Pc3y*Pc3y)*m3*cos(q2)*cos(q3)*sin(q3)+Pc3y*Pc3z*m3*cos(q3)*sin(q2)+Pc3x*Pc3z*m3*sin(q2)*sin(q3)+Pc3x*Pc3y*m3*cos(q2)*pow(cos(q3),2.0)*2.0;
    M(1,1) = Ic2xx+Ic3xx+(Pc2x*Pc2x)*m2+(Pc2y*Pc2y)*m2+(Pc3y*Pc3y)*m3+(Pc3z*Pc3z)*m3-Ic3xx*pow(cos(q3),2.0)+Ic3yy*pow(cos(q3),2.0)-Ic3xy*sin(q3*2.0)+(Pc3x*Pc3x)*m3*pow(cos(q3),2.0)-(Pc3y*Pc3y)*m3*pow(cos(q3),2.0)-Pc3x*Pc3y*m3*sin(q3*2.0);
    M(1,2) = -Ic3yz*cos(q3)-Ic3xz*sin(q3)-Pc3y*Pc3z*m3*cos(q3)-Pc3x*Pc3z*m3*sin(q3);
    M(2,0) = -Ic3xx*sin(q2)+Ic3xz*cos(q2)*cos(q3)-(Pc3x*Pc3x)*m3*sin(q2)-(Pc3y*Pc3y)*m3*sin(q2)-Ic3yz*cos(q2)*sin(q3)+Pc3x*Pc3z*m3*cos(q2)*cos(q3)-Pc3y*Pc3z*m3*cos(q2)*sin(q3);
    M(2,1) = -Ic3yz*cos(q3)-Ic3xz*sin(q3)-Pc3y*Pc3z*m3*cos(q3)-Pc3x*Pc3z*m3*sin(q3);
    M(2,2) = Ic3xx+(Pc3x*Pc3x)*m3+(Pc3y*Pc3y)*m3;

    // Reflected motor rotor inertia
    Matrix3d M_mot =  Matrix3d::Zero();
    M_mot(0,0) = Jm1*eta1*eta1;
    M_mot(1,1) = Jm2*eta2*eta2;
    M_mot(2,2) = Jm3*eta3*eta3;

    // Coriolis vector
    Vector3d V;
    V[0] = -Ic2yz*(q2d*q2d)*cos(q2)-Ic2xz*(q2d*q2d)*sin(q2)+Ic3xy*(q2d*q2d)*sin(q2)-Ic2xy*q1d*q2d*2.0+Ic3yz*(q2d*q2d)*cos(q2)*cos(q3)-Ic3yz*(q3d*q3d)*cos(q2)*cos(q3)+Ic3xz*(q2d*q2d)*cos(q2)*sin(q3)-Ic3xz*(q3d*q3d)*cos(q2)*sin(q3)+Ic3xz*q1d*q2d*cos(q3)*2.0-Ic3yy*q2d*q3d*cos(q2)-Ic3yz*q1d*q2d*sin(q3)*2.0-Ic3xy*(q2d*q2d)*pow(cos(q3),2.0)*sin(q2)*2.0+Ic2xy*q1d*q2d*pow(cos(q2),2.0)*4.0-Ic3xy*q1d*q3d*pow(cos(q2),2.0)*2.0-Ic2xx*q1d*q2d*sin(q2*2.0)+Ic3xx*q1d*q2d*sin(q2*2.0)+Ic2yy*q1d*q2d*sin(q2*2.0)-Ic3yy*q1d*q2d*sin(q2*2.0)-Pc2y*Pc2z*m2*(q2d*q2d)*cos(q2)+Ic3xx*(q2d*q2d)*cos(q3)*sin(q2)*sin(q3)-Ic3yy*(q2d*q2d)*cos(q3)*sin(q2)*sin(q3)-Pc2x*Pc2z*m2*(q2d*q2d)*sin(q2)+Pc3x*Pc3y*m3*(q2d*q2d)*sin(q2)-(Pc3x*Pc3x)*m3*q2d*q3d*cos(q2)*2.0-Pc2x*Pc2y*m2*q1d*q2d*2.0-Ic3xx*q2d*q3d*cos(q2)*pow(cos(q3),2.0)*2.0-Ic3xz*q1d*q2d*pow(cos(q2),2.0)*cos(q3)*4.0+Ic3yy*q2d*q3d*cos(q2)*pow(cos(q3),2.0)*2.0+(Pc2x*Pc2x)*m2*q1d*q2d*sin(q2*2.0)-(Pc2y*Pc2y)*m2*q1d*q2d*sin(q2*2.0)+(Pc3y*Pc3y)*m3*q1d*q2d*sin(q2*2.0)-(Pc3z*Pc3z)*m3*q1d*q2d*sin(q2*2.0)+Ic3yz*q1d*q2d*pow(cos(q2),2.0)*sin(q3)*4.0+Ic3xy*q1d*q3d*pow(cos(q2),2.0)*pow(cos(q3),2.0)*4.0-Ic3xy*q2d*q3d*cos(q2)*cos(q3)*sin(q3)*4.0+Ic3yz*q1d*q3d*cos(q2)*cos(q3)*sin(q2)*2.0+Pc3x*Pc3z*m3*q1d*q2d*cos(q3)*2.0+Ic3xz*q1d*q3d*cos(q2)*sin(q2)*sin(q3)*2.0-Pc3y*Pc3z*m3*q1d*q2d*sin(q3)*2.0-Pc3x*Pc3y*m3*(q2d*q2d)*pow(cos(q3),2.0)*sin(q2)*2.0+(Pc3x*Pc3x)*m3*q2d*q3d*cos(q2)*pow(cos(q3),2.0)*2.0-(Pc3y*Pc3y)*m3*q2d*q3d*cos(q2)*pow(cos(q3),2.0)*2.0-Ic3xx*q1d*q2d*cos(q2)*pow(cos(q3),2.0)*sin(q2)*2.0-Ic3xx*q1d*q3d*pow(cos(q2),2.0)*cos(q3)*sin(q3)*2.0+Ic3yy*q1d*q2d*cos(q2)*pow(cos(q3),2.0)*sin(q2)*2.0+Ic3yy*q1d*q3d*pow(cos(q2),2.0)*cos(q3)*sin(q3)*2.0+Pc2x*Pc2y*m2*q1d*q2d*pow(cos(q2),2.0)*4.0-Pc3x*Pc3y*m3*q1d*q3d*pow(cos(q2),2.0)*2.0+Pc3y*Pc3z*m3*(q2d*q2d)*cos(q2)*cos(q3)-Pc3y*Pc3z*m3*(q3d*q3d)*cos(q2)*cos(q3)-(Pc3x*Pc3x)*m3*(q2d*q2d)*cos(q3)*sin(q2)*sin(q3)+(Pc3y*Pc3y)*m3*(q2d*q2d)*cos(q3)*sin(q2)*sin(q3)+Pc3x*Pc3z*m3*(q2d*q2d)*cos(q2)*sin(q3)-Pc3x*Pc3z*m3*(q3d*q3d)*cos(q2)*sin(q3)+(Pc3x*Pc3x)*m3*q1d*q2d*cos(q2)*pow(cos(q3),2.0)*sin(q2)*2.0-(Pc3y*Pc3y)*m3*q1d*q2d*cos(q2)*pow(cos(q3),2.0)*sin(q2)*2.0+(Pc3x*Pc3x)*m3*q1d*q3d*pow(cos(q2),2.0)*cos(q3)*sin(q3)*2.0-(Pc3y*Pc3y)*m3*q1d*q3d*pow(cos(q2),2.0)*cos(q3)*sin(q3)*2.0-Pc3x*Pc3z*m3*q1d*q2d*pow(cos(q2),2.0)*cos(q3)*4.0+Pc3y*Pc3z*m3*q1d*q2d*pow(cos(q2),2.0)*sin(q3)*4.0+Pc3x*Pc3y*m3*q1d*q3d*pow(cos(q2),2.0)*pow(cos(q3),2.0)*4.0-Ic3xy*q1d*q2d*cos(q2)*cos(q3)*sin(q2)*sin(q3)*4.0-Pc3x*Pc3y*m3*q2d*q3d*cos(q2)*cos(q3)*sin(q3)*4.0+Pc3y*Pc3z*m3*q1d*q3d*cos(q2)*cos(q3)*sin(q2)*2.0+Pc3x*Pc3z*m3*q1d*q3d*cos(q2)*sin(q2)*sin(q3)*2.0-Pc3x*Pc3y*m3*q1d*q2d*cos(q2)*cos(q3)*sin(q2)*sin(q3)*4.0;
    V[1] = Ic2xy*(q1d*q1d)-Ic3xz*(q1d*q1d)*cos(q3)-Ic3xz*(q3d*q3d)*cos(q3)+Ic3yz*(q1d*q1d)*sin(q3)+Ic3yz*(q3d*q3d)*sin(q3)+Ic3xy*q2d*q3d*2.0-Ic2xy*(q1d*q1d)*pow(cos(q2),2.0)*2.0+Ic2xx*(q1d*q1d)*sin(q2*2.0)*(1.0/2.0)-Ic3xx*(q1d*q1d)*sin(q2*2.0)*(1.0/2.0)-Ic2yy*(q1d*q1d)*sin(q2*2.0)*(1.0/2.0)+Ic3yy*(q1d*q1d)*sin(q2*2.0)*(1.0/2.0)+Pc2x*Pc2y*m2*(q1d*q1d)+Ic3xx*q1d*q3d*cos(q2)*2.0-Ic3yy*q1d*q3d*cos(q2)+Ic3xz*(q1d*q1d)*pow(cos(q2),2.0)*cos(q3)*2.0-(Pc2x*Pc2x)*m2*(q1d*q1d)*sin(q2*2.0)*(1.0/2.0)+(Pc2y*Pc2y)*m2*(q1d*q1d)*sin(q2*2.0)*(1.0/2.0)-(Pc3y*Pc3y)*m3*(q1d*q1d)*sin(q2*2.0)*(1.0/2.0)+(Pc3z*Pc3z)*m3*(q1d*q1d)*sin(q2*2.0)*(1.0/2.0)-Ic3yz*(q1d*q1d)*pow(cos(q2),2.0)*sin(q3)*2.0-Ic3xy*q2d*q3d*pow(cos(q3),2.0)*4.0+Ic3xx*q2d*q3d*sin(q3*2.0)-Ic3yy*q2d*q3d*sin(q3*2.0)-Pc3x*Pc3z*m3*(q1d*q1d)*cos(q3)-Pc3x*Pc3z*m3*(q3d*q3d)*cos(q3)+Pc3y*Pc3z*m3*(q1d*q1d)*sin(q3)+Pc3y*Pc3z*m3*(q3d*q3d)*sin(q3)+(Pc3y*Pc3y)*m3*q1d*q3d*cos(q2)*2.0+Ic3xz*q1d*q3d*cos(q3)*sin(q2)*2.0+Pc3x*Pc3y*m3*q2d*q3d*2.0-Ic3yz*q1d*q3d*sin(q2)*sin(q3)*2.0+Ic3xx*(q1d*q1d)*cos(q2)*pow(cos(q3),2.0)*sin(q2)-Ic3yy*(q1d*q1d)*cos(q2)*pow(cos(q3),2.0)*sin(q2)-Pc2x*Pc2y*m2*(q1d*q1d)*pow(cos(q2),2.0)*2.0-Ic3xx*q1d*q3d*cos(q2)*pow(cos(q3),2.0)*2.0+Ic3yy*q1d*q3d*cos(q2)*pow(cos(q3),2.0)*2.0-(Pc3x*Pc3x)*m3*q2d*q3d*sin(q3*2.0)+(Pc3y*Pc3y)*m3*q2d*q3d*sin(q3*2.0)-Ic3xy*q1d*q3d*cos(q2)*cos(q3)*sin(q3)*4.0-(Pc3x*Pc3x)*m3*(q1d*q1d)*cos(q2)*pow(cos(q3),2.0)*sin(q2)+(Pc3y*Pc3y)*m3*(q1d*q1d)*cos(q2)*pow(cos(q3),2.0)*sin(q2)+Pc3x*Pc3z*m3*(q1d*q1d)*pow(cos(q2),2.0)*cos(q3)*2.0-Pc3y*Pc3z*m3*(q1d*q1d)*pow(cos(q2),2.0)*sin(q3)*2.0+(Pc3x*Pc3x)*m3*q1d*q3d*cos(q2)*pow(cos(q3),2.0)*2.0-(Pc3y*Pc3y)*m3*q1d*q3d*cos(q2)*pow(cos(q3),2.0)*2.0-Pc3x*Pc3y*m3*q2d*q3d*pow(cos(q3),2.0)*4.0+Ic3xy*(q1d*q1d)*cos(q2)*cos(q3)*sin(q2)*sin(q3)*2.0+Pc3x*Pc3z*m3*q1d*q3d*cos(q3)*sin(q2)*2.0-Pc3y*Pc3z*m3*q1d*q3d*sin(q2)*sin(q3)*2.0+Pc3x*Pc3y*m3*(q1d*q1d)*cos(q2)*cos(q3)*sin(q2)*sin(q3)*2.0-Pc3x*Pc3y*m3*q1d*q3d*cos(q2)*cos(q3)*sin(q3)*4.0;
    V[2] = -Ic3xy*(q2d*q2d)+Ic3xy*(q1d*q1d)*pow(cos(q2),2.0)+Ic3xy*(q2d*q2d)*pow(cos(q3),2.0)*2.0-Ic3xx*(q2d*q2d)*sin(q3*2.0)*(1.0/2.0)+Ic3yy*(q2d*q2d)*sin(q3*2.0)*(1.0/2.0)-Pc3x*Pc3y*m3*(q2d*q2d)-Ic3xx*q1d*q2d*cos(q2)*2.0+Ic3yy*q1d*q2d*cos(q2)+(Pc3x*Pc3x)*m3*(q2d*q2d)*sin(q3*2.0)*(1.0/2.0)-(Pc3y*Pc3y)*m3*(q2d*q2d)*sin(q3*2.0)*(1.0/2.0)-Ic3xy*(q1d*q1d)*pow(cos(q2),2.0)*pow(cos(q3),2.0)*2.0-Ic3yz*(q1d*q1d)*cos(q2)*cos(q3)*sin(q2)-Ic3xz*(q1d*q1d)*cos(q2)*sin(q2)*sin(q3)-(Pc3y*Pc3y)*m3*q1d*q2d*cos(q2)*2.0-Ic3xz*q1d*q2d*cos(q3)*sin(q2)*2.0+Ic3yz*q1d*q2d*sin(q2)*sin(q3)*2.0+Ic3xx*(q1d*q1d)*pow(cos(q2),2.0)*cos(q3)*sin(q3)-Ic3yy*(q1d*q1d)*pow(cos(q2),2.0)*cos(q3)*sin(q3)+Pc3x*Pc3y*m3*(q1d*q1d)*pow(cos(q2),2.0)+Pc3x*Pc3y*m3*(q2d*q2d)*pow(cos(q3),2.0)*2.0+Ic3xx*q1d*q2d*cos(q2)*pow(cos(q3),2.0)*2.0-Ic3yy*q1d*q2d*cos(q2)*pow(cos(q3),2.0)*2.0+Ic3xy*q1d*q2d*cos(q2)*cos(q3)*sin(q3)*4.0-(Pc3x*Pc3x)*m3*(q1d*q1d)*pow(cos(q2),2.0)*cos(q3)*sin(q3)+(Pc3y*Pc3y)*m3*(q1d*q1d)*pow(cos(q2),2.0)*cos(q3)*sin(q3)-(Pc3x*Pc3x)*m3*q1d*q2d*cos(q2)*pow(cos(q3),2.0)*2.0+(Pc3y*Pc3y)*m3*q1d*q2d*cos(q2)*pow(cos(q3),2.0)*2.0-Pc3x*Pc3y*m3*(q1d*q1d)*pow(cos(q2),2.0)*pow(cos(q3),2.0)*2.0-Pc3x*Pc3z*m3*q1d*q2d*cos(q3)*sin(q2)*2.0+Pc3y*Pc3z*m3*q1d*q2d*sin(q2)*sin(q3)*2.0-Pc3y*Pc3z*m3*(q1d*q1d)*cos(q2)*cos(q3)*sin(q2)-Pc3x*Pc3z*m3*(q1d*q1d)*cos(q2)*sin(q2)*sin(q3)+Pc3x*Pc3y*m3*q1d*q2d*cos(q2)*cos(q3)*sin(q3)*4.0;
   
    // Gravity vector
    Vector3d G;
    G[0] = -g*(-Pc1y*m1*sin(q1)+Pc2z*m2*sin(q1)+Pc1x*m1*cos(q1)+Pc2y*m2*cos(q1)*cos(q2)-Pc3z*m3*cos(q1)*cos(q2)+Pc2x*m2*cos(q1)*sin(q2)+Pc3y*m3*cos(q3)*sin(q1)+Pc3x*m3*sin(q1)*sin(q3)+Pc3x*m3*cos(q1)*cos(q3)*sin(q2)-Pc3y*m3*cos(q1)*sin(q2)*sin(q3));
    G[1] = -g*sin(q1)*(-Pc2y*m2*sin(q2)+Pc3z*m3*sin(q2)+Pc2x*m2*cos(q2)+Pc3x*m3*cos(q2)*cos(q3)-Pc3y*m3*cos(q2)*sin(q3));
    G[2] = g*m3*(Pc3x*cos(q1)*cos(q3)-Pc3y*cos(q1)*sin(q3)+Pc3y*cos(q3)*sin(q1)*sin(q2)+Pc3x*sin(q1)*sin(q2)*sin(q3));

    // Damping
    Vector3d B;
    B[0] = b1*q1d;
    B[1] = b2*q2d;
    B[2] = b3*q3d;

    // Kinetic friction
    Vector3d Fk;
    Fk[0] = fk1*std::tanh(q1d*10);
    Fk[1] = fk2*std::tanh(q2d*10);
    Fk[2] = fk3*std::tanh(q3d*10);    

    // Torque vector
    Vector3d Tau;
    Tau[0] = tau1 + hardstop_torque(q1,q1d,q1min,q1max,Khard,Bhard);
    Tau[1] = tau2 + hardstop_torque(q2,q2d,q2min,q2max,Khard,Bhard);
    Tau[2] = tau3 + hardstop_torque(q3,q3d,q3min,q3max,Khard,Bhard);

    // Solved for accelerations
    // 1) Tau = (M + M_mot) * Qdd + V + G + B + Fk
    // 2) (M + M_mot) * Qdd = Tau - V -G - B - Fk
    // 3) A             x   = b
    Matrix3d A = M + M_mot;
    Vector3d b = Tau - V - G - B - Fk;
    Vector3d x = A.householderQr().solve(b);

    q1dd = x[0];
    q2dd = x[1];
    q3dd = x[2];

    // integrate acclerations to find velocities
    q1d = q1dd_q1d.update(q1dd, t);
    q2d = q2dd_q2d.update(q2dd, t);
    q3d = q3dd_q3d.update(q3dd, t);

    // integrate velocities to find positions
    q1 = q1d_q1.update(q1d, t);
    q2 = q2d_q2.update(q2d, t);
    q3 = q3d_q3.update(q3d, t);

}

void OpenWristModel::set_torques(double _tau1, double _tau2, double _tau3) {
    tau1 = _tau1;
    tau2 = _tau2;
    tau3 = _tau3;
}

void OpenWristModel::set_positions(double _q1, double _q2, double _q3) {
    q1 = _q1;
    q2 = _q2;
    q3 = _q3;
    q1d_q1 = mel::Integrator(q1);
    q2d_q2 = mel::Integrator(q2);
    q3d_q3 = mel::Integrator(q3);
}

void OpenWristModel::set_velocities(double _q1d, double _q2d, double _q3d) {
    q1d = _q1d;
    q2d = _q2d;
    q3d = _q3d;
    q1dd_q1d = mel::Integrator(q1d);
    q2dd_q2d = mel::Integrator(q2d);
    q3dd_q3d = mel::Integrator(q3d);
}

void OpenWristModel::reset() {
    set_torques(0,0,0);
    set_positions(0,0,0);
    set_velocities(0,0,0);
}
