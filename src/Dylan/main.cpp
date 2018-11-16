#include <MEL/Communications/MelShare.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Math/Functions.hpp>
#include <MEL/Utility/Console.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Utility/System.hpp>
#include "OpenWrist.hpp"

//added libraries for implementing a filter
#include <MEL/Math/Butterworth.hpp>
#include <MEL/Math/Filter.hpp>

//added library for matrix math
#include <Eigen/Dense>
#include <iostream>

//added for keyboard interface
#include <MEL/Utility/Windows/Keyboard.hpp>

//added for data logging
#include <MEL/Logging/DataLogger.hpp>

using namespace mel;
using namespace Eigen;

// ctrl-c signal handler
ctrl_bool ctrlc(false);
bool handler(CtrlEvent event) {
    print("Ctrl+C Pressed");
    ctrlc = true;
    return true;
}

// this is my function for measuring the human's applied force
Vector3d getForce(Butterworth& lp_filter0, Butterworth& lp_filter1, Butterworth& lp_filter2, OpenWrist& ow) {
    
    VectorXd TORQUE_THRESHOLD(3);
    TORQUE_THRESHOLD << 0.3, 0.2, 0.3;

    Vector3d u_h(3);
    u_h(0) = -lp_filter0.update(ow[0].get_torque_sense());
    u_h(1) = -lp_filter1.update(ow[1].get_torque_sense());
    u_h(2) = -lp_filter2.update(ow[2].get_torque_sense());
    for (int index = 0; index < 3; index++) {
        if (std::abs(u_h(index)) <= TORQUE_THRESHOLD(index)) {
            u_h(index) = 0.0;
        }
        else if (u_h(index) > TORQUE_THRESHOLD(index)) {
            u_h(index) -= TORQUE_THRESHOLD(index);
        }
        else {
            u_h(index) += TORQUE_THRESHOLD(index);
        }
    }
    return u_h;

}

// this is my function for getting the desired pose
VectorXd getDesired(VectorXd theta, double t) {

    double a = theta(0);
    double b = theta(1);
    double c = theta(2);
    double d = theta(3);
    double e = theta(4);
    double f = theta(5);
    double g = theta(6);
    
    VectorXd q_d(3);
    q_d << c, a*mel::cos(g*t)*mel::cos(f) - b*mel::sin(g*t)*mel::sin(f) + d, a*mel::cos(g*t)*mel::sin(f) + b*mel::sin(g*t)*mel::cos(f) + e;
    
    VectorXd qdot_d(3);
    qdot_d << 0, -a*g*mel::sin(g*t)*mel::cos(f) - b*g*mel::cos(g*t)*mel::sin(f), -a*g*mel::sin(g*t)*mel::sin(f) + b*g*mel::cos(g*t)*mel::cos(f);
    
    VectorXd x_d(6);
    x_d << q_d, qdot_d;

    return x_d;

}

// this is my function for getting the Jacobian of the desired trajectory
MatrixXd getJacobian(VectorXd psi, VectorXd x_d, double t) {
    
    double DELTA = 0.01;
    int m = (int)psi.size();
    MatrixXd xdJacobian(6, m);

    for (int index = 0; index < m; index++) {

        VectorXd psi_p = psi;
        psi_p(index) += DELTA;

        VectorXd x_d_p = getDesired(psi_p, t);

        VectorXd partialxd(6);
        partialxd << 1 / DELTA * (x_d_p - x_d);
        xdJacobian.col(index) = partialxd;

    }
    return xdJacobian;

}

// this is my function for logging all the data
// log: time (1), mode (2), u_h (3-5), x (6-11), x_d (12-17), x_d_theta (18-23), psi (24-30), u_r (31-33)
void getData(std::vector<double>& row, double count, double mode, Vector3d u_h, std::vector<double>& x, VectorXd x_d, VectorXd x_d_theta, VectorXd psi, Vector3d u_r) {
    row[0] = count;
    row[1] = mode;
    row[2] = u_h(0);
    row[3] = u_h(1);
    row[4] = u_h(2);
    row[5] = x[0];
    row[6] = x[1];
    row[7] = x[2];
    row[8] = x[3];
    row[9] = x[4];
    row[10] = x[5];
    row[11] = x_d(0);
    row[12] = x_d(1);
    row[13] = x_d(2);
    row[14] = x_d(3);
    row[15] = x_d(4);
    row[16] = x_d(5);
    row[17] = x_d_theta(0);
    row[18] = x_d_theta(1);
    row[19] = x_d_theta(2);
    row[20] = x_d_theta(3);
    row[21] = x_d_theta(4);
    row[22] = x_d_theta(5);
    row[23] = psi(0);
    row[24] = psi(1);
    row[25] = psi(2);
    row[26] = psi(3);
    row[27] = psi(4);
    row[28] = psi(5);
    row[29] = psi(6);
    row[30] = u_r(0);
    row[31] = u_r(1);
    row[32] = u_r(2);
}


int main(int argc, char* argv[]) {

    // make options
    Options options("openwrist_q8usb.exe", "OpenWrist Q8 USB Demo");
    options.add_options()
        ("c,calibrate", "Calibrates the OpenWrist")
        ("l,learn", "Runs the proposed algorithm for changing the desired trajectory")
        ("s,shared", "Runs the shared control method which decreases control gains")
        ("d,deform", "Runs the local deformation method from PITD")
        ("i,impedance", "Runs impedance for training")
        ("h,help", "Prints this help message");

    auto result = options.parse(argc, argv);
    if (result.count("help") > 0) {
        print(options.help());
        return 0;
    }

    // initialize MEL logger
    init_logger();

    // register ctrl-c handler
    register_ctrl_handler(handler);

    // make Q8 USB that's configured for current control with VoltPAQ-X4
    QOptions qoptions;
    qoptions.set_update_rate(QOptions::UpdateRate::Fast);
    qoptions.set_analog_output_mode(0, QOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    qoptions.set_analog_output_mode(1, QOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    qoptions.set_analog_output_mode(2, QOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
    Q8Usb q8(qoptions);
    VoltPaqX4 vpx4(q8.digital_output[{ 0, 1, 2 }],
                   q8.analog_output[{ 0, 1, 2 }],
                   q8.digital_input[{0, 1, 2}],
                   q8.analog_input[{ 0, 1, 2 }]);

    // create OpenWrist and bind Q8 channels to it
    OwConfiguration config(
        q8,
        q8.watchdog,
        q8.encoder[{ 0, 1, 2 }],
        q8.velocity[{ 0, 1, 2 }],
        vpx4.amplifiers
    );
    OpenWrist ow(config);

    // run calibration script
    if (result.count("calibrate") > 0) {
        ow.calibrate(ctrlc);
        disable_realtime();
        return 0;
    }




    // determine whether or not we want to log data
    int record_data;
    std::cout << "Should I save the data? (Press 1 if Yes) \n";
    std::cin >> record_data;

    // set time parameters for all methods
    int T = 1;
    double t_f = 35.0001;

    // initialize the desired trajectory for all methods
    VectorXd psi(7);
    //psi << 0.2, 0.2, 0, 0, 0, 0, 1; // Training
    //psi << 0.2, 0.1, 0, 0.2, 0.2, 0, 1; // TASK 1
    psi << 0.5, 0.0, 0.0, 0.0, 0.0, PI / 8, 1.5; // TASK 2
    //psi << 0.3, 0.3, -0.1, 0, 0, PI/8, 0.8; // TASK 3
    //psi << 0.3, 0, 0, 0.1, 0, 0, 1; // TASK 4

    // initialize the true desired trajectory for all methods
    VectorXd theta(7);
    //theta << 0.4, 0.3, 0.1, 0, 0, 0, 1; // Training
    //theta << 0.5, 0.5, 0.2, 0.0, 0.0, 0, 1; // TASK 1
    theta << 0.5, 0.0, 0.0, 0.0, 0.0, PI/8, 0.5; // TASK 2
    //theta << 0.3, 0.4, -0.1, 0.1, -0.1, PI / 8, 0.8; // TASK 3
    //theta << 0.5, 0, 0, 0.1, 0, 0, 1; // TASK 4

    // notification that the human can start for all methods
    int start_interaction = 0;

    // set up for unity
    MelShare ms_ow_position("ow_position");
    MelShare ms_ow_theta("ow_theta");
    MelShare ms_ow_interaction("ow_start");
    MelShare ms_ow_time("ow_time");
    std::vector<double> ow_position_data(3);
    std::vector<double> ow_theta_data(3);
    std::vector<double> ow_interaction_data(1);
    std::vector<double> ow_time_data(1);

    // construct the keyboard
    Keyboard mykeyboard;
    // create critically damped PD controller for each exis ( Nm/rad , Nm-s/rad )
    PdController pd0(0, 0);
    PdController pd1(0, 0);
    PdController pd2(0, 0);
    // construct filters                        
    Butterworth lp_filter0(2, 0.05, Butterworth::Lowpass);
    Butterworth lp_filter1(2, 0.05, Butterworth::Lowpass);
    Butterworth lp_filter2(2, 0.05, Butterworth::Lowpass);

    // construct the holder for state
    std::vector<double> x(6);

    int subj_num;
    if (record_data == 1) {
        // record the subject number for datalog
        std::cout << "What is the subject number? \n";
        std::cin >> subj_num;
    }
    // start my logger
    DataLogger mylogger;
    std::vector<double> row(33);
    std::fill(row.begin(), row.end(), 0.0);



    // enter proposed learning algorithm
    if (result.count("learn") > 0) {

        // set the learning rate parameters
        VectorXd alphavalues(7);
        alphavalues << 1, 1, 1, 1, 1, 1, 1;
        alphavalues *= 1.0;
        double LAMBDA = 0.0;

        // initialize values
        int interaction_mode = 2;
        double gain_interp = 0.0;
        double t = -T / 1000.0;
        double count = -T / 1000.0;
        int m = (int)psi.size();
        // create the alpha matrix
        MatrixXd alpha(m, m);
        alpha.setZero();
        alpha.diagonal() = alphavalues;
        // create the M matrix
        MatrixXd eye(3, 3);
        eye.setIdentity();
        MatrixXd M(6, 3);
        M << eye, LAMBDA * eye;

        // enable hardware
        q8.enable();
        ow.enable();
        // start watchdog and timer
        q8.watchdog.start();
        Timer timer(milliseconds(T));
        
        // enter control loop
        while (!ctrlc) {

            // kick watchdog
            q8.watchdog.kick();
            // update inputs
            q8.update_input();

            // get the current time
            t += T / 1000.0;
            count += T / 1000.0;
            if (count > t_f) {
                break;
            }

            // get the interaction mode
            if (mykeyboard.is_key_pressed(Key::Num1, false)) {
                interaction_mode = 1;
            }
            else if (mykeyboard.is_key_pressed(Key::Num2, false)) {
                interaction_mode = 2;
            }

            // perform learning method operations
            VectorXd x_d_theta = getDesired(theta, count);
            VectorXd x_d = getDesired(psi, t);
            VectorXd u_h = getForce(lp_filter0, lp_filter1, lp_filter2, ow);
            MatrixXd xdJacobian = getJacobian(psi, x_d, t);

            // get the robot's current state
            x[0] = ow[0].get_position();
            x[1] = ow[1].get_position();
            x[2] = ow[2].get_position();
            x[3] = ow[0].get_velocity();
            x[4] = ow[1].get_velocity();
            x[5] = ow[2].get_velocity();

            // get data for logging
            if (record_data == 1) {
                Vector3d u_r;
                u_r(0) = ow[0].get_torque_command();
                u_r(1) = ow[1].get_torque_command();
                u_r(2) = ow[2].get_torque_command();
                getData(row, count, interaction_mode, u_h, x, x_d, x_d_theta, psi, u_r);
            }

            // perform start up to avoid snap
            if (count < 5) {
                if (gain_interp < 0.25) {
                    gain_interp += 0.0001;
                    pd0.set_gains(gain_interp * 25, gain_interp * 1.15);
                    pd1.set_gains(gain_interp * 20, gain_interp * 1.00);
                    pd2.set_gains(gain_interp * 20, gain_interp * 0.25);
                }
            } 
            else {

                // turn on the interaction notification
                start_interaction = 1;

                // update the parameter psi
                VectorXd psi_delta(m);
                psi_delta = T / 1000.0 * alpha * xdJacobian.transpose() * M * u_h;

                // if we are changing the shape
                if (interaction_mode == 1) {
                    psi.head(m - 1) += psi_delta.head(m - 1);
                }
                
                // if we are changing the timing
                else {
                    // put limits on the timing changes
                    if (psi_delta(m - 1) > 0 && psi(m - 1) < 2.0) {
                        psi_delta(m - 1) = 0.002;
                    }
                    else if (psi_delta(m - 1) < 0 && psi(m - 1) > 0.1) {
                        psi_delta(m - 1) = -0.002;
                    }
                    else {
                        psi_delta(m - 1) = 0;
                    }
                    // make locally invariant wrt timing
                    t = t * psi(m - 1) / (psi(m - 1) + psi_delta(m - 1));
                    psi.tail(1) += psi_delta.tail(1);
                }

            }
            
            // set the controller 
            double torque0 = pd0.calculate(x_d(0), x[0], x_d(3), x[3]);
            double torque1 = pd1.calculate(x_d(1), x[1], x_d(4), x[4]);
            double torque2 = pd2.calculate(x_d(2), x[2], x_d(5), x[5]);
            // set the commanded torques
            ow[0].set_torque(torque0 + ow.compute_gravity_compensation(0));
            ow[1].set_torque(torque1 + ow.compute_gravity_compensation(1));
            ow[2].set_torque(torque2 + ow.compute_gravity_compensation(2));

            // pass to unity
            ow_position_data[0] = x[0];
            ow_position_data[1] = x[1];
            ow_position_data[2] = x[2];
            ms_ow_position.write_data(ow_position_data);
            ow_theta_data[0] = x_d_theta(0);
            ow_theta_data[1] = x_d_theta(1);
            ow_theta_data[2] = x_d_theta(2);
            ms_ow_theta.write_data(ow_theta_data);
            ow_interaction_data[0] = start_interaction;
            ms_ow_interaction.write_data(ow_interaction_data);
            ow_time_data[0] = count;
            ms_ow_time.write_data(ow_time_data);
            
            // check limits
            if (ow.any_limit_exceeded())
                ctrlc = true;
            // update outputs
            q8.update_output();
            if (record_data == 1) {
                mylogger.buffer(row);
            }
            // wait timer
            timer.wait();

        }

        // save the data
        if (record_data == 1) {
            std::string logger_name = "data_learn_" + std::to_string(subj_num) + "_task_2.csv";
            mylogger.save_data(logger_name, "/Users/MAHI/Desktop/Dylan's Data");
        }
        std::cout << "Done!";
        return 0;

    }




    // enter shared control code
    if (result.count("shared") > 0) {

        // initialize values
        bool grav_mode = false;
        int time_since_int = 0;
        double gain_interp = 0.0;
        double t = -T / 1000.0;
        double count = -T / 1000.0;

        // enable hardware
        q8.enable();
        ow.enable();

        // start watchdog and timer
        q8.watchdog.start();
        Timer timer(milliseconds(T));

        // enter control loop
        while (!ctrlc) {
            
            // kick watchdog
            q8.watchdog.kick();
            // update inputs
            q8.update_input();

            // get the current time
            t += T / 1000.0;
            count += T / 1000.0;
            if (count > t_f) {
                break;
            }

            // perform learning method operations
            VectorXd x_d_theta = getDesired(theta, t);
            VectorXd x_d = getDesired(psi, t);
            VectorXd u_h = getForce(lp_filter0, lp_filter1, lp_filter2, ow);

            // get the robot's current state
            x[0] = ow[0].get_position();
            x[1] = ow[1].get_position();
            x[2] = ow[2].get_position();
            x[3] = ow[0].get_velocity();
            x[4] = ow[1].get_velocity();
            x[5] = ow[2].get_velocity();

            // get data for logging
            if (record_data == 1) {
                Vector3d u_r;
                u_r(0) = ow[0].get_torque_command();
                u_r(1) = ow[1].get_torque_command();
                u_r(2) = ow[2].get_torque_command();
                getData(row, count, gain_interp, u_h, x, x_d, x_d_theta, psi, u_r);
            }

            // perform start up to avoid snap
            if (count < 5) {
                if (gain_interp < 0.25) {
                    gain_interp += 0.0001;
                    pd0.set_gains(gain_interp * 25, gain_interp * 1.15);
                    pd1.set_gains(gain_interp * 20, gain_interp * 1.00);
                    pd2.set_gains(gain_interp * 20, gain_interp * 0.25);
                }
            }
            else {

                // turn on the interaction notification
                start_interaction = 1;

                // if there is a human interaction
                if (u_h.norm() > 0.01) {
                    time_since_int = 0;
                    // enter gravity comp mode if not already
                    if (grav_mode == false) {
                        grav_mode = true;
                        gain_interp = 0.0;
                        pd0.set_gains(0.0, 0.0);
                        pd1.set_gains(0.0, 0.0);
                        pd2.set_gains(0.0, 0.0);
                    }
                }
                // if not a human interaction
                else {
                    time_since_int++;
                }
                // determine whether we should leave gravity comp mode
                if (time_since_int > 1000 && grav_mode) {
                    grav_mode = false;
                }
                // increase the gains if not in grav_comp mode
                if (gain_interp < 0.25 && grav_mode == false) {
                    gain_interp += 0.0001;
                    pd0.set_gains(gain_interp * 25, gain_interp * 1.15);
                    pd1.set_gains(gain_interp * 20, gain_interp * 1.00);
                    pd2.set_gains(gain_interp * 20, gain_interp * 0.25);
                }

            }            

            // set the controller 
            double torque0 = pd0.calculate(x_d(0), x[0], x_d(3), x[3]);
            double torque1 = pd1.calculate(x_d(1), x[1], x_d(4), x[4]);
            double torque2 = pd2.calculate(x_d(2), x[2], x_d(5), x[5]);
            // set the commanded torques
            ow[0].set_torque(torque0 + ow.compute_gravity_compensation(0));
            ow[1].set_torque(torque1 + ow.compute_gravity_compensation(1));
            ow[2].set_torque(torque2 + ow.compute_gravity_compensation(2));

            // pass to unity
            ow_position_data[0] = x[0];
            ow_position_data[1] = x[1];
            ow_position_data[2] = x[2];
            ms_ow_position.write_data(ow_position_data);
            ow_theta_data[0] = x_d_theta(0);
            ow_theta_data[1] = x_d_theta(1);
            ow_theta_data[2] = x_d_theta(2);
            ms_ow_theta.write_data(ow_theta_data);
            ow_interaction_data[0] = start_interaction;
            ms_ow_interaction.write_data(ow_interaction_data);
            ow_time_data[0] = count;
            ms_ow_time.write_data(ow_time_data);

            // check limits
            if (ow.any_limit_exceeded())
                ctrlc = true;
            // update outputs
            q8.update_output();
            if (record_data == 1) {
                mylogger.buffer(row);
            }
            // wait timer
            timer.wait();

        }

        // save the data
        if (record_data == 1) {
            std::string logger_name = "data_share_" + std::to_string(subj_num) + "_task_2.csv";
            mylogger.save_data(logger_name, "/Users/MAHI/Desktop/Dylan's Data");
        }
        std::cout << "Done!";
        return 0;

    }




    // enter deform control code
    if (result.count("deform") > 0) {

        // set the deform parameters
        double tau = 1.0;
        double mu = 0.001;

        // precompute the deformation shape
        int r = (int)floor(tau / T * 1000.0);
        MatrixXd A = MatrixXd::Zero(r + 3, r);
        for (int i = 0; i < r; i++) {
            A(i, i) = 1;
            A(i + 1, i) = -3;
            A(i + 2, i) = 3;
            A(i + 3, i) = -1;
        }
        MatrixXd R = A.transpose()*A;
        MatrixXd H = R.inverse() * MatrixXd::Ones(r, 1);
        H /= H.maxCoeff();
        // create the local desired trajectory
        MatrixXd gamma_d(r, 3);
        for (int i = 0; i < r; i++) {
            double t = i / 1000.0;
            VectorXd x_d = getDesired(psi, t);
            gamma_d.row(i) = x_d.head(3);
        }
        // create the dummy matrix for gamma_d
        MatrixXd dummy_gamma_d(r, 3);

        // initialize values
        double gain_interp = 0.0;
        double t = -T / 1000.0;
        double count = -T / 1000.0;

        // enable hardware
        q8.enable();
        ow.enable();

        // start watchdog and timer
        q8.watchdog.start();
        Timer timer(milliseconds(T));

        // enter control loop
        while (!ctrlc) {

            // kick watchdog
            q8.watchdog.kick();
            // update inputs
            q8.update_input();

            // get the current time
            t += T / 1000.0;
            count += T / 1000.0;
            if (count > t_f) {
                break;
            }

            // perform learning method operations
            VectorXd x_d_theta = getDesired(theta, t);
            VectorXd u_h = getForce(lp_filter0, lp_filter1, lp_filter2, ow);

            // get the desired position
            VectorXd q_d(3);
            q_d = gamma_d.row(0);
            // get the desired velocity
            VectorXd q_d_next(3);
            VectorXd qdot_d(3);
            q_d_next = gamma_d.row(1);
            qdot_d = (1000.0 / T) * (q_d_next - q_d);
            // get the desired state
            VectorXd x_d(6);
            x_d << q_d, qdot_d;

            // get the robot's current state
            x[0] = ow[0].get_position();
            x[1] = ow[1].get_position();
            x[2] = ow[2].get_position();
            x[3] = ow[0].get_velocity();
            x[4] = ow[1].get_velocity();
            x[5] = ow[2].get_velocity();

            // get data for logging
            if (record_data == 1) {
                Vector3d u_r;
                u_r(0) = ow[0].get_torque_command();
                u_r(1) = ow[1].get_torque_command();
                u_r(2) = ow[2].get_torque_command();
                getData(row, count, gain_interp, u_h, x, x_d, x_d_theta, psi, u_r);
            }

            // perform start up to avoid snap
            if (count < 5) {
                if (gain_interp < 0.25) {
                    gain_interp += 0.0001;
                    pd0.set_gains(gain_interp * 25, gain_interp * 1.15);
                    pd1.set_gains(gain_interp * 20, gain_interp * 1.00);
                    pd2.set_gains(gain_interp * 20, gain_interp * 0.25);
                }
            }
            else {

                // turn on the interaction notification
                start_interaction = 1;

                // deform the robot's desired trajectory
                gamma_d.col(0) += mu * H * u_h(0);
                gamma_d.col(1) += mu * H * u_h(1);
                gamma_d.col(2) += mu * H * u_h(2);

            }

            // create placeholder for updates desired trajectory
            for (int i = 0; i < r - 1; i++) {
                dummy_gamma_d.row(i) = gamma_d.row(i + 1);
            }
            // append the next point along the original desired trajectory
            VectorXd x_d_new = getDesired(psi, t + T / 1000.0 + tau);
            dummy_gamma_d.row(r - 1) = x_d_new.head(3);
            gamma_d = dummy_gamma_d;

            // set the controller 
            double torque0 = pd0.calculate(x_d(0), x[0], x_d(3), x[3]);
            double torque1 = pd1.calculate(x_d(1), x[1], x_d(4), x[4]);
            double torque2 = pd2.calculate(x_d(2), x[2], x_d(5), x[5]);
            // set the commanded torques
            ow[0].set_torque(torque0 + ow.compute_gravity_compensation(0));
            ow[1].set_torque(torque1 + ow.compute_gravity_compensation(1));
            ow[2].set_torque(torque2 + ow.compute_gravity_compensation(2));

            // pass to unity
            ow_position_data[0] = x[0];
            ow_position_data[1] = x[1];
            ow_position_data[2] = x[2];
            ms_ow_position.write_data(ow_position_data);
            ow_theta_data[0] = x_d_theta(0);
            ow_theta_data[1] = x_d_theta(1);
            ow_theta_data[2] = x_d_theta(2);
            ms_ow_theta.write_data(ow_theta_data);
            ow_interaction_data[0] = start_interaction;
            ms_ow_interaction.write_data(ow_interaction_data);
            ow_time_data[0] = count;
            ms_ow_time.write_data(ow_time_data);

            // check limits
            if (ow.any_limit_exceeded())
                ctrlc = true;
            // update outputs
            q8.update_output();
            if (record_data == 1) {
                mylogger.buffer(row);
            }
            // wait timer
            timer.wait();

        }

        // save the data
        if (record_data == 1) {
            std::string logger_name = "data_deform_" + std::to_string(subj_num) + "_task_2.csv";
            mylogger.save_data(logger_name, "/Users/MAHI/Desktop/Dylan's Data");
        }
        std::cout << "Done!";
        return 0;

    }




    // enter impedance code
    if (result.count("impedance") > 0) {

        // initialize values
        double gain_interp = 0.0;
        double t = -T / 1000.0;
        double count = -T / 1000.0;

        // enable hardware
        q8.enable();
        ow.enable();

        // start watchdog and timer
        q8.watchdog.start();
        Timer timer(milliseconds(T));

        // enter control loop
        while (!ctrlc) {

            // kick watchdog
            q8.watchdog.kick();
            // update inputs
            q8.update_input();

            // get the current time
            t += T / 1000.0;
            count += T / 1000.0;
            if (count > t_f) {
                break;
            }

            // perform learning method operations
            VectorXd x_d_theta = getDesired(theta, t);
            VectorXd x_d = getDesired(psi, t);

            // get the robot's current state
            x[0] = ow[0].get_position();
            x[1] = ow[1].get_position();
            x[2] = ow[2].get_position();
            x[3] = ow[0].get_velocity();
            x[4] = ow[1].get_velocity();
            x[5] = ow[2].get_velocity();

            // perform start up to avoid snap
            if (count < 5) {
                if (gain_interp < 0.25) {
                    gain_interp += 0.0001;
                    pd0.set_gains(gain_interp * 25, gain_interp * 1.15);
                    pd1.set_gains(gain_interp * 20, gain_interp * 1.00);
                    pd2.set_gains(gain_interp * 20, gain_interp * 0.25);
                }
            }
            else {
                // turn on the interaction notification
                start_interaction = 1;
            }

            // set the controller 
            double torque0 = pd0.calculate(x_d(0), x[0], x_d(3), x[3]);
            double torque1 = pd1.calculate(x_d(1), x[1], x_d(4), x[4]);
            double torque2 = pd2.calculate(x_d(2), x[2], x_d(5), x[5]);
            // set the commanded torques
            ow[0].set_torque(torque0 + ow.compute_gravity_compensation(0));
            ow[1].set_torque(torque1 + ow.compute_gravity_compensation(1));
            ow[2].set_torque(torque2 + ow.compute_gravity_compensation(2));

            // pass to unity
            ow_position_data[0] = x[0];
            ow_position_data[1] = x[1];
            ow_position_data[2] = x[2];
            ms_ow_position.write_data(ow_position_data);
            ow_theta_data[0] = x_d_theta(0);
            ow_theta_data[1] = x_d_theta(1);
            ow_theta_data[2] = x_d_theta(2);
            ms_ow_theta.write_data(ow_theta_data);
            ow_interaction_data[0] = start_interaction;
            ms_ow_interaction.write_data(ow_interaction_data);
            ow_time_data[0] = count;
            ms_ow_time.write_data(ow_time_data);

            // check limits
            if (ow.any_limit_exceeded())
                ctrlc = true;
            // update outputs
            q8.update_output();
            // wait timer
            timer.wait();

        }

        std::cout << "Done!";
        return 0;

    }




    // disable Windows realtime
    disable_realtime();
    return 0;

}