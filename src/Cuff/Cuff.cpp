#include "Cuff/Cuff.hpp"
#include "definitions.h"
#include "commands.h"
#include <chrono>
#include <iostream>
#include "qbmove_communications.h"

#define CUFF_ID 1

Cuff::Cuff() : Device("cuff"), comm_port_(0) {}
Cuff::Cuff(std::string name, mel::uint32 comm_port) : Device(name), comm_port_(comm_port) {}

Cuff::~Cuff() {
    if (is_enabled())
        disable();
}

bool Cuff::on_enable() {
    std::cout << "Enabling CUFF ...";
    // attempt to open port
    open_port();
    if (port_opened_) {
        /* attempt to activate communications */
        char activated = 0;
        commActivate(&comm_settings_t_, CUFF_ID, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        commGetActivate(&comm_settings_t_, CUFF_ID, &activated);
        if (activated) {
            // set initial motor positions
            short int motpos_zero[2];
            reference_motor_positions_[0] = 0;
            reference_motor_positions_[1] = 0;
            commSetInputs(&comm_settings_t_, CUFF_ID, motpos_zero);
            // start IO thread
            poll_io_ = true;
            io_thread_ = std::thread(&Cuff::io_thread_func, this);
            std::cout << "Done" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return true;
        }
        else {
            std::cout << "Failed. Could not activate communications." << std::endl;
            return false;
        }
    }
    else {
        std::cout << "Failed. Could not open port." << std::endl;
        return false;
    }
}

bool Cuff::on_disable() {
    std::cout << "Disabling CUFF ... ";
    set_motor_positions(0, 0, false);
    commActivate(&comm_settings_t_, CUFF_ID, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    poll_io_ = false;
    io_thread_.join();
    std::cout << "Done" << std::endl;
    return true;
}

void Cuff::open_port() {
    if (!port_opened_) {
        char port[255] = { 'C','O','M','0' };
        port[3] += comm_port_;
        openRS485(&comm_settings_t_, port, 2000000);
        if (comm_settings_t_.file_handle == INVALID_HANDLE_VALUE)
        {
            puts("Couldn't connect to the serial port.");
            port_opened_ = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        port_opened_ = true;
    }
}

// Function to select between all the ports
/*
int Cuff::port_selection() {
    int i;
    int aux_int;
    int num_ports = 0;
    char my_port[255];
    char ports[10][255];
    FILE *file;

    while (1) {
        num_ports = RS485listPorts(ports);

        if (num_ports) {
            puts("\nChoose the serial port for your QB:\n");

            for (i = 0; i < num_ports; ++i) {
                printf("[%d] - %s\n\n", i + 1, ports[i]);
            }
            printf("Serial port: ");
            scanf_s("%d", &aux_int);
            getchar();

            if (aux_int && (aux_int <= num_ports)) {
                strcpy_s(my_port, ports[aux_int - 1]);
            }
            else {
                puts("Choice not available");
                continue;
            }

            file = fopen_s(QBMOVE_FILE, "w+");
            if (file == NULL) {
                printf("Cannot open qbmove.conf\n");
            }
            fprintf(file, "serialport %s\n", my_port);
            fclose(file);
            return 1;

        }
        else {
            puts("No serial port available.");
            return 0;
        }
    }
}

*/


void Cuff::pretension(int force_newtons, short int* motpos_zero, short int* scaling_factor) {
    std::cout << "Pretensioning CUFF ... ";
    short int stepmot, act_mot_pos_0, act_mot_pos_1, act_mot_cur_0, act_mot_cur_1;
	std::chrono::high_resolution_clock::time_point tstart, tend;
    std::chrono::nanoseconds elapsed_time;
	int elapsed_time_us;
    int MAX_VAL_CAR_0 = 570;
    int MAX_VAL_CAR_1 = 570;

	motpos_zero[0] = 0;
	motpos_zero[1] = 0;

	tstart = std::chrono::high_resolution_clock::now();
   
	for (int i = 0; i < 80; i++) {
        get_motor_currents(act_mot_cur_0, act_mot_cur_1, false);
		motpos_zero[0] = motpos_zero[0] - (10 * (50 - act_mot_cur_0));
		motpos_zero[1] = motpos_zero[1] + (10 * (50 + act_mot_cur_1));
        //std::cout << act_mot_cur_0 << '\t' << act_mot_cur_1 << '\t' << motpos_zero[0] << '\t' << motpos_zero[1] << '\n';
        set_motor_positions(motpos_zero[0], motpos_zero[1], false);
		tend = std::chrono::high_resolution_clock::now();
		elapsed_time = std::chrono::high_resolution_clock::now() - tstart;
		elapsed_time_us = (int)((double)elapsed_time.count())/1000;
		std::this_thread::sleep_for(std::chrono::microseconds (50000 - elapsed_time_us));
		tstart = std::chrono::high_resolution_clock::now();
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(25));

	stepmot = 0;
	tstart = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < 80; i++) {
		stepmot = stepmot + 15;
		if (stepmot < 1200) {
			motpos_zero[0] = motpos_zero[0] + 15;
			motpos_zero[1] = motpos_zero[1] - 15;
            set_motor_positions(motpos_zero[0], motpos_zero[1], false);
		}
        tend = std::chrono::high_resolution_clock::now();
        elapsed_time = std::chrono::high_resolution_clock::now() - tstart;
        elapsed_time_us = (int)((double)elapsed_time.count()) / 1000;
        std::this_thread::sleep_for(std::chrono::microseconds(50000 - elapsed_time_us));
        tstart = std::chrono::high_resolution_clock::now();
	}

    short int motpos[2];
    motpos[0] = motpos_zero[0] - 20000;
    motpos[1] = motpos_zero[1] + 20000;
    set_motor_positions(motpos[0], motpos[1], false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    get_motor_currents(act_mot_pos_0, act_mot_pos_1, false);
    scaling_factor[0] = abs((act_mot_pos_0 - motpos_zero[0]) / MAX_VAL_CAR_0);
    scaling_factor[1] = abs((act_mot_pos_1 - motpos_zero[1]) / MAX_VAL_CAR_1);
    set_motor_positions(motpos_zero[0], motpos_zero[1], false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    motpos_zero[0] = motpos_zero[0] - (0.1138*pow(force_newtons, 3) - 5.204*pow(force_newtons, 2) + 89.22*force_newtons + 0) * scaling_factor[0];
    motpos_zero[1] = motpos_zero[1] + (0.1138*pow(force_newtons, 3) - 5.204*pow(force_newtons, 2) + 89.22*force_newtons + 0) * scaling_factor[1];
    set_motor_positions(motpos_zero[0], motpos_zero[1], false);
    std::cout << "Done" << std::endl;
}

void Cuff::cazpretension(int force_newtons, short int* motpos_zero, short int* scaling_factor) {

    std::cout << "Pretensioning CUFF ... ";
    short int act_mot_pos_0, act_mot_pos_1, act_mot_cur_0, act_mot_cur_1;//true values to read
    int MAX_VAL_CAR_0 = 570;
    int MAX_VAL_CAR_1 = 570;

	motpos_zero[0] = 0;
	motpos_zero[1] = 0;

    get_motor_currents(act_mot_cur_0,act_mot_cur_1,false);

    while(act_mot_cur_0<50 || act_mot_cur_1> -50) {//start with increments of 600
            get_motor_currents(act_mot_cur_0,act_mot_cur_1,false);

            if(act_mot_cur_0<50)
            motpos_zero[0] = motpos_zero[0] - 600;
            if(act_mot_cur_1>-50)
            motpos_zero[1] = motpos_zero[1] + 600;

           // std::cout << act_mot_cur_0 << '\t' << act_mot_cur_1 << '\t' << motpos_zero[0] << '\t' << motpos_zero[1] << '\n';
            set_motor_positions(motpos_zero[0], motpos_zero[1], false);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        motpos_zero[0] = motpos_zero[0] + 1200;//back up a bit
        motpos_zero[1] = motpos_zero[1] - 1200;
        set_motor_positions(motpos_zero[0], motpos_zero[1], false);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        get_motor_currents(act_mot_cur_0,act_mot_cur_1,false);

	while(act_mot_cur_0<50 || act_mot_cur_1> -50) {//then do increments of 300
        get_motor_currents(act_mot_cur_0,act_mot_cur_1,false);

        if(act_mot_cur_0<50)
		motpos_zero[0] = motpos_zero[0] - 300;
        if(act_mot_cur_1>-50)
		motpos_zero[1] = motpos_zero[1] + 300;

       // std::cout << act_mot_cur_0 << '\t' << act_mot_cur_1 << '\t' << motpos_zero[0] << '\t' << motpos_zero[1] << '\n';
        set_motor_positions(motpos_zero[0], motpos_zero[1], false);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
    motpos_zero[0] = motpos_zero[0] + 600;//back up a bit
	motpos_zero[1] = motpos_zero[1] - 600;
    set_motor_positions(motpos_zero[0], motpos_zero[1], false);
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
    get_motor_currents(act_mot_cur_0,act_mot_cur_1,false);

    while(act_mot_cur_0<50 || act_mot_cur_1> -50) {//now go with increments of 100
        get_motor_currents(act_mot_cur_0,act_mot_cur_1,false);

        if(act_mot_cur_0<50)
		motpos_zero[0] = motpos_zero[0] - 100;
        if(act_mot_cur_1>-50)
		motpos_zero[1] = motpos_zero[1] + 100;

       // std::cout << act_mot_cur_0 << '\t' << act_mot_cur_1 << '\t' << motpos_zero[0] << '\t' << motpos_zero[1] << '\n';
        set_motor_positions(motpos_zero[0], motpos_zero[1], false);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

    short int motpos[2];
    //motpos[0] = motpos_zero[0] - 20000;
    //motpos[1] = motpos_zero[1] + 20000;
    //set_motor_positions(motpos[0], motpos[1], false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    get_motor_currents(act_mot_pos_0, act_mot_pos_1, false);
    scaling_factor[0] = abs((act_mot_pos_0 - motpos_zero[0]) / MAX_VAL_CAR_0);
    scaling_factor[1] = abs((act_mot_pos_1 - motpos_zero[1]) / MAX_VAL_CAR_1);
    //set_motor_positions(motpos_zero[0], motpos_zero[1], false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    //motpos_zero[0] = motpos_zero[0] - (0.1138*pow(force_newtons, 3) - 5.204*pow(force_newtons, 2) + 89.22*force_newtons + 0) * scaling_factor[0];
    //motpos_zero[1] = motpos_zero[1] + (0.1138*pow(force_newtons, 3) - 5.204*pow(force_newtons, 2) + 89.22*force_newtons + 0) * scaling_factor[1];
    //set_motor_positions(motpos_zero[0], motpos_zero[1], false);
    std::cout << "Done" << std::endl;
}

int Cuff::io_thread_func() {
	while (poll_io_) {
        spinlock.lock();

        short reference_motor_positions[2];
        short actual_motor_positions[2];
        short actual_motor_currents[2];

        reference_motor_positions[0] = reference_motor_positions_[0];
        reference_motor_positions[1] = reference_motor_positions_[1];

		commSetInputs(&comm_settings_t_, CUFF_ID, reference_motor_positions);
		commGetMeasurements(&comm_settings_t_, CUFF_ID, actual_motor_positions);
		commGetCurrents(&comm_settings_t_, CUFF_ID, actual_motor_currents);

        actual_motor_positions_[0] = actual_motor_positions[0];
        actual_motor_positions_[1] = actual_motor_positions[1];

        actual_motor_currents_[0] = actual_motor_currents[0];
        actual_motor_currents_[1] = actual_motor_currents[1];

        spinlock.unlock();
	}
    return 1;
}

void Cuff::set_motor_positions(short int motor_position_0, short int motor_position_1, bool immediate) {
    if (!immediate)
        spinlock.lock();
    reference_motor_positions_[0] = motor_position_0;
    reference_motor_positions_[1] = motor_position_1;
    if (!immediate)
        spinlock.unlock();
}

void Cuff::get_motor_positions(short int& motor_position_0, short int& motor_position_1, bool immediate) {
    if (!immediate)
        spinlock.lock();
    motor_position_0 = actual_motor_positions_[0];
    motor_position_1 = actual_motor_positions_[1];
    if (!immediate)
        spinlock.unlock();
}

void Cuff::get_motor_currents(short int& motor_current_0, short int& motor_current_1, bool immediate) {
    if (!immediate)
        spinlock.lock();
    motor_current_0 = actual_motor_currents_[0];
    motor_current_1 = actual_motor_currents_[1];
    if (!immediate)
        spinlock.unlock();
}
