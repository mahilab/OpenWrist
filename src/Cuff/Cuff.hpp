#pragma once
#include <thread>
#include "qbmove_communications.h"
#include <MEL/Utility/Spinlock.hpp>
#include <MEL/Core/Device.hpp>
#include <atomic>
#include <MEL/Utility/Types.hpp>

class Cuff : public mel::Device{

public:

    Cuff();
    Cuff(std::string name, mel::uint32 comm_port);
    ~Cuff() override;

	bool enable() override;
    bool disable() override;
	void set_motor_positions(short int motor_position_0, short int motor_position_1, bool immediate);
    void get_motor_positions(short int& motor_position_0, short int& motor_position_1, bool immediate);
    void get_motor_currents(short int& motor_current_0, short int& motor_current_1, bool immediate);

	void pretension(int force_newtons, short int* motpos_zero, short int* scaling_factor);

private:

	std::atomic_short reference_motor_positions_[2];
	std::atomic_short actual_motor_positions_[2];
	std::atomic_short actual_motor_currents_[2];

	comm_settings comm_settings_t_;
    mel::Spinlock spinlock;
    std::thread io_thread_;
    int io_thread_func();

    const mel::uint32 comm_port_;

	void open_port();
    bool port_opened_ = false;
    //int port_selection();

	volatile std::atomic_bool poll_io_;

};