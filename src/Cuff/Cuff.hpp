#pragma once
#include <thread>
#include <MEL/Utility/Spinlock.hpp>
#include <MEL/Core/Device.hpp>
#include <atomic>
#include <MEL/Core/Types.hpp>
#include <MEL/Core/NonCopyable.hpp>
#include "comm_settings.h"

class Cuff : public mel::NonCopyable, public mel::Device {

public:

    Cuff();
    Cuff(std::string name, mel::uint32 comm_port);
    ~Cuff() override;


	void set_motor_positions(short int motor_position_0, short int motor_position_1, bool immediate);
    void get_motor_positions(short int& motor_position_0, short int& motor_position_1, bool immediate);
    void get_motor_currents(short int& motor_current_0, short int& motor_current_1, bool immediate);

	void oldpretension(int force_newtons, short int* motpos_zero, short int* scaling_factor);
    void pretension(short int* motpos_zero);

private:

    bool on_enable() override;
    bool on_disable() override;

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
