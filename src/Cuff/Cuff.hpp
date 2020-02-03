#pragma once
#include <thread>
#include <MEL/Utility/Spinlock.hpp>
#include <MEL/Core/Device.hpp>
#include <atomic>
#include <MEL/Core/Types.hpp>
#include <MEL/Core/NonCopyable.hpp>
#include "comm_settings.h"
#include <MEL/Daq/Output.hpp>

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

protected:

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

class CuffVoltPaq : public Cuff {
public:
    CuffVoltPaq(std::string name, mel::uint32 comm_port,mel::DigitalOutput::Channel d_out, mel::AnalogOutput::Channel a_out) : 
        Cuff(name, comm_port),
        d_out_(d_out),
        a_out_(a_out)
    { }

private:
    mel::DigitalOutput::Channel d_out_;
    mel::AnalogOutput::Channel a_out_;

    bool on_enable() override{
        d_out_.set_value(mel::High);
        d_out_.update();
        a_out_.set_value(-4);
        a_out_.update();
        return Cuff::on_enable();
    }

    bool on_disable() override{
        auto r = Cuff::on_disable();
        a_out_.set_value(0);
        a_out_.update();
        d_out_.set_value(mel::Low);
        d_out_.update();
        return r;
    }
};
