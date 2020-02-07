#pragma once
#include "OpenWristModel.hpp"
#include <MEL/Core.hpp>
#include <MEL/Communications.hpp>
#include <thread>
#include <mutex>
#include <atomic>

// This is the dll that drives the model in the Unity OpenWristSim visualization
// Unity intefaces via the the C methods below, mostly just to start/stop the simulation, and read joint positions
// End-users inteface through the MelShares or at a higher level, through OpenWristSim.hpp/cpp in their own C++ code
// This was all sort of hastily put together...it could be better!

#define EXPORT extern "C" __declspec(dllexport)

OpenWristModel g_model;
std::thread g_thread;
std::mutex g_mtx;
std::atomic_bool g_stop;

using namespace mel;

void simulation()
{
    MelShare ms_in("openwrist_sim_in");
    MelShare ms_out("openwrist_sim_out");
    std::vector<double> ms_in_data(3, 0);
    std::vector<double> ms_out_data(12, 0);
    ms_in.write_data(ms_in_data);
    ms_out.write_data(ms_out_data);
    Timer timer(hertz(1000), Timer::Hybrid);
    Time t;
    while (!g_stop)
    {
        ms_in_data = ms_in.read_data();
        {
            std::lock_guard<std::mutex> lock(g_mtx);
            g_model.set_torques(ms_in_data[0], ms_in_data[1], ms_in_data[2]);
            g_model.update(t);
            ms_out_data[0] = g_model.q1;
            ms_out_data[1] = g_model.q2;
            ms_out_data[2] = g_model.q3;
            ms_out_data[3] = g_model.q1d;
            ms_out_data[4] = g_model.q2d;
            ms_out_data[5] = g_model.q3d;
            ms_out_data[6] = g_model.q1dd;
            ms_out_data[7] = g_model.q2dd;
            ms_out_data[8] = g_model.q3dd;
            ms_out_data[9] = g_model.tau1;
            ms_out_data[10] = g_model.tau2;
            ms_out_data[11] = g_model.tau3;
        }
        ms_out.write_data(ms_out_data);
        t = timer.wait();
    }
};

EXPORT void stop()
{
    g_stop = true;
    if (g_thread.joinable())
        g_thread.join();
}

EXPORT void start()
{
    stop();
    g_model.reset();
    g_stop = false;
    g_thread = std::thread(simulation);
}

EXPORT void set_torques(double tau1, double tau2, double tau3)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_model.set_torques(tau1, tau2, tau3);
}

EXPORT void set_positions(double q1, double q2, double q3)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_model.set_positions(q1, q2, q3);
}

EXPORT void set_velocities(double q1d, double q2d, double q3d)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_model.set_velocities(q1d, q2d, q3d);
}

EXPORT void get_positions(double *positions)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    positions[0] = g_model.q1;
    positions[1] = g_model.q2;
    positions[2] = g_model.q3;
}