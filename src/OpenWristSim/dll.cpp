#pragma once
#include "OpenWristSim.hpp"
#include <MEL/Core.hpp>
#include <MEL/Communications.hpp>
#include <thread>
#include <mutex>
#include <atomic>

#ifdef _WIN32
#define EXPORT extern "C" __declspec(dllexport)
#else
#define EXPORT extern "C"
#endif

OpenWristSim g_sim;
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
            g_sim.set_torques(ms_in_data[0], ms_in_data[1], ms_in_data[2]);
            g_sim.update(t);
            ms_out_data[0] = g_sim.q1;
            ms_out_data[1] = g_sim.q2;
            ms_out_data[2] = g_sim.q3;
            ms_out_data[3] = g_sim.q1d;
            ms_out_data[4] = g_sim.q2d;
            ms_out_data[5] = g_sim.q3d;
            ms_out_data[6] = g_sim.q1dd;
            ms_out_data[7] = g_sim.q2dd;
            ms_out_data[8] = g_sim.q3dd;
            ms_out_data[9] = g_sim.tau1;
            ms_out_data[10] = g_sim.tau2;
            ms_out_data[11] = g_sim.tau3;
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
    g_sim.reset();
    g_stop = false;
    g_thread = std::thread(simulation);
}

EXPORT void set_torques(double tau1, double tau2, double tau3)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_sim.set_torques(tau1, tau2, tau3);
}

EXPORT void set_positions(double q1, double q2, double q3)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_sim.set_positions(q1, q2, q3);
}

EXPORT void set_velocities(double q1d, double q2d, double q3d)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    g_sim.set_velocities(q1d, q2d, q3d);
}

EXPORT void get_positions(double *positions)
{
    std::lock_guard<std::mutex> lock(g_mtx);
    positions[0] = g_sim.q1;
    positions[1] = g_sim.q2;
    positions[2] = g_sim.q3;
}