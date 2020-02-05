#include "OpenWristSim.hpp"
#include <MEL/Core.hpp>

using namespace mel;

int main(int argc, char const *argv[])
{
    OpenWristSim sim;
    Clock clk;
    for (int i = 0; i < 1000000; ++i) 
        sim.update(clk.get_elapsed_time());
    print(1000000 / clk.get_elapsed_time().as_seconds(), "Hz");    
    return 0;
}
