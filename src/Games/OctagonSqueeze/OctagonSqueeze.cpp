#include  "OctagonSqueeze.hpp"

OctagonSqueeze::OctagonSqueeze(Q8Usb& ow_daq, OpenWrist& ow, ctrl_bool& stop_flag) :
    StateMachine(4),
    timer_(hertz(1000)),
    ow_daq_(ow_daq),
    ow_(ow),
    stop_flag_(stop_flag),
    ms_player_state_("player_state"),
    ms_force_torque_("force_torque")
{

}
