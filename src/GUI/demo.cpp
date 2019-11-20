// #define CARNOT_NO_CONSOLE
#include <carnot>
#include <thread>
#include <MEL/Core/Timer.hpp>
#include <MEL/Core/Console.hpp>
#include <mutex>
#include <queue>
#include <vector>
#include <MEL/Utility/Windows/ExternalApp.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Devices/VoltPaqX4.hpp>
#include <MEL/Daq/Quanser/Q8Usb.hpp>

#include "OpenWrist.hpp"


#include <windows.h>

using namespace carnot;
using namespace mel;

std::vector<double> pos_cur(3,0);

std::string UnityLocation = "C:/Unity/";

const int samples = 1000;
float pos_0 [samples];
float pos_1 [samples];
float pos_2 [samples];


bool enable_buttons = true;
bool queue_calibrate = false;

ExternalApp jedi_unity("jedi_unity",        UnityLocation + "AJLS/Build/AJLS.exe");
ExternalApp octagon_unity("octagon_unity",  UnityLocation + "OctagonSqueeze/Build/Evan Prezents.exe");
ExternalApp furuta_unity("furuta_unity",    UnityLocation + "FurutaPendulum/Build/FurutaPendulum.exe");
ExternalApp airplane_unity("airplane_unity",UnityLocation + "AerialTargets/Build/AerialTargets.exe");

ExternalApp demos("demos","demos.exe");
ExternalApp haptictraining("haptictraining","haptic_training.exe");



class DemoGui : public GameObject {

    void start() {

        controlThread = std::thread(&DemoGui::controlThreadFunc, this);
        //Debug::show(true);

    }

    void update() override {
        //STARTUP WINDOW
        ImGui::SetNextWindowPos(Vector2f(5,5));
        ImGui::SetNextWindowSize(Vector2f(240,300));
        ImGui::Begin("Startup", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
        
            ImGui::Text("HOW TO START");
            ImGui::Separator();
            ImGui::Text("1) Make sure Quanser Q8 is on");
            ImGui::Text("   and VoltPAQ is off");
            ImGui::Text("2) Move each joint of the ");
            ImGui::Text("   OpenWrist and check the");
            ImGui::Text("   position graphs");
            ImGui::Text("3) Turn on VoltPAQ");
            ImGui::Text("4) Run 'Calibration' script");
            ImGui::Text("5) Run any program or demo!");
            ImGui::Separator();

            if(enable_buttons){
                ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0/7.0f, 0.6f, 0.6f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(0/7.0f, 0.7f, 0.7f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(0/7.0f, 0.8f, 0.8f));
                ImGui::PushItemWidth(0.5);
                
                ImGui::Text("");
                if (ImGui::Button("Calibration")){
                    carnot::print("Calibrate");
                    enable_buttons = false;
                    queue_calibrate = true;
                                      
                }
                ImGui::Text("");
                if (ImGui::Button("Output to Melshare 'ow_state'")){
                    carnot::print("debug");
                    demos.launch("-d");
                    stop = true;
                    destroy();
                }
                ImGui::PopStyleColor(3);
                ImGui::PopItemWidth();
            }
            else{//enable buttons
                ImGui::Button("Output to Melshare 'ow_state'");
                ImGui::Button("Calibration");
            }
            
            
            
            
        ImGui::End(); 

        //GRAPH WINDOW
        ImGui::SetNextWindowPos(Vector2f(250,5));
        ImGui::SetNextWindowSize(Vector2f(240,300));
        ImGui::Begin("Graphs", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
            ImGui::Text("Position Graphs");
            ImGui::Separator();

            {    
                std::lock_guard<std::mutex> lock(m_mutex);
                //carnot::print(pos_1[samples]);
                ImGui::PlotLines("",pos_0,IM_ARRAYSIZE(pos_0),0,"Joint 0 Position",-1.2f,1.2f,ImVec2(240,80));
                ImGui::PlotLines("",pos_1,IM_ARRAYSIZE(pos_1),0,"Joint 1 Position",-1.2f,1.2f,ImVec2(240,80));
                ImGui::PlotLines("",pos_2,IM_ARRAYSIZE(pos_2),0,"Joint 2 Position",-1.2f,1.2f,ImVec2(240,80));
                
            }  
        ImGui::End();

        //APPS WINDOW
        ImGui::SetNextWindowPos(Vector2f(5,310));
        ImGui::SetNextWindowSize(Vector2f(485,185));
        ImGui::Begin("Apps", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
            ImGui::Text("OpenWrist Demo Launchers");
            ImGui::Separator();

            if(enable_buttons){

                ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(4/7.0f, 0.6f, 0.6f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(4/7.0f, 0.7f, 0.7f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(4/7.0f, 0.8f, 0.8f));

                if (ImGui::Button("Transparency Mode")){
                    carnot::print("transparency");
                    demos.launch("-t");
                    stop=true;
                    destroy();              
                }
                if (ImGui::Button("A Jedi's Last Stand")){
                    carnot::print("jedi");
                    demos.launch("-j");
                    jedi_unity.launch();
                    stop=true;
                    destroy();
                }
                if (ImGui::Button("Octagon Squeeze")){
                    carnot::print("octagon");
                    demos.launch("-o");
                    octagon_unity.launch();
                    stop=true;
                    destroy();
                }
                
                if (ImGui::Button("Aerial Target Flying")){
                    carnot::print("airplane");
                    demos.launch("-a");
                    airplane_unity.launch();
                    stop=true;
                    destroy();
                }
                if (ImGui::Button("Furuta Pendulum (No Cuff)")){
                    carnot::print("haptictraining");
                    haptictraining.launch("-s 1 -c 1 -t T1-1");
                    furuta_unity.launch();
                    stop=true;
                    destroy();
                }
                if (ImGui::Button("Furuta Pendulum (Cuff)")){
                    carnot::print("haptictraining");
                    haptictraining.launch("-s 1 -c 2 -t T1-1");
                    furuta_unity.launch();
                    stop=true;
                    destroy();
                }
                ImGui::PopStyleColor(3);
            }
            else{
                ImGui::Button("Transparency Mode");
                ImGui::Button("A Jedi's Last Stand");
                ImGui::Button("Octagon Squeeze");
                ImGui::Button("Furuta Pendulum Balancing");
                ImGui::Button("Aerial Target Flying");
            }

  
      
        ImGui::End();
        
          
      
    }

    void controlThreadFunc() { // treat this as main
        //OPENWRIST SETUP
        QuanserOptions qoptions;
        //qoptions.set_update_rate(QuanserOptions::UpdateRate::Fast);
        qoptions.set_analog_output_mode(0, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
        qoptions.set_analog_output_mode(1, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
        qoptions.set_analog_output_mode(2, QuanserOptions::AoMode::CurrentMode1, 0, 2.0, 20.0, 0, -1, 0, 1000);
        qoptions.set_analog_output_mode(3, QuanserOptions::AoMode::VoltageMode,0,0,0,0,0,0,0);
        Q8Usb q8(qoptions);

        if (!q8.open()) {
            LOG(Fatal) << "Unable to open Q8-USB. Aborting OpenWrist demo application.";
        }

        VoltPaqX4 vpx4(q8.DO[{ 0, 1, 2 }], q8.AO[{ 0, 1, 2 }], q8.DI[{0, 1, 2}], q8.AI[{ 0, 1, 2 }]);

        // create OpenWrist and bind Q8 channels to it
        OwConfiguration config(q8, q8.watchdog, q8.encoder[{0, 1, 2}], vpx4.amplifiers);
        OpenWrist ow(config);

        q8.enable();
        ow.enable();

        mel::Timer timer(mel::hertz(1000));
        while (!stop) {
            {
                q8.update_input();
                if(queue_calibrate){
                    ow.calibrate(stop);
                    queue_calibrate = false;
                    enable_buttons = true;
                    q8.enable();
                    ow.enable();
                }

                {
                std::lock_guard<std::mutex> lock(m_mutex);
                
                for(int i=0;i<samples-1;i++){//shift all elements 1 left
                    pos_0[i] = pos_0[i+1];
                    pos_1[i] = pos_1[i+1];
                    pos_2[i] = pos_2[i+1];
                }
                pos_0[samples-1] = ow[0].get_position();//append current data at end
                pos_1[samples-1] = ow[1].get_position();  
                pos_2[samples-1] = ow[2].get_position();                  
                }
            }
            timer.wait();
        }
        ow.disable();
        q8.disable();

    }

    std::thread controlThread;
    std::mutex m_mutex;
    ctrl_bool stop = false;

    int data = 0;
};

int main(int argc, char const *argv[])
{
    

    //IMGUI SETUP
    Engine::init(500,500,"OpenWrist Demo Launcher");
    Engine::makeRoot<DemoGui>();
    Engine::run();    
    return 0;
}
