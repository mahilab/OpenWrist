// #define CARNOT_NO_CONSOLE
#include <carnot>
#include <thread>
#include <MEL/Core/Timer.hpp>
#include <MEL/Core/Console.hpp>
#include <mutex>
#include <queue>
#include <vector>
#include <MEL/Utility/Windows/ExternalApp.hpp>

#include <windows.h>

using namespace carnot;
using namespace mel;

std::vector<double> pos_cur(3,0);

std::queue<double> pos_0;
std::queue<double> pos_1;
std::queue<double> pos_2;
const int samples = 1000;

ExternalApp jedi_unity("jedi_unity","C:/Unity/AJLS/Build/AJLS.exe");
ExternalApp octagon_unity("octagon_unity","C:/Unity/OctagonSqueeze/Build/Evan Prezents.exe");
ExternalApp furuta_unity("furuta_unity","C:/Unity/FurutaPendulum/build/AJLS.exe");
ExternalApp airplane_unity("airplane_unity","C:/Unity/AerialTargets/Build/AerialTargets.exe");

ExternalApp demos("demos","demos.exe");



class DemoGui : public GameObject {

    void start() {

        for(int i=0;i<samples;i++){
            pos_0.push(0);
            pos_1.push(0);
            pos_2.push(0);
        }
        controlThread = std::thread(&DemoGui::controlThreadFunc, this);
        //Debug::show(true);

    }

    void update() override {
        //STARTUP WINDOW
        ImGui::SetNextWindowPos(Vector2f(5,5));
        ImGui::SetNextWindowSize(Vector2f(240,300));
        ImGui::Begin("Startup", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
        
            ImGui::Text("Startup Procedures");
            ImGui::Separator();

            ImGui::Text("Hello, World");
            if (ImGui::Button("Debug Mode (No Power)")){
                carnot::print("debug");

            }
            if (ImGui::Button("Calibration")){
                carnot::print("calibrate");
            }
        ImGui::End(); 

        //GRAPH WINDOW
        ImGui::SetNextWindowPos(Vector2f(250,5));
        ImGui::SetNextWindowSize(Vector2f(240,300));
        ImGui::Begin("Graphs", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
            ImGui::Text("Position Graphs");
            ImGui::Separator();
        ImGui::End();

        //APPS WINDOW
        ImGui::SetNextWindowPos(Vector2f(5,310));
        ImGui::SetNextWindowSize(Vector2f(485,185));
        ImGui::Begin("Apps", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
            ImGui::Text("OpenWrist Demo Launchers");
            ImGui::Separator();

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
            if (ImGui::Button("Furuta Pendulum Balancing")){
                carnot::print("haptictraining");
                //ShellExecute(NULL,"open","demos.exe","-t",NULL,SW_SHOWDEFAULT);
                furuta_unity.launch();
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
                
        ImGui::End();
        
        {    
        std::lock_guard<std::mutex> lock(m_mutex);
            data++;
        }    
      
    }

    void controlThreadFunc() { // treat this as main
        mel::Timer timer(mel::hertz(1000));
        while (!stop) {
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                //carnot::print(data++);
            }
            timer.wait();
        }
    }

    std::thread controlThread;
    std::mutex m_mutex;
    ctrl_bool stop = false;

    int data = 0;
};

int main(int argc, char const *argv[])
{
    Engine::init(500,500,"OpenWrist Demo Launcher");
    Engine::makeRoot<DemoGui>();
    Engine::run();    
    return 0;
}
