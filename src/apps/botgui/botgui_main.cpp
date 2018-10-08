#include <apps/botgui/botgui.hpp>
#include <thread>

int main(int argc, char** argv)
{
    lcm::LCM lcmInstance;
    BotGui gui(&lcmInstance, argc, argv, 1000, 800, 15);
    
    std::thread lcmThread([&]() {
        while(true) {
            lcmInstance.handleTimeout(100);
        }
    });
    
    gui.run();
    
    return 0;
}
