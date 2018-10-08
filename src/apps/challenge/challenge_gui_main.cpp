#include <apps/challenge/challenge_gui.hpp>
#include <apps/challenge/challenge_results.hpp>
#include <thread>

int main(int argc, char** argv)
{
    lcm::LCM lcmInstance;
    
    ChallengeResults results;
    results.subscribeToData(lcmInstance);
    
    SLAMChallengeGUI window(argc, argv, 1700, 1000, 15);
    window.subscribeToData(lcmInstance);

    std::thread lcmThread([&]() {
        while(true) {
            lcmInstance.handleTimeout(100);
            
            if(results.haveUpdatedResults())
            {
                window.updateResults(results);
            }
            
            if(results.haveStartedNewRun())
            {
                window.clearRunData();
            }
        }
    });

    window.run();

    return 0;
}
