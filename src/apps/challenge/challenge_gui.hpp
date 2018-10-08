#ifndef APPS_SLAM_CHALLENGE_GUI_HPP
#define APPS_SLAM_CHALLENGE_GUI_HPP

#include <apps/challenge/grid_visualization.hpp>
#include <apps/utils/vx_gtk_window_base.hpp>
#include <vx/vx_display.h>
#include <gtk/gtk.h>
#include <lcm/lcm-cpp.hpp>
#include <mutex>


class ChallengeResults;

/**
* SLAMChallengeGUI
*/
class SLAMChallengeGUI : public VxGtkWindowBase
{
public:
    
    /**
    * Constructor for SLAMChallengeGUI.
    *
    * \param    argc                    Count of command-line arguments for the program
    * \param    argv                    Command-line arguments for the program
    * \param    widthInPixels           Initial width of the GTK window to create
    * \param    heightInPixels          Initial height of the GTK window to create
    * \param    framesPerSecond         Number of frames per second to render visualizations
    *
    * \pre widthInPixels > 0
    * \pre heightInPixels > 0
    * \pre framesPerSecond > 0
    */
    SLAMChallengeGUI(int argc, char** argv, int widthInPixels, int heightInPixels, int framesPerSecond);
    
    /**
    * updateResults updates the displayed results.
    */
    void updateResults(const ChallengeResults& results);
    
    /**
    * clearRunData clears all data associated with the current run. This method should be used when a new run begins to
    * ensure that the previous pose trace doesn't get drawn again.
    */
    void clearRunData(void);
    
    // VxGtkWindowBase interface -- GUI event handling
    int onMouseEvent(vx_layer_t* layer, 
                     vx_camera_pos_t* cameraPosition, 
                     vx_mouse_event_t* event, 
                     Point<float> worldPoint);// override;
    int onKeyEvent(vx_layer_t* layer, vx_key_event_t* event);// override;
    void onDisplayStart(vx_display_t * display);// override;
    void onDisplayFinish(vx_display_t * display);// override;
    void render(void);// override;

    // LCM event handling
    void subscribeToData(lcm::LCM& lcmInstance);

private:
    
    GtkListStore* resultsData_;
    int numStoredResults_;      // can't find an easy way to get this directly from the list store
    
    GtkWidget* resultsView_;
    
    GridVisualization teamVis_;
    GridVisualization staffVis_;

    std::mutex vxLock_;
    
    // VxGtkWindowBase interface -- GUI construction
    void createGuiLayout(GtkWidget* window, GtkWidget* vxCanvas);
};

#endif // APPS_SLAM_CHALLENGE_GUI_HPP
