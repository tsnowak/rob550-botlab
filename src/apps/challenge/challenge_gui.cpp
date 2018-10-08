#include <apps/challenge/challenge_gui.hpp>
#include <apps/challenge/challenge_channels.hpp>
#include <apps/challenge/challenge_results.hpp>
#include <vx/gtk/vx_gtk_display_source.h>
#include <vx/vx_colors.h>
#include <iterator>
#include <glib.h>
#include <unistd.h>

// Definition of the columns used for displaying the current results
enum
{
    TEAM_COLUMN,
    TIME_COLUMN,
    CORRELATION_COLUMN,
    EXPLORATION_COLUMN,
    OVERALL_COLUMN,
    ACTIVE_COLUMN,          // active result has background set so it is easier to see
    ACTIVE_COLOR_COLUMN,    // all use same color and it'll switch between the one to be shown
    NUM_COLUMNS,
};


SLAMChallengeGUI::SLAMChallengeGUI(int argc, 
                                   char** argv, 
                                   int widthInPixels, 
                                   int heightInPixels, 
                                   int framesPerSecond)
: VxGtkWindowBase(argc, argv, widthInPixels, heightInPixels, framesPerSecond)
, resultsData_(nullptr)
, numStoredResults_(0)
, resultsView_(nullptr)
, teamVis_("Team", vx_blue)
, staffVis_("Staff", vx_red)
{
}


void SLAMChallengeGUI::updateResults(const ChallengeResults& results)
{
    std::lock_guard<std::mutex> lock(vxLock_);
    gdk_threads_enter();
    
    // Can't fill in results until the GUI has been initialized
    if(!resultsData_ || !resultsView_)
    {
        return;
    }
    
    auto resultBegin = results.begin();
    auto resultEnd = results.end();
    
    // Add enough rows to the store for every result
    for(int n = numStoredResults_; n < std::distance(resultBegin, resultEnd); ++n)
    {
        GtkTreeIter appended;
        gtk_list_store_append(resultsData_, &appended);
        ++numStoredResults_;
    }
    
    std::cout << "Num results:" << std::distance(resultBegin, resultEnd) << '\n';
    
    // Go through and write the values for each row in the column store
    GtkTreeIter storeIter;
    gtk_tree_model_get_iter_first(GTK_TREE_MODEL(resultsData_), &storeIter);
    for(int n = 0; n < std::distance(resultBegin, resultEnd); ++n)
    {
        const TeamResult& result = *(resultBegin + n);
        
        assert(gtk_list_store_iter_is_valid(resultsData_, &storeIter));
        gtk_list_store_set(resultsData_, &storeIter,
                           TEAM_COLUMN, result.teamNumber(),
                           TIME_COLUMN, result.timeElapsed(),
                           CORRELATION_COLUMN, result.correlation(),
                           EXPLORATION_COLUMN, result.explorationPercent(),
                           OVERALL_COLUMN, result.overallScore(),
                           ACTIVE_COLUMN, !result.isFinished(),
                           ACTIVE_COLOR_COLUMN, "#1E90FF",
                           -1);
        gtk_tree_model_iter_next(GTK_TREE_MODEL(resultsData_), &storeIter);
    }
    
    gtk_tree_view_set_model(GTK_TREE_VIEW(resultsView_), GTK_TREE_MODEL(resultsData_));
    gdk_threads_leave();
}


void SLAMChallengeGUI::clearRunData(void)
{
    std::lock_guard<std::mutex> lock(vxLock_);
    
    staffVis_.reset();
    teamVis_.reset();
}


int SLAMChallengeGUI::onMouseEvent(vx_layer_t* layer, 
                                   vx_camera_pos_t* cameraPosition, 
                                   vx_mouse_event_t* event, 
                                   Point<float> worldPoint)
{
    // By default, don't consume mouse events
    return 0;
}


int SLAMChallengeGUI::onKeyEvent(vx_layer_t* layer, vx_key_event_t* event)
{
    // By default, don't consume keyboard events
    return 0;
}


void SLAMChallengeGUI::onDisplayStart(vx_display_t * display)
{
    staffVis_.initializeDisplay(display, true);
    teamVis_.initializeDisplay(display, false);
}


void SLAMChallengeGUI::onDisplayFinish(vx_display_t * display)
{
    // Nothing to do when finishing
}


void SLAMChallengeGUI::render(void)
{
    staffVis_.render();
    teamVis_.render();
}


void SLAMChallengeGUI::subscribeToData(lcm::LCM& lcmInstance)
{
    lcmInstance.subscribe(STAFF_MAP_CHANNEL, &GridVisualization::handleOccupancyGrid, &teamVis_);
    lcmInstance.subscribe(TEAM_MAP_CHANNEL, &GridVisualization::handleOccupancyGrid, &staffVis_);

    lcmInstance.subscribe(STAFF_POSE_CHANNEL, &GridVisualization::handlePose, &teamVis_);
    lcmInstance.subscribe(TEAM_POSE_CHANNEL, &GridVisualization::handlePose, &staffVis_);
}


void SLAMChallengeGUI::createGuiLayout(GtkWidget* window, GtkWidget* vxCanvas)
{
    std::lock_guard<std::mutex> lock(vxLock_);

    // Create a horizontal pane. Results on the right. Vx and live score on the left.
    GtkWidget* mainPane = gtk_hpaned_new();
    gtk_container_add(GTK_CONTAINER(window), mainPane);
    
    // Create the Vx and live scoring pane as a vertical box
    GtkWidget* vxPaneBox = gtk_vbox_new(FALSE, 5);
    gtk_paned_pack1(GTK_PANED(mainPane), vxPaneBox, TRUE, FALSE);
    
    gtk_box_pack_start(GTK_BOX (vxPaneBox), vxCanvas, 1, 1, 0);
    gtk_widget_show(vxCanvas);    // XXX Show all causes errors!
    
    // Use a tree view for the results
    resultsData_ = gtk_list_store_new(NUM_COLUMNS, 
                                    G_TYPE_INT, 
                                    G_TYPE_DOUBLE, 
                                    G_TYPE_DOUBLE, 
                                    G_TYPE_DOUBLE, 
                                    G_TYPE_DOUBLE,
                                    G_TYPE_BOOLEAN,
                                    G_TYPE_STRING);
    resultsView_ = gtk_tree_view_new_with_model(GTK_TREE_MODEL(resultsData_));
    
    // Set the column titles
    GtkCellRenderer* renderer = gtk_cell_renderer_text_new();
    GtkTreeViewColumn* column = gtk_tree_view_column_new_with_attributes("Team",
                                                                        renderer,
                                                                        "text", TEAM_COLUMN,
                                                                            "cell-background-set", ACTIVE_COLUMN,
                                                                            "cell-background", ACTIVE_COLOR_COLUMN,
                                                                        NULL);
    renderer = gtk_cell_renderer_text_new();
    gtk_tree_view_append_column(GTK_TREE_VIEW(resultsView_), column);
    column = gtk_tree_view_column_new_with_attributes("Time",
                                                    renderer,
                                                    "text", TIME_COLUMN,
                                                    "cell-background-set", ACTIVE_COLUMN,
                                                    "cell-background", ACTIVE_COLOR_COLUMN,
                                                    NULL);
    renderer = gtk_cell_renderer_text_new();
    gtk_tree_view_append_column(GTK_TREE_VIEW(resultsView_), column);
    column = gtk_tree_view_column_new_with_attributes("Exploration",
                                                    renderer,
                                                    "text", EXPLORATION_COLUMN,
                                                    "cell-background-set", ACTIVE_COLUMN,
                                                    "cell-background", ACTIVE_COLOR_COLUMN,
                                                    NULL);
    renderer = gtk_cell_renderer_text_new();
    gtk_tree_view_append_column(GTK_TREE_VIEW(resultsView_), column);
    column = gtk_tree_view_column_new_with_attributes("Correlation",
                                                    renderer,
                                                    "text", CORRELATION_COLUMN,
                                                    "cell-background-set", ACTIVE_COLUMN,
                                                    "cell-background", ACTIVE_COLOR_COLUMN,
                                                    NULL);
    renderer = gtk_cell_renderer_text_new();
    gtk_tree_view_append_column(GTK_TREE_VIEW(resultsView_), column);
    column = gtk_tree_view_column_new_with_attributes("Overall",
                                                    renderer,
                                                    "text", OVERALL_COLUMN,
                                                    "cell-background-set", ACTIVE_COLUMN,
                                                    "cell-background", ACTIVE_COLOR_COLUMN,
                                                    NULL);
    renderer = gtk_cell_renderer_text_new();
    gtk_tree_view_append_column(GTK_TREE_VIEW(resultsView_), column);
    gtk_paned_pack2(GTK_PANED(mainPane), resultsView_, FALSE, FALSE);
    
    gtk_widget_show(vxCanvas);    // XXX Show all causes errors!
    gtk_widget_show_all(window);
}
