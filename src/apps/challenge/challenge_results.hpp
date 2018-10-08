#ifndef APPS_SLAM_CHALLENGE_RESULTS_HPP
#define APPS_SLAM_CHALLENGE_RESULTS_HPP

#include <lcmtypes/exploration_status_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <vector>

/**
* TeamResult
*/
class TeamResult
{
public:
    
    /**
    * Constructor for TeamResult.
    * 
    * \param    teamNumber          Number identifying the team
    */
    TeamResult(int teamNumber);
    
    /**
    * startRun starts the clock counting for a team's result.
    */
    void startRun(void);
    
    /**
    * finishRun stops the current run.
    */
    void finishRun(void);
    
    /**
    * updateScore updates the scoring information based on map being built by the team and the map being built by the
    * staff code.
    * 
    * \param    teamMap         Map being built by the team's code
    * \param    staffMap        Map being built by the staff's code
    * \param    staffPose       Pose of the robot in the current grid
    */
    void updateScore(const OccupancyGrid& teamMap, const OccupancyGrid& staffMap, const pose_xyt_t& staffPose);
    
    // Properties of the current result
    int teamNumber(void) const { return teamNumber_; }
    bool isFinished(void) const { return finishTime_ > startTime_; }
    double timeElapsed(void) const;
    double explorationPercent(void) const { return exploration_ * 100.0; }
    double correlation(void) const { return correlation_; }
    double overallScore(void) const;
    
    // Format: team start finish exploration correlation
    friend std::ostream& operator<<(std::ostream& out, const TeamResult& result);
    friend std::istream& operator>>(std::istream& in, TeamResult& result);
    
private:
    
    int teamNumber_;
    int64_t startTime_;
    int64_t finishTime_;
    double exploration_;
    double correlation_;
};


/**
* ChallengeResults 
*/
class ChallengeResults
{
public:
    
    typedef std::vector<TeamResult>::const_iterator const_iterator;
    
    /**
    * Constructor for ChallengeResults.
    */
    ChallengeResults(void);
    
    /**
    * subscribeToData subscribes the ChallengeResults instance to the appropriate channels it needs for calculating
    * the scores for individual teams.
    * 
    * \param    lcmInstance         LCM instance used by the process for the challenge results
    */
    void subscribeToData(lcm::LCM& lcmInstance);
    
    /**
    * haveUpdatedResults checks if the results have been updated since the last time the method was called. Consecutive
    * calls to haveUpdateResults will always return false on the second call.
    * 
    * \return   True if the scores were updated since the last call to haveUpdatedResults.
    */
    bool haveUpdatedResults(void);
    
    /**
    * haveStartedNewRun checks if a new run has begun since the last time this method was called. Consecutive
    * calls to haveStartedNewRun will always return false on the second call.
    * 
    * \return   True if a new run started since the last call to this method.
    */
    bool haveStartedNewRun(void);
    
    // Iterate through the challenge results -- in sorted order!
    // WARNING: begin invalidates any previous iterators returned by begin or end.
    const_iterator begin(void) const;
    const_iterator end(void) const;
        
    // Handlers for the maps being judged
    void handleOccupancyGrid(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const occupancy_grid_t* map);
    void handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose);
    void handleExplorationStatus(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const exploration_status_t* exploration);
    
private:
    
    // Possible states for the current challenge
    enum RunState
    {
        WAITING,
        EXPLORING,
    };
    
    RunState state_;
    bool haveUpdatedResult_;
    bool startedNewRun_;
    TeamResult activeRun_;                              // run currently executing
    std::vector<TeamResult> completedRuns_;             // once completed, activeRun goes here
    mutable std::vector<TeamResult> currentResults_;    // completedRuns + activeRun
    int activeTeam_;
    
    OccupancyGrid teamMap_;
    OccupancyGrid staffMap_;
    pose_xyt_t staffPose_;
    bool havePose_;
    
    mutable std::mutex resultsLock_;
    
    void initializeNewRun(int teamNumber);
    void finishRun(void);
};

#endif // APPS_SLAM_CHALLENGE_RESULTS_HPP
