#include <apps/challenge/challenge_results.hpp>
#include <apps/challenge/challenge_channels.hpp>
#include <apps/challenge/map_statistics.hpp>
#include <common/timestamp.h>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <cassert>

const int kNoActiveTeam = -1;
const std::string kResultsBackFilename("finished_runs.txt");

////////////////////// TeamResults implementation //////////////////////////////

std::ostream& operator<<(std::ostream& out, const TeamResult& result)
{
    out << result.teamNumber_ << ' ' << result.startTime_ << ' ' << result.finishTime_ << ' ' << result.exploration_
        << ' ' << result.correlation_ << '\n';
    return out;
}


std::istream& operator>>(std::istream& in, TeamResult& result)
{
    in >> result.teamNumber_ >> result.startTime_ >> result.finishTime_ >> result.exploration_ >> result.correlation_;
    return in;
}


TeamResult::TeamResult(int teamNumber)
: teamNumber_(teamNumber)
, startTime_(0)
, finishTime_(0)
, exploration_(0.0)
, correlation_(0.0)
{
    
}


void TeamResult::startRun(void)
{
    assert(startTime_ == 0);
    startTime_ = utime_now();
}


void TeamResult::finishRun(void)
{
    assert(finishTime_ == 0);
    finishTime_ = utime_now();
    assert(finishTime_ > startTime_);
}


void TeamResult::updateScore(const OccupancyGrid& teamMap, const OccupancyGrid& staffMap, const pose_xyt_t& staffPose)
{
    correlation_ = map_correlation(teamMap, staffMap);
    exploration_ = amount_explored(staffMap, staffPose);
    
    std::cout << "Correlation: " << correlation() << " Explored:" << explorationPercent() << '\n';
}


double TeamResult::timeElapsed(void) const
{
    // If haven't started, no time has elapsed
    if(startTime_ == 0)
    {
        return 0.0;
    }
    // If not yet finishd, elapsed time goes from start of the current time
    else if(finishTime_ == 0)
    {
        return (utime_now() - startTime_) / 1000000.0;
    }
    // Otherwise finished, so elapsed time is well-defined
    else
    {
        return (finishTime_ - startTime_) / 1000000.0;
    }
}


double TeamResult::overallScore(void) const
{
    return (explorationPercent() * correlation_) / std::max(1.0, timeElapsed());
}

////////////////////// ChallengeResults implementation /////////////////////////

ChallengeResults::ChallengeResults(void)
: state_(WAITING)
, haveUpdatedResult_(false)
, startedNewRun_(false)
, activeRun_(kNoActiveTeam)
, activeTeam_(kNoActiveTeam)
{
    std::ifstream in(kResultsBackFilename);
    if(in.is_open())
    {
        int numResults = 0;
        in >> numResults;
        
        TeamResult tempResult(kNoActiveTeam);
        for(int n = 0; n < numResults; ++n)
        {
            // Load and validate the result 
            in >> tempResult;
            
            if(tempResult.teamNumber() != kNoActiveTeam)
            {
                completedRuns_.push_back(tempResult);
            }
        }
        
        haveUpdatedResult_ = completedRuns_.size() > 0;
    }
}


void ChallengeResults::subscribeToData(lcm::LCM& lcmInstance)
{
    lcmInstance.subscribe(EXPLORATION_CHANNEL, &ChallengeResults::handleExplorationStatus, this);
    lcmInstance.subscribe(TEAM_MAP_CHANNEL, &ChallengeResults::handleOccupancyGrid, this);
    lcmInstance.subscribe(STAFF_MAP_CHANNEL, &ChallengeResults::handleOccupancyGrid, this);
    lcmInstance.subscribe(STAFF_POSE_CHANNEL, &ChallengeResults::handlePose, this);
}


bool ChallengeResults::haveUpdatedResults(void)
{
    std::lock_guard<std::mutex> lock(resultsLock_);
    
    bool updated = haveUpdatedResult_;
    haveUpdatedResult_ = false;
    return updated;
}


bool ChallengeResults::haveStartedNewRun(void)
{
    std::lock_guard<std::mutex> lock(resultsLock_);
    
    bool haveNew = startedNewRun_;
    startedNewRun_ = false;
    return haveNew;
}


ChallengeResults::const_iterator ChallengeResults::begin(void) const
{
    std::lock_guard<std::mutex> lock(resultsLock_);
    
    currentResults_.clear();
    currentResults_ = completedRuns_;
    
    if(activeRun_.teamNumber() != kNoActiveTeam)
    {
        currentResults_.push_back(activeRun_);
    }
    
    std::sort(currentResults_.begin(), currentResults_.end(), [](const TeamResult& lhs, const TeamResult& rhs) {
        return lhs.overallScore() > rhs.overallScore();
    });
    
    return currentResults_.begin();
}


ChallengeResults::const_iterator ChallengeResults::end(void) const
{
    return currentResults_.end();
}


void ChallengeResults::handleOccupancyGrid(const lcm::ReceiveBuffer* rbuf, 
                                           const std::string& channel, 
                                           const occupancy_grid_t* map)
{
    std::lock_guard<std::mutex> lock(resultsLock_);
    
    if(channel == STAFF_MAP_CHANNEL)
    {
        staffMap_.fromLCM(*map);
    }
    else if(channel == TEAM_MAP_CHANNEL)
    {
        teamMap_.fromLCM(*map);
    }
    
    // If maps have been received, then the score can be updated
    if((staffMap_.widthInCells() > 0) && (teamMap_.widthInCells() > 0) && havePose_ && (state_ == EXPLORING))
    {
        activeRun_.updateScore(teamMap_, staffMap_, staffPose_);
        haveUpdatedResult_ = true;
    }
}


void ChallengeResults::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{
    staffPose_ = *pose;
    havePose_ = true;
}


void ChallengeResults::handleExplorationStatus(const lcm::ReceiveBuffer* rbuf, 
                                               const std::string& channel, 
                                               const exploration_status_t* exploration)
{
    std::lock_guard<std::mutex> lock(resultsLock_);
    
    switch(state_)
    {
    case WAITING:
        if(exploration->state == exploration_status_t::STATE_EXPLORING_MAP)
        {
            initializeNewRun(exploration->team_number);
        }
        else if(exploration->state == exploration_status_t::STATE_COMPLETED_EXPLORATION)
        {
            std::cerr << "WARNING: ChallengeResults: Received a COMPLETE message from " << exploration->team_number
                << " while in the WAITING state. Their run never began.\n";
        }
        break;
        
    case EXPLORING:
        if(exploration->status == exploration_status_t::STATE_EXPLORING_MAP)
        {
            finishRun();
        }
        break;
        
    default:
        std::cerr << "ERROR: ChallengeResults: In an incorrect state:" << state_ << '\n';
    }
}


void ChallengeResults::initializeNewRun(int teamNumber)
{
    state_ = EXPLORING;
    startedNewRun_ = true;
    
    activeRun_ = TeamResult(teamNumber);
    activeRun_.startRun();
}


void ChallengeResults::finishRun(void)
{
    activeRun_.finishRun();
    completedRuns_.push_back(activeRun_);
    
    // Whenever a run finishes, save everything to disk to ensure that they aren't lost if the program crashes
    // Format: # results, then one result per line
    std::ofstream out(kResultsBackFilename);
    out << completedRuns_.size() << '\n';
    if(out.is_open())
    {
        std::copy(completedRuns_.begin(), completedRuns_.end(), std::ostream_iterator<TeamResult>(out));
    }
    
    state_ = WAITING;
}
