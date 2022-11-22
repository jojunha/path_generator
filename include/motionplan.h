#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>

#include <ompl/geometric/SimpleSetup.h>
 
#include <ompl/config.h>
#include <iostream>

#include <Eigen/Dense>
#include <time.h>

#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h> 

#define PI 3.14159265358979323846 /* pi */

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace Eigen;
using namespace std;

class CMotionPlan{
public:
    CMotionPlan();
    ~CMotionPlan();

    void setStartState(Vector3d start_pos, Vector3d start_ori);
    void setGoalState(Vector3d start_pos, Vector3d start_ori);
    void solveWithoutSmoothing();
    void solve();

    Eigen::MatrixXd getResultPosMatrix();
    Eigen::MatrixXd getResultOriMatrix();


private:
    ob::StateSpacePtr _space;
    ob::SpaceInformationPtr _si;
    ob::ProblemDefinitionPtr _pdef;
    ob::PlannerPtr _planner;
    ob::PlannerStatus _solved;

    ob::State *_start;
    ob::State *_goal;

    Vector3d _low_bounds;
    Vector3d _high_bounds;

    Vector3d _start_pos;
    Vector3d _start_ori;
    Vector3d _goal_pos;
    Vector3d _goal_ori;

    MatrixXd _result_mat_pos; 
    MatrixXd _result_mat_ori;


    void initialize();

    void constructStateSpace();
    void constructSpaceInformation();
    void createProblemDefinition();
    void setBoundsOfState(Vector3d low_bounds, Vector3d high_bounds);
    void setOptimalPlanner();

    std::size_t _state_count;

};