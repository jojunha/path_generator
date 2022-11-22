#include "motionplan.h"

CMotionPlan::CMotionPlan()
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    initialize();
}
CMotionPlan::~CMotionPlan()
{

}

static bool isStateValid(const ob::State *state) 
{
    // cast the abstract state type to the type we expect
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
 
    // extract the first component of the state and cast it to what we expect
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
 
    // extract the second component of the state and cast it to what we expect
    const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
 
    // check validity of state defined by pos & rot

    // if((pos->values[0] < 2.5) && (pos->values[0] > -2.5) &&(pos->values[1] < 2.5) && (pos->values[1] > -2.5) 
    // && (pos->values[2] < 2.5) && (pos->values[2] > -2.5)){
    //     return false;
    // }

    // if((pos->values[2] > 0.01) || (pos->values[2] < -0.01)){
    //     return false;
    // }

    // Vector3d ori;
    // tf::Quaternion q(rot->x, rot->y, rot->z, rot->w);
    // tf::Matrix3x3 m(q);
    // m.getRPY(ori(0), ori(1), ori(2)); // get Roll, Pitch, Yaw

    // if((ori(0) != 0) && (ori(1) != 0) && (ori(2) != 0)){
    //     return false;
    // }

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings

    return (const void*)rot != (const void*)pos;
}

void CMotionPlan::solve()
{
    constructStateSpace();
    constructSpaceInformation();
    createProblemDefinition();
    setOptimalPlanner();
    
    clock_t start = clock();
    _solved = _planner->ob::Planner::solve(1.0);
    clock_t end = clock();
    
    // check time to solve
    std::cout << double(end-start) / CLOCKS_PER_SEC << std::endl;

    if (_solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        og::PathSimplifier* pathBSpline = new og::PathSimplifier(_si);
		og::PathGeometric* _path_smooth = _pdef->getSolutionPath()->as<og::PathGeometric>();
        
        std::cout << "before :" << _path_smooth->getStateCount() << std::endl;
        // smoothBSpline(PathGeometric &path, unsigned int maxSteps = 5, double minChange = std::numeric_limits<double>::epsilon())
		pathBSpline->smoothBSpline(*_path_smooth,4);
        std::cout << "after :" << _path_smooth->getStateCount() << std::endl;

        Vector3d result_vec;
		Vector3d result_ori;

        _state_count = _path_smooth->getStateCount();

        _result_mat_pos.resize(3, _state_count);
		_result_mat_ori.resize(3, _state_count);
		_result_mat_pos.setZero(3, _state_count);
		_result_mat_ori.setZero(3, _state_count);
		
        std::cout << "Found solution:" << std::endl;

		for (std::size_t path_idx = 0; path_idx < _state_count; path_idx++)
		{
			const ob::SE3StateSpace::StateType *se3state = _path_smooth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

			result_vec(0) = pos->values[0];
			result_vec(1) = pos->values[1];
			result_vec(2) = pos->values[2];

            tf::Quaternion q(rot->x, rot->y, rot->z, rot->w);
            tf::Matrix3x3 m(q);
            m.getRPY(result_ori(0), result_ori(1), result_ori(2)); // get Roll, Pitch, Yaw

            _result_mat_pos.block<3,1>(0, path_idx) = result_vec;
            _result_mat_ori.block<3,1>(0, path_idx) = result_ori;
        }
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void CMotionPlan::solveWithoutSmoothing()
{
    constructStateSpace();
    constructSpaceInformation();
    createProblemDefinition();
    setOptimalPlanner();

    _solved = _planner->ob::Planner::solve(5.0);
    
    if (_solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path

        // ob::PathPtr path = _pdef->getSolutionPath();
		og::PathGeometric* pth = _pdef->getSolutionPath()->as<og::PathGeometric>();

        Vector3d result_vec;
		Vector3d result_ori;

        _state_count = pth->getStateCount();

        _result_mat_pos.resize(3, _state_count);
		_result_mat_ori.resize(3, _state_count);
		_result_mat_pos.setZero(3, _state_count);
		_result_mat_ori.setZero(3, _state_count);
		
        for (std::size_t path_idx = 0; path_idx < _state_count; path_idx++)
		{
			const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
			
			result_vec(0) = pos->values[0];
			result_vec(1) = pos->values[1];
			result_vec(2) = pos->values[2];
			
            tf::Quaternion q(rot->x, rot->y, rot->z, rot->w);
            tf::Matrix3x3 m(q);
            m.getRPY(result_ori(0), result_ori(1), result_ori(2)); // get Roll, Pitch, Yaw

            _result_mat_pos.block<3,1>(0, path_idx) = result_vec;
            _result_mat_ori.block<3,1>(0, path_idx) = result_ori;
        }
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void CMotionPlan::constructStateSpace()
{
    // construct the state space we are planning in
    ob::StateSpacePtr se3(new ompl::base::SE3StateSpace());

    _space = se3;
    setBoundsOfState(_low_bounds,_high_bounds);
}

void CMotionPlan::setBoundsOfState(Vector3d low_bounds, Vector3d high_bounds){
    
    ob::RealVectorBounds bounds(3);

	for (int i = 0; i < 3; i++)
	{
		bounds.setLow(i, low_bounds(i));
		bounds.setHigh(i, high_bounds(i));
	}
    _space->as<ob::SE3StateSpace>()->setBounds(bounds);
}

void CMotionPlan::constructSpaceInformation()
{
    // construct an instance of  space information from this state space
    _si = std::make_shared<ob::SpaceInformation>(_space);
	_si->setStateValidityChecker(isStateValid);
	_si->setup();
}

void CMotionPlan::setStartState(Vector3d start_pos, Vector3d start_ori)
{
    _start_pos = start_pos;
    _start_ori = start_ori;
}

void CMotionPlan::setGoalState(Vector3d goal_pos, Vector3d goal_ori)
{
    _goal_pos = goal_pos;
    _goal_ori = goal_ori;
}

void CMotionPlan::createProblemDefinition()
{
    // create a problem instance
    _pdef = std::make_shared<ob::ProblemDefinition>(_si);

    ob::State *start = _space->allocState();
	start->as<ob::SE3StateSpace::StateType>()->setX(_start_pos(0));
	start->as<ob::SE3StateSpace::StateType>()->setY(_start_pos(1));
	start->as<ob::SE3StateSpace::StateType>()->setZ(_start_pos(2));

    tf::Quaternion q_start;
    q_start.setRPY(_start_ori(0),_start_ori(1),_start_ori(2));

    start->as<ob::SE3StateSpace::StateType>()->rotation().x = q_start[0];
	start->as<ob::SE3StateSpace::StateType>()->rotation().y = q_start[1];
	start->as<ob::SE3StateSpace::StateType>()->rotation().z = q_start[2];
	start->as<ob::SE3StateSpace::StateType>()->rotation().w = q_start[3];

	_pdef->addStartState(start);
	_space->freeState(start);

    ob::State *goal = _space->allocState();
	goal->as<ob::SE3StateSpace::StateType>()->setX(_goal_pos(0));
	goal->as<ob::SE3StateSpace::StateType>()->setY(_goal_pos(1));
	goal->as<ob::SE3StateSpace::StateType>()->setZ(_goal_pos(2));

    tf::Quaternion q_goal;
    q_goal.setRPY(_goal_ori(0), _goal_ori(1), _goal_ori(2));

	goal->as<ob::SE3StateSpace::StateType>()->rotation().x = q_goal[0];
	goal->as<ob::SE3StateSpace::StateType>()->rotation().y = q_goal[1];
	goal->as<ob::SE3StateSpace::StateType>()->rotation().z = q_goal[2];
	goal->as<ob::SE3StateSpace::StateType>()->rotation().w = q_goal[3];

	_pdef->setGoalState(goal); 
	_space->freeState(goal);

}

void CMotionPlan::setOptimalPlanner()
{   
    // select planner type
	_planner = std::make_shared<og::RRTstar>(_si);

    // _planner = std::make_shared<og::RRTConnect>(_si);
    // _planner = std::make_shared<og::RRT>(_si);

    // _planner = std::make_shared<og::PRM>(_si);
    // _planner = std::make_shared<og::PRMstar>(_si);

    // setProblemDefinition (const ProblemDefinitionPtr &pdef)
    _planner->setProblemDefinition(_pdef);
	_planner->setup();
}

Eigen::MatrixXd CMotionPlan::getResultPosMatrix()
{
	return _result_mat_pos;
}

Eigen::MatrixXd CMotionPlan::getResultOriMatrix()
{
	return _result_mat_ori;
}

void CMotionPlan::initialize()
{   
    // set environment boundary 
    _low_bounds(0) = -3.0;
    _low_bounds(1) = -3.0;
    _low_bounds(2) = -3.0;

    _high_bounds(0) = 3.0;
    _high_bounds(1) = 3.0;
    _high_bounds(2) = 3.0;
    
    _start_pos.setZero();
    _start_ori.setZero();

    _goal_pos.setZero();
    _goal_ori.setZero();

    _result_mat_pos.setZero(3, 100);
	_result_mat_ori.setZero(3, 100);

}