#include "motionplan.h"
#include "visualization.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "visualization");
    ros::NodeHandle nh_;

    Visualization visual(nh_);

    CMotionPlan path;

    Vector3d s_state;
    Vector3d s_ori;
    Vector3d g_state;
    Vector3d g_ori;

    for(int i = 0; i < 3; i++)
    {
        s_state(i) = -1.0;
        s_ori(i) = 0.0;
        g_state(i) = 1.0;
        g_ori(i) = 0.0;
    }
    
    // tf::Quaternion q(-0.255752365305, 0.541094747431, -0.72133327014, -0.348547723177);
    // tf::Matrix3x3 m(q);
    // m.getRPY(g_ori(0), g_ori(1), g_ori(2)); // get Roll, Pitch, Yaw
    
    path.setStartState(s_state, s_ori);
    path.setGoalState(g_state, g_ori);
    // path.solveWithoutSmoothing();
    
    MatrixXd result_pos;
    MatrixXd result_ori;

    path.solve();
    result_pos = path.getResultPosMatrix();
    result_ori = path.getResultOriMatrix();

    visual.getPath(result_pos, result_ori);
    visual.publishAllMarker();


    // ros::Rate loop_rate(0.5);
    // while(ros::ok())
    // {
    //     MatrixXd result_pos;
    //     MatrixXd result_ori;

    //     path.solve();
    //     result_pos = path.getResultPosMatrix();
    //     result_ori = path.getResultOriMatrix();

    //     visual.getPath(result_pos, result_ori);
    //     visual.publishAllMarker();

    //     loop_rate.sleep();
    // }

    return 0;
}
