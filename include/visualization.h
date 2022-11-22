#include <ros/ros.h>
#include <tf/tf.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <Eigen/Dense>

#define PI 3.14159265358979323846 /* pi */

using namespace Eigen;
using namespace std;

class Visualization{
    public:
        Visualization(const ros::NodeHandle& nh);
        ~Visualization();

    private:
        ros::NodeHandle _nh;

        ros::Publisher pub_MarkerArray;
        visualization_msgs::Marker _marker;
        visualization_msgs::MarkerArray _markerArray;

        MatrixXd _posMatrix;
        MatrixXd _oriMatrix;
        int _size;
        
        visualization_msgs::Marker makerGenerator(int i);


    public: 
        void getPath(MatrixXd posMatrix, MatrixXd oriMatrix);
        void publishAllMarker();
        void stateUpdate(double i);
};
