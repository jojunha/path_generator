#include "visualization.h"

Visualization::Visualization(const ros::NodeHandle& nh) : _nh(nh)
{
    pub_MarkerArray = _nh.advertise<visualization_msgs::MarkerArray>("/visualization",1);
}

Visualization::~Visualization()
{
}

void Visualization::getPath(MatrixXd posMatrix, MatrixXd oriMatrix)
{   
    _size = posMatrix.cols();
    _posMatrix = posMatrix;
    _oriMatrix = oriMatrix;
}

visualization_msgs::Marker Visualization::makerGenerator(int i)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.type = visualization_msgs::Marker::ARROW;
    marker.id = i;

    tf::Quaternion q;
    q.setRPY(_oriMatrix(0,i), _oriMatrix(1,i), _oriMatrix(2,i));

    marker.pose.position.x = _posMatrix(0,i);
    marker.pose.position.y = _posMatrix(1,i);
    marker.pose.position.z = _posMatrix(2,i);
    marker.pose.orientation.x = q.getX();
    marker.pose.orientation.y = q.getY();
    marker.pose.orientation.z = q.getZ();
    marker.pose.orientation.w = q.getW();
    
    // Set the scale of the marker
    marker.scale.x = 0.1;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // start, end point check.
    if((i == 0) || i ==(_size-1))
    {
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
    }
    marker.lifetime = ros::Duration(100);
    
    return marker;
}

void Visualization::publishAllMarker()
{
    visualization_msgs::MarkerArray _Array;
    for(int i = 0; i < _size; i++)
    {
        _marker = Visualization::makerGenerator(i);
        _Array.markers.push_back(_marker);
    }

    pub_MarkerArray.publish(_Array);
    _Array.markers.clear();
}