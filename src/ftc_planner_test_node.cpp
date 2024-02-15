#include <ros/ros.h>
#include "ftc_local_planner/PlannerGetProgress.h"
#include "ftc_local_planner/recovery_behaviors.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <ftc_local_planner/FTCPlannerConfig.h>
#include <ftc_local_planner/PID.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include "tf2_eigen/tf2_eigen.h"
#include <mbf_costmap_core/costmap_controller.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ftc_planner_test_node");

    geometry_msgs::TransformStamped map_to_base;
    map_to_base.transform.rotation.x = -0.014;
    map_to_base.transform.rotation.y = -0.009;
    map_to_base.transform.rotation.z =  0.323;
    map_to_base.transform.rotation.w = -0.946;
    map_to_base.transform.translation.x = -12.472;
    map_to_base.transform.translation.y = 5.585;
    map_to_base.transform.translation.z = 0.04;

    geometry_msgs::Pose pose;
    pose.position.x = -12.38;
    pose.position.y = 5.651;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = -0.69;
    pose.orientation.w = 0.724;
    Eigen::Affine3d current_control_point;
    tf2::fromMsg(pose, current_control_point);

    Eigen::Affine3d local_control_point;

    tf2::doTransform(current_control_point, local_control_point, map_to_base);

    double lat_error = local_control_point.translation().y();
    double lon_error = local_control_point.translation().x();
    double angle_error = local_control_point.rotation().eulerAngles(0, 1, 2).z();

    tf2::Quaternion q;
    tf2::fromMsg(map_to_base.transform.rotation,q);
    tf2::quatRotate();
    tf2::Matrix3x3 m(q);
        
    double r,p,y;
    m.getRPY(r,p,y);
    ROS_INFO_STREAM("FTCLocalPlannerROS: Angular err: EA " << angle_error << " +pi" << (angle_error + M_PI) << " Y " << y );
}