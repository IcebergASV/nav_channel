#ifndef WAYPOINTSENDERLIB_H
#define WAYPOINTSENDERLIB_H

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

 struct waypoint {
     float x;
     float y;
     float z;
     float psi;
 };

geometry_msgs::Point enu_2_local(nav_msgs::Odometry current_pose_enu);
geometry_msgs::Point get_current_location(nav_msgs::Odometry current_pose);
geometry_msgs::PoseStamped set_heading(float heading, float local_desired_heading, float correction_heading, geometry_msgs::PoseStamped waypoint);
geometry_msgs::PoseStamped set_destination(float x, float y, float z, float psi, float correction_heading, float local_desired_heading, geometry_msgs::Pose correction_vector, geometry_msgs::Point local_offset_pose, geometry_msgs::PoseStamped waypoint);
int check_waypoint_reached(float tolerance, nav_msgs::Odometry current_pose, geometry_msgs::PoseStamped waypoint);


#endif