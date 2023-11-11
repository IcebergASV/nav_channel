#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>

#include "waypointSenderLib.h"

geometry_msgs::Point enu_2_local(nav_msgs::Odometry current_pose_enu)
{
  float x = current_pose_enu.pose.pose.position.x;
  float y = current_pose_enu.pose.pose.position.y;
  float z = current_pose_enu.pose.pose.position.z;
  float deg2rad = (M_PI/180);
  geometry_msgs::Point current_pos_local;
  current_pos_local.x = x*cos((-90)*deg2rad) - y*sin((-90)*deg2rad);
  current_pos_local.y = x*sin((-90)*deg2rad) + y*cos((-90)*deg2rad);
  current_pos_local.z = z;

  return current_pos_local;
}

geometry_msgs::Point get_current_location(nav_msgs::Odometry current_pose)
{
	geometry_msgs::Point current_pos_local;
	current_pos_local = enu_2_local(current_pose);
	return current_pos_local;
}

geometry_msgs::PoseStamped set_heading(float heading, float local_desired_heading, float correction_heading, geometry_msgs::PoseStamped waypoint)
{
  local_desired_heading = heading; 
  heading = heading + correction_heading;
  
  ROS_INFO("Desired Heading %f ", local_desired_heading);
  float yaw = heading*(M_PI/180);
  float pitch = 0;
  float roll = 0;

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  float qw = cy * cr * cp + sy * sr * sp;
  float qx = cy * sr * cp - sy * cr * sp;
  float qy = cy * cr * sp + sy * sr * cp;
  float qz = sy * cr * cp - cy * sr * sp;

  waypoint.pose.orientation.w = qw;
  waypoint.pose.orientation.x = qx;
  waypoint.pose.orientation.y = qy;
  waypoint.pose.orientation.z = qz;

  return waypoint;
}

geometry_msgs::PoseStamped set_destination(float x, float y, float z, float psi, float correction_heading, float local_desired_heading, geometry_msgs::Pose correction_vector, geometry_msgs::Point local_offset_pose, geometry_msgs::PoseStamped waypoint)
{
	waypoint = set_heading(psi, local_desired_heading, correction_heading, waypoint);

	//transform map to local
	float deg2rad = (M_PI/180);
	float Xlocal = x*cos((correction_heading - 90)*deg2rad) - y*sin((correction_heading - 90)*deg2rad);
	float Ylocal = x*sin((correction_heading - 90)*deg2rad) + y*cos((correction_heading - 90)*deg2rad);
	float Zlocal = z;

	x = Xlocal + correction_vector.position.x;
	y = Ylocal + correction_vector.position.y;
	z = Zlocal + correction_vector.position.z;
	ROS_INFO("Destination set to x: %f y: %f z: %f origin frame", x, y, z);

	waypoint.pose.position.x = -x;
	waypoint.pose.position.y = -y;
	waypoint.pose.position.z = z;

	return waypoint;
}

/**
@return 1 - waypoint reached 
@return 0 - waypoint not reached
*/
int check_waypoint_reached(float tolerance, nav_msgs::Odometry current_pose, geometry_msgs::PoseStamped waypoint)
{
	float deltaX = abs(waypoint.pose.position.x - current_pose.pose.pose.position.x);
  float deltaY = abs(waypoint.pose.position.y - current_pose.pose.pose.position.y);
  float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2));
  ROS_INFO("dMag %f", dMag);
  ROS_INFO("current pose x %F y %f z %f", (current_pose.pose.pose.position.y), -1 *(current_pose.pose.pose.position.x), (current_pose.pose.pose.position.z));
  ROS_INFO("waypoint pose x %F y %f z %f", waypoint.pose.position.y, -1 * waypoint.pose.position.x, waypoint.pose.position.z);

  if( dMag < tolerance)
	{
		return 1;
	}
  else
  {
		return 0;
	}
}