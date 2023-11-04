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

#include "waypointSender.h"

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

//get current position of drone

float pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  nav_msgs::Odometry current_pose = *msg;
  enu_2_local(current_pose);
  float q0 = current_pose.pose.pose.orientation.w;
  float q1 = current_pose.pose.pose.orientation.x;
  float q2 = current_pose.pose.pose.orientation.y;
  float q3 = current_pose.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  float current_heading = psi*(180/M_PI);
  //IS YAWING COUNTERCLOCKWISE POSITIVE?
  return current_heading;

}

geometry_msgs::Point get_current_location(nav_msgs::Odometry current_pose)
{
	geometry_msgs::Point current_pos_local;
	current_pos_local = enu_2_local(current_pose);
	return current_pos_local;
}

//set orientation of the drone (drone should always be level) 
// Heading input should match the ENU coordinate system
/**
\ingroup control_functions
This function is used to specify the drone’s heading in the local reference frame. Psi is a counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
@returns n/a
*/
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

// set position to fly to in the local frame
/**
\ingroup control_functions
This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
@returns n/a
*/
geometry_msgs::PoseStamped set_destination(float x, float y, float z, float psi, float correction_heading, float local_desired_heading, geometry_msgs::Pose correction_vector, geometry_msgs::Point local_offset_pose, geometry_msgs::PoseStamped waypoint)
{
	set_heading(psi, local_desired_heading, correction_heading, waypoint);

	//transform map to local
	float deg2rad = (M_PI/180);
	float Xlocal = x*cos((correction_heading - 90)*deg2rad) - y*sin((correction_heading - 90)*deg2rad);
	float Ylocal = x*sin((correction_heading - 90)*deg2rad) + y*cos((correction_heading - 90)*deg2rad);
	float Zlocal = z;

	x = Xlocal + correction_vector.position.x;
	y = Ylocal + correction_vector.position.y;
	z = Zlocal + correction_vector.position.z;
	ROS_INFO("Destination set to x: %f y: %f z: %f origin frame", x, y, z);

	waypoint.pose.position.x = x;
	waypoint.pose.position.y = y;
	waypoint.pose.position.z = z;

	return waypoint;
}

/**
\ingroup control_functions
This function returns an int of 1 or 0. THis function can be used to check when to request the next waypoint in the mission. 
@return 1 - waypoint reached 
@return 0 - waypoint not reached
*/
int check_waypoint_reached(float tolerance, float current_heading, float local_desired_heading, nav_msgs::Odometry current_pose, geometry_msgs::PoseStamped waypoint)
{
	
	//check for correct position 
	float deltaX = abs(waypoint.pose.position.x - current_pose.pose.pose.position.x);
  float deltaY = abs(waypoint.pose.position.y - current_pose.pose.pose.position.y);
  float deltaZ = 0;
  float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
  ROS_INFO("dMag %f", dMag);
  ROS_INFO("current pose x %F y %f z %f", (current_pose.pose.pose.position.x), (current_pose.pose.pose.position.y), (current_pose.pose.pose.position.z));
  ROS_INFO("waypoint pose x %F y %f z %f", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
  //check orientation
  float cosErr = cos(current_heading*(M_PI/180)) - cos(local_desired_heading*(M_PI/180));
  float sinErr = sin(current_heading*(M_PI/180)) - sin(local_desired_heading*(M_PI/180));
  
  float headingErr = sqrt( pow(cosErr, 2) + pow(sinErr, 2) );
  ROS_INFO("current heading error %f", headingErr);
  if( dMag < tolerance) //&& headingErr < 0.01)
	{
		return 1;
	}else{
		return 0;
	}
}