#include <ros/ros.h>
#include <task_master/TaskStatus.h>
#include <task_master/TaskGoalPosition.h>
#include <task_master/Task.h>
#include <string>
#include <prop_mapper/PropArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Point.h>

class NavChannel {
public:
    NavChannel(): nh_(""), private_nh_("~"), green_id_(-1), red_id_(-1)
    {
        // ROS parameters
        private_nh_.param<double>("error", wp_error_tolerance, 1.0); // Tolerance radius for determining if asv reached gate waypoint
        private_nh_.param<double>("gate_max_dist", gate_max_dist, 8.0); // max distance between asv and gate 1 at start, determines if we are looking at the right gate
        private_nh_.param<double>("gate_max_width", gate_max_width, 4.0); // max distance between two buoys in gate, accounts for max gap and diameter
        private_nh_.param<double>("dist_past_gate", dist_past_gate, 2.0);
        private_nh_.param<std::string>("red_marker", red_marker_str, "red_marker");
        private_nh_.param<std::string>("green_marker", green_marker_str, "green_marker");

        // ROS subscribers
        private_nh_.param<std::string>("local_pose_topic", local_pose_topic_, "/mavros/local_position/pose");

        // task_master dictates which task is is be executed
        task_to_exec_ = nh_.subscribe("task_to_execute", 10, &NavChannel::navChannelCallback, this);

        prop_map_ = nh_.subscribe("/prop_array", 10, &NavChannel::propMapCallback, this);
        
        global_pos_ = nh_.subscribe(local_pose_topic_, 10, &NavChannel::localPositionCallback, this);

        // ROS publishers

        // publishes task progress to /task_status
        task_status_ = nh_.advertise<task_master::TaskStatus>("task_status", 10);

        // publishes desired positions to /task_goal_position to move asv
        task_goal_position_ = nh_.advertise<task_master::TaskGoalPosition>("task_goal_position", 10);
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber task_to_exec_;
    ros::Subscriber prop_map_;
    ros::Subscriber global_pos_;
    ros::Publisher task_status_;
    ros::Publisher task_goal_position_;

    std::string local_pose_topic_;

    geometry_msgs::PoseStamped current_pos_;
    prop_mapper::PropArray props_; //temporarily replacing with fake props
    task_master::TaskGoalPosition goal_pos_;

    std::string TAG = "NAV_CHANNEL_CTRL: ";

    double wp_error_tolerance;
    double gate_max_dist;
    double gate_max_width;
    double dist_past_gate;
    std::string red_marker_str;
    std::string green_marker_str;

    // Marker IDs
    int green_id_;
    int red_id_;

    enum Colour 
    {
        RED,
        GREEN,
        BLUE
    };

    enum States {NOT_STARTED, FIND_GATE1, MOVE_TO_GATE1, FIND_GATE2, MOVE_TO_GATE2, COMPLETE};

    States status = States::NOT_STARTED;

    void setDestination() 
    {
        ROS_DEBUG_STREAM(TAG << "setDestination() called");
        goal_pos_.task.current_task = task_master::Task::NAVIGATION_CHANNEL; 
        
        ROS_INFO_STREAM(TAG << "Midpoint set to x: " << goal_pos_.point.x << ", y: "<< goal_pos_.point.y);

        task_goal_position_.publish(goal_pos_);
    }

    bool isValidMarker(prop_mapper::Prop marker, Colour colour)
    {
        ROS_DEBUG_STREAM(TAG << "isValidMarker() called");
        bool valid = false;
        if (marker.prop_label == red_marker_str && colour == Colour::RED && marker.id != red_id_) {
            ROS_INFO_STREAM(TAG << "Valid red marker found");
            valid = true;
        }
        if (marker.prop_label == green_marker_str && colour == Colour::GREEN && marker.id != green_id_) {
            ROS_INFO_STREAM(TAG << "Valid green marker found");
            valid = true;
        }
        return valid;
    }

    bool isValidGate(prop_mapper::Prop red_marker, prop_mapper::Prop green_marker)
    {
        ROS_DEBUG_STREAM(TAG << "isValidGate() called");
        bool valid = true;
        double dist = sqrt(pow((red_marker.point.x - green_marker.point.x), 2) + pow((red_marker.point.y - green_marker.point.y), 2));
        
        if (dist > gate_max_width) {
            valid = false;
            ROS_DEBUG_STREAM(TAG << "Markers too far apart for valid gate"); // TODO add reference to prop IDs
        }

        //double red_polar_angle = findPolarAngle(red_marker); TODO FIX
        //double green_polar_angle = findPolarAngle(green_marker);
//
        //if (abs(green_polar_angle-red_polar_angle) > 180) {
        //    if (red_polar_angle > green_polar_angle) {
        //        valid = false;
        //        ROS_DEBUG_STREAM(TAG << "Red marker NOT to the right of green marker -> Invalid gate");
        //    }
        //}
        //else if (green_polar_angle > red_polar_angle) {
        //    valid = false;
        //    ROS_DEBUG_STREAM(TAG << "Red marker NOT to the right of green marker -> Invalid gate");
        //}
        if (valid)
        {
            ROS_INFO_STREAM(TAG << "Valid gate Identified"); // TODO add references to props IDs
        }
        return valid;
    }

    double findPolarAngle(prop_mapper::Prop marker) {// TODO remove if nto needed
        ROS_DEBUG_STREAM(TAG << "findPolarAngle() called");
        double angle;
        if (marker.point.x > 0 && marker.point.y > 0) {
            angle = atan2(marker.point.y, marker.point.x);
        }
        else if (marker.point.x < 0 && marker.point.y > 0) {
            angle = atan2(marker.point.x, marker.point.y) + M_PI_2;
        }
        else if (marker.point.x < 0 && marker.point.y < 0) {
            angle = atan2(marker.point.y, marker.point.x) + M_PI;
        }
        else {
            angle = atan2(marker.point.x, marker.point.y) + M_PI + M_PI_2;
        }
        ROS_DEBUG_STREAM(TAG << "polar angle: " << angle);
        return angle;
    }

    bool findGate(prop_mapper::Prop &green_marker, prop_mapper::Prop &red_marker)
    {
        ROS_DEBUG_STREAM(TAG << "findGate() called");

        int green_idx;
        int red_idx;

        bool green_found = false;
        bool red_found = false;
        bool gate_found = false;

        int i = 0;

        // Look for valid gate in the prop_array
        while ((i<props_.props.size()) && !gate_found) //TODO take the closest valid markers, not the first occurances in array
        {
            if (isValidMarker(props_.props[i], Colour::RED))
            {
                red_idx = i;
                red_found = true;
            }
            else if (isValidMarker(props_.props[i], Colour::GREEN))
            {
                green_idx = i;
                green_found = true;
            }

            if (green_found && red_found)
            {
                ROS_DEBUG_STREAM(TAG << "Red and Green markers found");
                if (isValidGate(props_.props[red_idx], props_.props[green_idx]))
                {
                    red_marker = props_.props[red_idx];
                    green_marker = props_.props[green_idx];
                    gate_found = true;
                }
            }
            i++;
        }

        return gate_found;
    }

    geometry_msgs::Point translatePoint(geometry_msgs::Point point, double x_trans, double y_trans )
    {
        point.x = point.x + x_trans;
        point.y = point.y + y_trans;
        return point;        
    }

    geometry_msgs::Point rotatePoint(geometry_msgs::Point point, double rot_angle )
    {
        double radius = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
        double angle = std::atan2(point.y, point.x);

        angle += rot_angle;

        point.x = radius * std::cos(angle);
        point.y = radius * std::sin(angle);

        return point;
        
    }
    
    geometry_msgs::Point findEndpoint(geometry_msgs::Point marker_1, geometry_msgs::Point midpnt)
    {
        ROS_DEBUG_STREAM(TAG << "findEndpoint() called");
        // Translate the a marker so we can get the rotation angle 
        marker_1 = translatePoint(marker_1, -midpnt.x, -midpnt.y);

        // Get rotation angle to align marker on x axis
        double angle = std::atan2(marker_1.y, marker_1.x);// get the amount to rotate by
        
        geometry_msgs::Point endpnt;
        endpnt.x = 0;
        endpnt.y = dist_past_gate;

        endpnt = rotatePoint(endpnt, angle);

        endpnt = translatePoint(endpnt, midpnt.x, midpnt.y);

        ROS_INFO_STREAM(TAG << "Enpoint set " << dist_past_gate << " past the last gate at x: " << endpnt.x << ", y: " << endpnt.y );

        return endpnt;
    }

    bool findMidpoint(prop_mapper::Prop marker1, prop_mapper::Prop marker2, geometry_msgs::Point &midpnt)
    {
        // geometry_msgs::Point midpnt;
        ROS_DEBUG_STREAM(TAG << "m1 x = " << marker1.point.x << ", m1 y = " << marker1.point.y);
        ROS_DEBUG_STREAM(TAG << "m2 x = " << marker2.point.x << ", m2 y = " << marker2.point.y);
        midpnt.x = (marker1.point.x+marker2.point.x)/2;
        midpnt.y = (marker1.point.y+marker2.point.y)/2;
        midpnt.z = 0;

        if (checkLeftRight(midpnt, marker1, marker2)) {
            ROS_DEBUG_STREAM(TAG << "Midpoint x: " << midpnt.x << ", y: " << midpnt);
            return true;
        }
        else {
            return false;
        }
    }

    bool checkLeftRight(geometry_msgs::Point mid, prop_mapper::Prop green, prop_mapper::Prop red) {
        // double delta_x = current_pos_.pose.position.x;
        // double delta_y = current_pos_.pose.position.y;
        // double mid_x = mid.x - delta_x;
        // double mid_y = mid.y - delta_y;
        // double green_x = green.point.x - delta_x;
        // double green_y = green.point.y - delta_y;
        // double red_x = red.point.x - delta_x;
        // double red_y = red.point.y - delta_y;
        // double angle = std::atan2(mid_y, mid_x);
        // ROS_DEBUG_STREAM(TAG << "rotation angle = " << angle);
        // green_y = green_x * std::sin(angle) + green_y * std::cos(angle);
        // red_y = red_x * std::sin(angle) + red_y * std::cos(angle);
        // ROS_DEBUG_STREAM(TAG << "Green rotated y = " << green_y << ", Red rotated y = " << red_y);
        // if (red_y >= 0 && green_y < 0) {
            // ROS_DEBUG_STREAM(TAG << "Correct left and right buoys detected");
            // return true;
        // }
        // else {
            // ROS_DEBUG_STREAM(TAG << "Incorrect left and right buoys detected");
            // return false;
        // }

        // TODO: fix left-right checking, issue seems to be with the calculated angle
        return true;
    }

    bool isReached() {
        ROS_DEBUG_STREAM(TAG << "isReached() called");
        ROS_INFO_STREAM(TAG << "Checking if goal position reached");

        // check to see it we are at the goal (within a set amount of error)
        bool atDestination = false;
        if (current_pos_.pose.position.x < goal_pos_.point.x+wp_error_tolerance & current_pos_.pose.position.x > goal_pos_.point.x-wp_error_tolerance) {
            if (current_pos_.pose.position.y < goal_pos_.point.y+wp_error_tolerance & current_pos_.pose.position.y > goal_pos_.point.y-wp_error_tolerance) {
                atDestination = true;
                ROS_INFO_STREAM(TAG << "Goal position reached");
            }
        }

        return atDestination;
    }

    void navChannelCallback(const task_master::Task msg) {

        if(msg.current_task == task_master::Task::NAVIGATION_CHANNEL) {
            task_master::TaskStatus taskStatus;
            switch (status)
            {
            case States::NOT_STARTED: 
            {
                ROS_DEBUG_STREAM(TAG << "waiting for at least 2 markers");

                taskStatus.status = task_master::TaskStatus::IN_PROGRESS;
                taskStatus.task.current_task = task_master::Task::NAVIGATION_CHANNEL;

                if (props_.props.size() >= 2)
                {
                    ROS_INFO_STREAM(TAG << "At least 2 markers detected, moving on to find gate 1");
                    status = States::FIND_GATE1;
                }
                break;
            }
            case States::FIND_GATE1: {
                // if have two good props, ie. red on left, green on right, within acceptable distance of each other, then go

                ROS_INFO_STREAM(TAG << "Looking for the 1st gate");
                
                prop_mapper::Prop green_marker; // TODO change to Prop ID
                prop_mapper::Prop red_marker; // TODO change to Prop ID
                
                if (findGate(green_marker, red_marker)) 
                {
                    geometry_msgs::Point midpnt;
                    if(findMidpoint(green_marker, red_marker, midpnt)) {
                        goal_pos_.point = midpnt;
                        green_id_ = green_marker.id;
                        red_id_ = red_marker.id;
                        status = States::MOVE_TO_GATE1;
                    }
                }
                }
                break;

            case States::MOVE_TO_GATE1: {
                ROS_INFO_STREAM(TAG << "Moving to first gate");
                if(!isReached())
                {
                    ROS_DEBUG_STREAM(TAG << "Midpoint 1 not reached yet");
                    setDestination();
                    ros::Rate rate(10);
                    rate.sleep();
                }
            
                if (isReached()) {
                    status = States::FIND_GATE2;
                    ROS_DEBUG_STREAM(TAG << "Midpoint 1 reached");
                };
                }
                break;
            // should consider case where props are 100ft out, being out of lidar range

            case States::FIND_GATE2: {

                ROS_INFO_STREAM(TAG << "Looking for the 2nd gate");
                
                prop_mapper::Prop green_marker; // TODO change to Prop ID
                prop_mapper::Prop red_marker; // TODO change to Prop ID
                
                if (findGate(green_marker, red_marker)) 
                {
                    geometry_msgs::Point midpnt;
                    if (findMidpoint(green_marker, red_marker, midpnt)) {
                        goal_pos_.point = findEndpoint(green_marker.point, midpnt); // extends the midpoint to actually pass through the gate
                        status = States::MOVE_TO_GATE2;
                    }
                }
                }
                break;

            case States::MOVE_TO_GATE2: {

                ROS_INFO_STREAM(TAG << "Moving to 2nd gate");
                if(!isReached())
                {
                    ROS_DEBUG_STREAM(TAG << "Midpoint 2 not reached yet");
                    setDestination();
                    ros::Rate rate(10);
                    rate.sleep();
                }
            
                if (isReached()) {
                    status = States::COMPLETE;
                    ROS_DEBUG_STREAM(TAG << "Midpoint 2 reached");
                };
                }
                break;

            case States::COMPLETE: {
                ROS_INFO_STREAM(TAG << "Navigation Channel Complete");
                taskStatus.status = task_master::TaskStatus::COMPLETE;
                task_status_.publish(taskStatus);
                }
                break;

            default:
                break;
            }
            task_status_.publish(taskStatus);
        }
    }

    // prop_array is a list of props, which are themselves a
    // msg with a string (prop_label) and a Point position
    void propMapCallback(const prop_mapper::PropArray msg) {
        props_ = msg;
    }

    void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pos_ = *msg;
    }
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_channel_ctrl");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    NavChannel nav_channel;

    nav_channel.spin();

    return 0;
}