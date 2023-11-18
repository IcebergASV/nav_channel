#include <ros/ros.h>
#include <nav_channel/TaskStatus.h>
#include <nav_channel/TaskGoalPosition.h>
#include <nav_channel/Task.h>
#include <string>
#include <prop_mapper/PropArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>

class NavChannel {
public:
    NavChannel(): nh_(""), private_nh_("~")
    {
        // ROS parameters
        private_nh_.param<double>("error", error, 1.0); // leniency for if we are at the gate
        private_nh_.param<double>("gate_max_dist", gate_max_dist, 8.0); // max distance for gate 1, determines if we are looking at the right gate, 8.0m is roughly 25ft, can make more accurate if needed.
        private_nh_.param<double>("gate_max_width", gate_max_width, 4.0); // max distance between two buoys for a gate, confirming we have the two correct ones. 4.0m is roughly 13ft, accounts for max gap and diameter.


        // ROS subscribers

        // task_to_exec will confirm we are on nav_channel, then execute task
        task_to_exec_ = nh_.subscribe("task_to_execute", 10, &NavChannel::navChannelCallback, this);
        
        // props will get the prop map and store it
        prop_map_ = nh_.subscribe("props", 10, &NavChannel::propMapCallback, this);

        // global_position/local grabs position from mavros and stores it
        global_pos_ = nh_.subscribe("mavros/global_position/local", 10, &NavChannel::globalPositionCallback, this);


        // ROS publishers

        // we publish to task_status to update us on progress of task
        task_status_ = nh_.advertise<nav_channel::TaskStatus>("task_status", 10);

        // we publish to task_goal_position to send the boat to a waypoint
        task_goal_position_ = nh_.advertise<nav_channel::TaskGoalPosition>("task_goal_position", 10);
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    void setDestination(geometry_msgs::PoseStamped midpoint) {
        // sets goal_pos and then publishes it
        goal_pos_.goal_pose = midpoint;
        // does this set orientation to (0,0,0,0), do we need to grab current orientation and set that to new orientation?

        ROS_INFO_STREAM("Midpoint set at " << midpoint.pose.position.x << ","<< midpoint.pose.position.y << "," << midpoint.pose.position.z);

        task_goal_position_.publish(goal_pos_);
    }

    geometry_msgs::PoseStamped findMidpoint(int gate) {
        // validates positions of props, then calculates midpoint, and returns
        // it as PoseStamped
        geometry_msgs::PoseStamped midpoint;

        bool red = false;   // use two booleans to determine if props exist
        bool green = false;
        prop_mapper::Prop red_prop;
        prop_mapper::Prop green_prop;
        ROS_DEBUG_STREAM("Gate #" << gate);

        for (int i = 0; (!red || !green) || i >= sizeof(props_.props) ; i++) {
            ROS_INFO("here 1");
            if (props_.props[i].prop_label == "Red Prop" || props_.props[i].prop_label == "Green Prop") {
                ROS_INFO("here 2");
                double dist_to_gate = sqrt(pow(props_.props[i].vector.x - current_pos_.pose.pose.position.x, 2) + pow(props_.props[i].vector.y - current_pos_.pose.pose.position.y, 2));
                if ((gate == 1 && dist_to_gate <= gate_max_dist) || (gate == 2 && dist_to_gate > gate_max_dist)) {
                // using 8m as roughly 25ft
                // gate 1 should be within 25ft, gate 2 should be at least 25ft away
                    ROS_INFO("here 3");
                    if (props_.props[i].prop_label == "Red Prop") {
                        if (green) {
                            ROS_INFO("here 4");
                            float dist = sqrt(pow(green_prop.vector.x - props_.props[i].vector.x, 2) + pow(green_prop.vector.y - props_.props[i].vector.y, 2));
                            if (dist < gate_max_width) {
                                red = true;
                                red_prop  = props_.props[i];
                                ROS_INFO_STREAM("Red Buoy found at " << red_prop.vector.x << ", " << red_prop.vector.y);
                            }
                        }
                        else {
                            ROS_INFO("here 5");
                            red = true;
                            red_prop  = props_.props[i];
                        }
                    }

                    if (props_.props[i].prop_label == "Green Prop") {
                        if (red) {
                            ROS_INFO("here 6");
                            float dist = sqrt(pow(red_prop.vector.x - props_.props[i].vector.x, 2) + pow(red_prop.vector.y - props_.props[i].vector.y, 2));
                            if (dist < gate_max_width) {
                                green = true;
                                green_prop = props_.props[i];
                                ROS_INFO_STREAM("Green Buoy found at " << green_prop.vector.x << ", " << green_prop.vector.y);
                            }
                        }
                        else {
                            ROS_INFO("here 7");
                            green = true;
                            green_prop  = props_.props[i];
                        }
                    }
                }
            }
        }
            
        if (red && green) {
            ROS_INFO("here 8");
            float red_x = red_prop.vector.x;
            float red_y = red_prop.vector.y;
            //float red_z = red_prop.vector.z;
            float green_x = green_prop.vector.x;
            float green_y = green_prop.vector.y;
            //float green_z = green_prop.vector.z;

            midpoint.pose.position.x = (red_x+green_x)/2;
            midpoint.pose.position.y = (red_y+green_y)/2;
            //midpoint.pose.position.z = (red_z+green_z)/2;
            midpoint.pose.position.z = 0;
        }

        else {
            ROS_INFO("gates were not found.");
        }

        ROS_DEBUG("returning midpoint.");

        return midpoint;
    }

    bool travelling() {
        ROS_DEBUG("travelling...");
        // check to see it we are at the goal (within a set amount of error)
        bool atDestination = false;
        if (current_pos_.pose.pose.position.x < goal_pos_.goal_pose.pose.position.x+error & current_pos_.pose.pose.position.x > goal_pos_.goal_pose.pose.position.x-error) {
            if (current_pos_.pose.pose.position.y < goal_pos_.goal_pose.pose.position.y+error & current_pos_.pose.pose.position.y > goal_pos_.goal_pose.pose.position.y-error) {
                atDestination = true;
            }
        }

        return atDestination;
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber task_to_exec_;
    ros::Subscriber prop_map_;
    ros::Subscriber global_pos_;
    ros::Publisher task_status_;
    ros::Publisher task_goal_position_;

    nav_msgs::Odometry current_pos_;
    prop_mapper::PropArray props_; //temporarily replacing with fake props
    nav_channel::TaskGoalPosition goal_pos_;

    double error;
    double gate_max_dist;
    double gate_max_width;
    enum states {not_started, find_wp1, moving_to_wp1, find_wp2, moving_to_wp2, complete};
    states status = states::not_started;

    void navChannelCallback(const nav_channel::TaskStatus msg) {
        if(msg.task.current_task == nav_channel::Task::NAVIGATION_CHANNEL) {
            // start task
            nav_channel::TaskStatus taskStatus;
            taskStatus.status = nav_channel::TaskStatus::IN_PROGRESS;
            task_status_.publish(taskStatus);

            switch (status)
            {
            case states::not_started: {
                ROS_INFO("in not started case.");
                status = states::find_wp1;
                }
                break;
            
            case states::find_wp1: {
                // if have two good props, ie. red on left, green on right, within 10 feet of each other, then go

                ROS_INFO("start task.");
                geometry_msgs::PoseStamped midpoint = findMidpoint(1);
                setDestination(midpoint);

                status = states::moving_to_wp1;
                }
                break;

            case states::moving_to_wp1: {

                bool at_destination = travelling();
                
                if (at_destination) {
                    status = states::find_wp2;
                };
                }
                break;

            // should consider case where props are 100ft out, being out of lidar range

            case states::find_wp2: {
                
                ROS_INFO("at gate 1.");
                geometry_msgs::PoseStamped midpoint = findMidpoint(2);
                setDestination(midpoint);

                status = states::moving_to_wp2;
                }
                break;

            case states::moving_to_wp2: {
                
                bool at_destination = travelling();
                
                if (at_destination) {
                    status = states::complete;
                };
                }
                break;

            case states::complete: {

                taskStatus.status = nav_channel::TaskStatus::COMPLETE;
                ROS_DEBUG("at gate 2");
                ROS_DEBUG("TASK COMPLETE");
                task_status_.publish(taskStatus);
                }
                break;

            default:
                break;
            }
        }
    }

    // prop_array is a list of props, which are themselves a
    // msg with a string (prop_label) and a Vector3 position
    void propMapCallback(const prop_mapper::PropArray msg) {
        props_ = msg;
    }

    void globalPositionCallback(const nav_msgs::Odometry msg) {
        // set our current position
        current_pos_ = msg;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_channel_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();

    NavChannel nav_channel;

    nav_channel.spin();

    return 0;
}