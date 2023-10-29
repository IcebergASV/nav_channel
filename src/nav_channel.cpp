#include <ros/ros.h>
#include <nav_channel/TaskStatus.h>
#include <nav_channel/TaskGoalPosition.h>
#include <nav_channel/Task.h>
#include <string>
#include <prop_mapper/PropArray.h> // isn't included yet
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class NavChannel {
public:
    NavChannel():
    {
        // task_to_exec will confirm we are on nav_channel, then execute task
        task_to_exec = nh.subscribe("task_to_execute", 10, navChannelCallback);
        
        // props will get the prop map, and return relevant information
        prop_map = nh.subscribe("props", 10, propMapCallback);

        // global_position/local grabs position from mavros
        global_pos = nh.subscribe("global_position/local", 10, globalPositionCallback);

        // we publish to task_status to update us on progress of task
        task_status = nh.advertise<nav_channel::TaskStatus>("task_status", 10);

        // we publish to task_goal_position to send the boat to a waypoint
        task_goal_position = nh.advertise<nav_channel::TaskGoalPosition>("task_goal_position", 10);
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    void setDestination(float x, float y, float z) {
        // sets goal_pos and then publishes it
        goal_pos.goal_pose.pose.position.x = x;
        goal_pos.goal_pose.pose.position.y = y;
        goal_pos.goal_pose.pose.position.z = z;

        task_goal_position.pub(goal_pos);
    }

private:

    ros::NodeHandle nh;
    ros::Subscriber task_to_exec;
    ros::Subscriber prop_map;
    ros::Subscriber global_pos;
    ros::Publisher task_status;
    ros::Publisher task_goal_position;

    geometry_msgs::PoseWithCovarianceStamped current_pos;
    prop_mapper::PropArray props;
    nav_channel::TaskGoalPosition goal_pos;

    void navChannelCallback(const nav_channel::TaskStatus msg) {
        string status = "not started";
        if(msg.task.current_task = nav_channel::Task::NAVIGATION_CHANNEL) {
            // start task
            switch (status)
            {
            case "not started":
                status = "finding gate 1";
                break;
            
            case "finding gate 1":
                // if have two good props, ie. red on left, green on right, within 10
                // feet of each other, then go
                bool go = true;
                if (go) {
                    float x1 = 0;   //
                    float y1 = 0;   //
                    float z1 = 0;   // temp values, will get them from prop_mapper
                    float x2 = 5;   //
                    float y2 = 0;   //
                    float z2 = 0;   //

                    float midpoint_x = (x1+x2)/2;
                    float midpoint_y = (y1+y2)/2;
                    float midpoint_z = (z1+z2)/2;

                    setDestination(midpoint_x, midpoint_y, midpoint_z);

                    status = "travelling to wp1";
                };
                    // if we know the coords of both props, we calculate midpoint
                    // by doing x1 + x2 / 2 and y1 + y2 / 2

                    // when midpoint calculated, publish co-ords to /goal_position
                    // set goal_pos, then publish it ^^^
                break;

            case "travelling to wp1":
                // if global position == goal_position, within so much error
                bool complete = false;
                float error = 1; // can adjust for how close we want to be
                if (current_pos.pose.pose.position.x < goal_pos.goal_pose.position.x+error & current_pos.pose.pose.position.x > goal_pos.goal_pose.position.x-error) {
                    if (current_pos.pose.pose.position.y < goal_pos.goal_pose.position.y+error & current_pos.pose.pose.position.y > goal_pos.goal_pose.position.y-error) {
                        complete = true;
                    }
                }
                if (complete) {
                    status = "finding gate 2";
                };
                break;

            // should consider case where props are 100ft out, being out of lidar
            // range

            case "finding gate 2":
                // if have two good props, ie. red on left, green on right, within 10
                // feet of each other, then go
                bool go = true;
                if (go) {
                    float x1 = 0;   //
                    float y1 = 0;   // temp values, will get them from prop_mapper
                    float x2 = 5;   //
                    float y2 = 0;   //

                    float midpoint_x = (x1+x2)/2;
                    float midpoint_y = (y1+y2)/2;

                    setDestination(midpoint_x, midpoint_y);

                    status = "travelling to wp2";
                };
                    // so if we know the coords of both props, we calculate midpoint
                    // by doing x1 + x2 / 2 and y1 + y2 / 2

                    // when midpoint calculated, publish co-ords to /goal_position

                    // could consider going a bit past the waypoint
                break;

            case "travelling to wp2":
                // if global position == goal_position, within so much error
                bool complete = false;
                float error = 1; // can adjust for how close we want to be
                if (current_pos.pose.pose.position.x < goal_pos.goal_pose.position.x+error & current_pos.pose.pose.position.x > goal_pos.goal_pose.position.x-error) {
                    if (current_pos.pose.pose.position.x < goal_pos.goal_pose.position.x+error & current_pos.pose.pose.position.x > goal_pos.goal_pose.position.x-error) {
                        complete = true;
                    }
                }
                if (complete) {
                    status = "complete";
                };
                    // publish taskStatus == complete
                    // ^^ does this end the switch/case calls?

                break;

            default:
                break;
            }
        }
    }

    // prop_array is a list of props, which are themselves a
    // msg with a string (prop_label) and a Vector3 position
    void propMapCallback(const prop_mapper::PropArray msg) {
        props = msg;
    }

    void globalPositionCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {
        // set our current position
        current_pos = msg;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_channel_node");

    NavChannel nav_channel;

    // what else is needed here (wait4connect, wait4start, localframe, arm ?)

    nav_channel.spin();

    return 0;
}