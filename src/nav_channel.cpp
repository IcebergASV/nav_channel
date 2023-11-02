#include <ros/ros.h>
#include <nav_channel/TaskStatus.h>
#include <nav_channel/TaskGoalPosition.h>
#include <nav_channel/Task.h>
#include <string>
//#include <prop_mapper/PropArray.h> // isn't included yet
#include <nav_channel/PropArray.h> // fake prop map to replace prop_mapper for now
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

class NavChannel {
public:
    NavChannel(): nh("")
    {
        // task_to_exec will confirm we are on nav_channel, then execute task
        task_to_exec = nh.subscribe("task_to_execute", 10, &NavChannel::navChannelCallback, this);
        
        // props will get the prop map and store it
        prop_map = nh.subscribe("props", 10, &NavChannel::propMapCallback, this);

        // global_position/local grabs position from mavros and stores it
        global_pos = nh.subscribe("mavros/global_position/local", 10, &NavChannel::globalPositionCallback, this);

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

    void setDestination(geometry_msgs::PoseStamped midpoint) {
        // sets goal_pos and then publishes it
        goal_pos.goal_pose = midpoint;
        // will set orientation to (0,0,0,0), so probably want to grab current
        // orientation and set that to new orientation

        ROS_INFO_STREAM("Midpoint set at " << midpoint.pose.position.x << ","<< midpoint.pose.position.y << "," << midpoint.pose.position.z);

        task_goal_position.publish(goal_pos);
    }

    geometry_msgs::PoseStamped findMidpoint() {
        ROS_INFO("in findMidpoint.");
        // validates positions of props, then calculates midpoint, and returns
        // it as PoseStamped
        geometry_msgs::PoseStamped midpoint;

        bool red = false;   // use two booleans to determine if props exist
        bool green = false; // add math to determine if valid postions
        nav_channel::Prop red_prop;     // again fake props, change later
        nav_channel::Prop green_prop;

        for (int i = 0; !red || !green; i++) {
            if (props.props[i].prop_label == "Red Prop") {
                red = true;             // add math in these to check
                red_prop  = props.props[i];   // if green, is it close enough
            }
            if (props.props[i].prop_label == "Green Prop") {
                green = true;
                green_prop = props.props[i];
            }
        }
        if (red && green) {
            float red_x = red_prop.vector.x;
            float red_y = red_prop.vector.y;
            float red_z = red_prop.vector.z;
            float green_x = green_prop.vector.x;
            float green_y = green_prop.vector.y;
            float green_z = green_prop.vector.z;

            midpoint.pose.position.x = (red_x+green_x)/2;
            midpoint.pose.position.y = (red_y+green_y)/2;
            midpoint.pose.position.z = (red_z+green_z)/2;
        }
        ROS_INFO("returning midpoint.");

        return midpoint;
    }

    bool travelling() {
        ROS_INFO("travelling...");
        // check to see it we are at the goal (within a set amount of error)
        bool atDestination = true; // should be false, true for testing w/o movement
        float error = 1; // can adjust for how close we want to be
        // unsure about differences in PoseStamped and PoseWithCovarianceStamped
        if (current_pos.pose.pose.position.x < goal_pos.goal_pose.pose.position.x+error & current_pos.pose.pose.position.x > goal_pos.goal_pose.pose.position.x-error) {
            if (current_pos.pose.pose.position.y < goal_pos.goal_pose.pose.position.y+error & current_pos.pose.pose.position.y > goal_pos.goal_pose.pose.position.y-error) {
                if (current_pos.pose.pose.position.z < goal_pos.goal_pose.pose.position.z+error & current_pos.pose.pose.position.z > goal_pos.goal_pose.pose.position.z-error) {
                    atDestination = true;
                }
            }
        }

        return atDestination;
    }

private:

    ros::NodeHandle nh;
    ros::Subscriber task_to_exec;
    ros::Subscriber prop_map;
    ros::Subscriber global_pos;
    ros::Publisher task_status;
    ros::Publisher task_goal_position;

    nav_msgs::Odometry current_pos;
    //prop_mapper::PropArray props; temporarily replacing with fake props
    nav_channel::PropArray props;
    nav_channel::TaskGoalPosition goal_pos;

    int status = 0;

    void navChannelCallback(const nav_channel::TaskStatus msg) {
        //string status = "not started";
        // ^ does doing this prevent us from ever getting out of not_started?
        if(msg.task.current_task == nav_channel::Task::NAVIGATION_CHANNEL) {
            // start task
            nav_channel::TaskStatus taskStatus;
            taskStatus.status = nav_channel::TaskStatus::IN_PROGRESS;
            task_status.publish(taskStatus);

            switch (status)
            {
            case 0: {
                ROS_INFO("in not started case.");
                status = 1;
                }
                break;
            
            case 1: {
                // if have two good props, ie. red on left, green on right, within 10
                // feet of each other, then go (say 11ft to account for error)

                ROS_INFO("start task.");
                geometry_msgs::PoseStamped midpoint = findMidpoint();
                setDestination(midpoint);

                status = 2;
                }
                break;

            case 2: {
                // if global position == goal_position, within so much error

                bool complete = travelling();
                
                if (complete) {
                    status = 3;
                };
                }
                break;

            // should consider case where props are 100ft out, being out of lidar
            // range

            case 3: {
                
                ROS_INFO("at gate 1.");
                geometry_msgs::PoseStamped midpoint = findMidpoint();
                setDestination(midpoint);

                status = 4;
                }
                break;

            case 4: {
                
                bool complete = travelling();
                
                if (complete) {
                    status = 5;
                };
                }
                break;

            case 5: {
                // publish taskStatus == complete
                taskStatus.status = nav_channel::TaskStatus::COMPLETE;
                ROS_INFO("at gate 2");
                ROS_INFO("TASK COMPLETE");
                task_status.publish(taskStatus);

                // ^^ does this end the switch/case calls?
                }
                break;

            default:
                break;
            }
        }
    }

    // prop_array is a list of props, which are themselves a
    // msg with a string (prop_label) and a Vector3 position
    void propMapCallback(const nav_channel::PropArray msg) {
        props = msg;
    }

    void globalPositionCallback(const nav_msgs::Odometry msg) {
        // set our current position
        current_pos = msg;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_channel_node");

    NavChannel nav_channel;

    nav_channel.spin();

    return 0;
}