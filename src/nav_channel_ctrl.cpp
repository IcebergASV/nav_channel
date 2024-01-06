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
    NavChannel(): nh_(""), private_nh_("~")
    {
        // ROS parameters
        private_nh_.param<double>("error", error, 1.0); // leniency for if we are at the gate
        private_nh_.param<double>("gate_max_dist", gate_max_dist, 8.0); // max distance for gate 1, determines if we are looking at the right gate, 8.0m is roughly 25ft, can make more accurate if needed.
        private_nh_.param<double>("gate_max_width", gate_max_width, 4.0); // max distance between two buoys for a gate, confirming we have the two correct ones. 4.0m is roughly 13ft, accounts for max gap and diameter.
        private_nh_.param<std::string>("red_marker", red_marker, "red_marker");
        private_nh_.param<std::string>("green_marker", green_marker, "red_marker");

        // ROS subscribers
        private_nh_.param<std::string>("local_pose_topic", local_pose_topic_, "/mavros/local_position/pose");

        // task_to_exec will confirm we are on nav_channel, then execute task
        task_to_exec_ = nh_.subscribe("task_to_execute", 10, &NavChannel::navChannelCallback, this);
        
        // props will get the prop map and store it
        prop_map_ = nh_.subscribe("/prop_array", 10, &NavChannel::propMapCallback, this);

        // global_position/local grabs position from mavros and stores it
        global_pos_ = nh_.subscribe(local_pose_topic_, 10, &NavChannel::globalPositionCallback, this);


        // ROS publishers

        // we publish to task_status to update us on progress of task
        task_status_ = nh_.advertise<task_master::TaskStatus>("task_status", 10);

        // we publish to task_goal_position to send the boat to a waypoint
        task_goal_position_ = nh_.advertise<task_master::TaskGoalPosition>("task_goal_position", 10);
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    void setDestination(geometry_msgs::Point midpoint) {
        // sets goal_pos and then publishes it
        goal_pos_.point = midpoint;
        goal_pos_.task.current_task = 1; // sets task to nav_channel for message filtering
        // does this set orientation to (0,0,0,0), do we need to grab current orientation and set that to new orientation?

        ROS_INFO_STREAM("Midpoint set at " << midpoint.x << ","<< midpoint.y << "," << midpoint.z);

        task_goal_position_.publish(goal_pos_);
    }

    enum Colour {
        RED,
        GREEN,
        BLUE
    };

    /**
     * TODO
    */
    bool isValidMarker(prop_mapper::Prop marker, Colour colour)
    {
        bool valid = false;
        if (marker.prop_label == "red_marker" && colour = Colour::RED) {
            valid = true;
        }
        if (marker.prop_label == "green_marker" && colour = Colour::GREEN) {
            valid = true;
        }

        return valid;
    }

    /**
     * TODO
    */
    bool isValidGate(prop_mapper::Prop red_marker, prop_mapper::Prop green_marker)
    {
        bool valid = true;
        double dist = sqrt(pow((red_marker.vector.x - green_marker.vector.x), 2) + pow((red_marker.vector.y - green_marker.vector.y), 2));
        if (dist > gate_max_width) {
            valid = false;
        }

        double red_polar_angle = findPolarAngle(red_marker);
        double green_polar_angle = findPolarAngle(green_marker);

        if (abs(green_polar_angle-red_polar_angle) > 180) {
            if (red_polar_angle > green_polar_angle) {
                valid = false;
            }
        }
        else if (green_polar_angle > red_polar_angle) {
            valid = false;
        }

        return valid;
    }

    double findPolarAngle(prop_mapper::Prop marker) {
        double angle;
        if (marker.vector.x > 0 && marker.vector.y > 0) {
            angle = tan(y, x);
        }
        else if (marker.vector.x < 0 && marker.vector.y > 0) {
            angle = tan(x, y) + M_PI_2;
        }
        else if (marker.vector.x < 0 && marker.vector.y < 0) {
            angle = tan(y, x) + M_PI;
        }
        else {
            angle = tan(x, y) + M_PI + M_PI_2;
        }

        return angle;
    }

    bool findGate(int gate_to_find, prop_mapper::Prop &green_marker, prop_mapper::Prop &red_marker)
    {
        prop_mapper::Prop temp_green;
        prop_mapper::Prop temp_red;

        bool green_found = false;
        bool red_found = false;
        bool gate_found = false;

        int i = 0;
        // Iterate through the prop array
        while ((i<sizeof(props_.props)) || gate_found)
        {
            if (isValidMarker(props_.props[i], RED))
            {
                temp_red = props_.props[i];
                red_found = true;
            }
            else if (isValidMarker(props_.props[i], GREEN))
            {
                temp_green = props_.props[i];
                green_found = true;
            }

            if (green_found && red_found)
            {
                if (isValidGate(temp_red, temp_green))
                {
                    red_marker = temp_red;
                    green_marker = temp_green;
                    gate_found == true;
                }
            }

            i++;
        }

        return gate_found;
    }


    // TODO - change name to "findMidpoint" after old findMindpoint removed
    geometry_msgs::Point findMidpoint(prop_mapper::Prop marker1, prop_mapper::Prop marker2)
    {
        geometry_msgs::Point midpnt;
        midpnt.x = (marker1.vector.x+marker2.vector.x)/2;
        midpnt.y = (marker1.vector.y+marker2.vector.y)/2;
        midpnt.z = 0;
    }



    /**
     * TODO - remove this function and switch usage to the above
    */
    geometry_msgs::Point old_findMidpoint(int gate) {
        // validates positions of props, then calculates midpoint, and returns
        // it as Point
        geometry_msgs::Point midpoint;


        bool red = false;   // use two booleans to determine if props exist
        bool green = false;
        prop_mapper::Prop red_prop;
        prop_mapper::Prop green_prop;

        ROS_DEBUG_STREAM("Gate #" << gate);
        if (props_.props.size() > 0)
        {
            for (int i = 0; (!red || !green) || i >= sizeof(props_.props) ; i++) {
                ROS_INFO("here 1");
                ROS_DEBUG_STREAM("Prop array " << props_);

                if (props_.props[i].prop_label == "red_marker" || props_.props[i].prop_label == "green_marker" || true) {
                
                    ROS_INFO("here 2");
                    double dist_to_gate = sqrt(pow(props_.props[i].vector.x - current_pos_.pose.position.x, 2) + pow(props_.props[i].vector.y - current_pos_.pose.position.y, 2));
                    if ((gate == 1 && dist_to_gate <= gate_max_dist) || (gate == 2 && dist_to_gate > gate_max_dist)) {
                    // using 8m as roughly 25ft
                    // gate 1 should be within 25ft, gate 2 should be at least 25ft away
                        ROS_INFO("here 3");
                        if (props_.props[i].prop_label == red_marker) {
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

                        if (props_.props[i].prop_label == green_marker) {
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
                    else 
                    {
                        ROS_INFO("Gate not within max distance.");
                    }
                }
            }
        }
        else
        {
            ROS_INFO_STREAM(TAG << "No props in prop_array");
        }

            
        if (red && green) {
            ROS_INFO("here 8");
            float red_x = red_prop.vector.x;
            float red_y = red_prop.vector.y;
            //float red_z = red_prop.vector.z;
            float green_x = green_prop.vector.x;
            float green_y = green_prop.vector.y;
            //float green_z = green_prop.vector.z;

            midpoint.x = (red_x+green_x)/2;
            midpoint.y = (red_y+green_y)/2;
            //midpoint.z = (red_z+green_z)/2;
            midpoint.z = 0;
        }

        else {
            ROS_INFO("gates were not found.");
        }

        ROS_DEBUG("returning midpoint.");

        return midpoint;
    }

    bool isReached() {
        ROS_DEBUG("checking if reaching wp");
        // check to see it we are at the goal (within a set amount of error)
        bool atDestination = false;
        if (current_pos_.pose.position.x < goal_pos_.point.x+error & current_pos_.pose.position.x > goal_pos_.point.x-error) {
            if (current_pos_.pose.position.y < goal_pos_.point.y+error & current_pos_.pose.position.y > goal_pos_.point.y-error) {
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

    std::string local_pose_topic_;

    geometry_msgs::PoseStamped current_pos_;
    prop_mapper::PropArray props_; //temporarily replacing with fake props
    task_master::TaskGoalPosition goal_pos_;

    std::string TAG = "NAV_CHANNEL_CTRL: ";

    double error;
    double gate_max_dist;
    double gate_max_width;
    std::string red_marker;
    std::string green_marker;
    enum states {not_started, find_wp1, moving_to_wp1, find_wp2, moving_to_wp2, complete};
    states status = states::not_started;

    void navChannelCallback(const task_master::Task msg) {
        ROS_DEBUG_STREAM(TAG << "APPLE");
        if(msg.current_task == task_master::Task::NAVIGATION_CHANNEL) {
            ROS_DEBUG_STREAM(TAG << "ORANGE");
            // start task
            task_master::TaskStatus taskStatus;
            taskStatus.status = task_master::TaskStatus::IN_PROGRESS;
            task_status_.publish(taskStatus);

            switch (status)
            {
            case states::not_started: {
                ROS_DEBUG_STREAM(TAG << "GRADE");
                ROS_DEBUG("in not started case.");

                taskStatus.status = task_master::TaskStatus::IN_PROGRESS;
                taskStatus.task.current_task = task_master::Task::NAVIGATION_CHANNEL;
                task_status_.publish(taskStatus);

                status = states::find_wp1;
                }
                break;
            
            case states::find_wp1: {
                ROS_DEBUG_STREAM(TAG << "Peach");
                // if have two good props, ie. red on left, green on right, within 10 feet of each other, then go

                ROS_INFO("start task.");

                prop_mapper::Prop green_marker;
                prop_mapper::Prop red_marker;
                if (findGate(1, green_marker, red_marker)) {
                    geometry_msgs::Point midpoint = findMidpoint(green_marker, red_marker);

                    setDestination(midpoint);
                    ROS_DEBUG_STREAM(TAG << "about to check if midpoint reached");
                    if(!isReached())
                    {
                        ROS_DEBUG_STREAM(TAG << "Midpoint 1 not reached yet");
                        setDestination(midpoint);
                        ros::Rate rate(10);
                        rate.sleep();
                    }
                
                    if (isReached()) {
                        status = states::find_wp2;
                        ROS_DEBUG_STREAM(TAG << "midpoint 1 reached");
                    };
                }
                
                
                }
                break;
            // should consider case where props are 100ft out, being out of lidar range

            case states::find_wp2: {
                 ROS_DEBUG_STREAM(TAG << "PEAR");
                ROS_DEBUG("at gate 1.");
                geometry_msgs::Point midpoint = findMidpoint(2);
                setDestination(midpoint);
                
                prop_mapper::Prop green_marker;
                prop_mapper::Prop red_marker;
                if (findGate(2, green_marker, red_marker)) {
                    geometry_msgs::Point midpoint = findMidpoint(green_marker, red_marker);

                    setDestination(midpoint);
                    ROS_DEBUG_STREAM(TAG << "about to check if midpoint reached");
                    if(!isReached())
                    {
                        ROS_DEBUG_STREAM(TAG << "Midpoint 1 not reached yet");
                        setDestination(midpoint);
                        ros::Rate rate(10);
                        rate.sleep();
                    }
                
                    if (isReached()) {
                        status = states::find_wp2;
                        ROS_DEBUG_STREAM(TAG << "midpoint 1 reached");
                    };
                }
            }
                break;

            case states::complete: {
                ROS_DEBUG_STREAM(TAG << "hfhhhhhhh");
                taskStatus.status = task_master::TaskStatus::COMPLETE;
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

    void globalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // set our current position
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