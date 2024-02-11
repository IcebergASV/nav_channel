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
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/State.h>

class NavChannel {
public:
    NavChannel(): nh_(""), private_nh_("~"), green_id_(-1), red_id_(-1), reached_count_(0)
    {
        // ROS parameters
        private_nh_.param<double>("error", wp_error_tolerance, 1.0); // Tolerance radius for determining if asv reached gate waypoint
        private_nh_.param<double>("gate_max_dist", gate_max_dist, 8.0); // max distance between asv and gate 1 at start, determines if we are looking at the right gate
        private_nh_.param<double>("gate_max_width", gate_max_width, 4.0); // max distance between two buoys in gate, accounts for max gap and diameter
        private_nh_.param<double>("dist_past_second_gate", dist_past_second_gate, 2.0);
        private_nh_.param<double>("dist_to_est_gate_2", dist_to_est_gate_2, 2.0);
        private_nh_.param<double>("dist_from_gate", dist_from_gate, 1.0);
        private_nh_.param<bool>("color_blind", colour_blind_p, true);

        private_nh_.param<bool>("use_pixawk_reached", use_pixawk_reached_p, 2.0);
        private_nh_.param<bool>("freq_disc_mode", freq_disc_mode_p, 2.0);
        private_nh_.param<int>("freq_disc_count", freq_disc_count_p, 1.0);
        private_nh_.param<double>("freq", freq_p, 1.0);

        count_ = freq_disc_count_p; //start with timer disabled


        private_nh_.param<std::string>("red_marker", red_marker_str, "red_marker");
        private_nh_.param<std::string>("green_marker", green_marker_str, "green_marker");

        // ROS subscribers
        private_nh_.param<std::string>("local_pose_topic", local_pose_topic_, "/mavros/local_position/pose");

        // task_master dictates which task is is be executed
        task_to_exec_ = nh_.subscribe("task_to_execute", 10, &NavChannel::navChannelCallback, this);

        prop_map_ = nh_.subscribe("/prop_array", 10, &NavChannel::propMapCallback, this);
        
        global_pos_ = nh_.subscribe(local_pose_topic_, 10, &NavChannel::localPositionCallback, this);

        wp_reached_sub_ = nh_.subscribe("/mavros/mission/reached", 10, &NavChannel::missionReachedCallback, this);

        state_sub_ = nh_.subscribe("/mavros/state", 10, &NavChannel::state_cb, this);

        // ROS publishers

        // publishes task progress to /task_status
        pub_task_status_ = nh_.advertise<task_master::TaskStatus>("task_status", 10);

        // publishes desired positions to /task_goal_position to move asv
        task_goal_position_ = nh_.advertise<task_master::TaskGoalPosition>("task_goal_position", 10);

        // Set publishing rate
        publish_timer_ = nh_.createTimer(ros::Duration(1.0 /freq_p), &NavChannel::publishTimerCallback, this);
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    void setInitialStatus()
    {
        task_status_.task.current_task = task_master::Task::NAVIGATION_CHANNEL; 
        task_status_.status = task_master::TaskStatus::NOT_STARTED;
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber task_to_exec_;
    ros::Subscriber prop_map_;
    ros::Subscriber global_pos_;
    ros::Subscriber state_sub_;
    ros::Subscriber wp_reached_sub_;
    ros::Publisher pub_task_status_;
    ros::Publisher task_goal_position_;
    ros::Timer publish_timer_;

    task_master::TaskStatus task_status_;

    std::string local_pose_topic_;

    geometry_msgs::PoseStamped current_pos_;
    prop_mapper::PropArray props_; //temporarily replacing with fake props
    task_master::TaskGoalPosition goal_pos_;
    geometry_msgs::Point est_gate_2_; //estimated gate 2 until we find the actual gate 2
    geometry_msgs::Point before_gate_; // holds a point before the gate
    geometry_msgs::Point after_gate_;

    std::string TAG = "NAV_CHANNEL_CTRL: ";

    mavros_msgs::State current_state_;
    mavros_msgs::State prev_state_;
    double wp_error_tolerance;
    double gate_max_dist;
    double gate_max_width;
    double dist_past_second_gate;
    double dist_to_est_gate_2;
    int reached_count_;
    double dist_from_gate;
    bool use_pixawk_reached_p;
    bool freq_disc_mode_p;
    bool colour_blind_p;
    int freq_disc_count_p;
    double freq_p;
    int count_;
    double is_reached_;
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

    enum States {NOT_STARTED, FIND_GATE1, MOVE_BEFORE_GATE1, MOVE_AFTER_GATE1, FIND_GATE2, MOVE_BEFORE_GATE2, MOVE_AFTER_GATE2, COMPLETE}; 

    States status = States::NOT_STARTED;

    // for controlled frequency
    void publishTimerCallback(const ros::TimerEvent&) {
        if (freq_disc_mode_p && count_ < freq_disc_count_p && current_state_.mode == "GUIDED"){
        
            // Publish setpoint
            setDestination();
            count_++;
        }
    }

    void resetTimer()
    {
        count_ = 0;
    }

    void stopTimer()
    {
        count_ = freq_disc_count_p;
    }


    void setDestination() 
    {
        ROS_DEBUG_STREAM(TAG << "setDestination() called");
        goal_pos_.task.current_task = task_master::Task::NAVIGATION_CHANNEL; 
        
        ROS_DEBUG_STREAM(TAG << "Setpoint x: " << goal_pos_.point.x << ", y: "<< goal_pos_.point.y);

        task_goal_position_.publish(goal_pos_);
    }

    bool isValidMarker(prop_mapper::Prop marker, Colour colour)
    {
        ROS_DEBUG_STREAM(TAG << "isValidMarker() called");
        bool valid = false;
        if (marker.prop_label == red_marker_str && colour == Colour::RED && marker.id != red_id_) {
            ROS_DEBUG_STREAM(TAG << "Valid red marker found");
            valid = true;
        }
        if (marker.prop_label == green_marker_str && colour == Colour::GREEN && marker.id != green_id_) {
            ROS_DEBUG_STREAM(TAG << "Valid green marker found");
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

    double calcHypotenuse(prop_mapper::Prop prop) {
        // Calculate the square of the lengths of the catheti
        double x = prop.point.x;
        double y = prop.point.y;
        double x_squared = x * x;
        double y_squared = y * y;

        // Calculate the square of the length of the hypotenuse
        double hypotenuse_squared = x_squared + y_squared;

        // Calculate the length of the hypotenuse by taking the square root
        double hypotenuse = std::sqrt(hypotenuse_squared);

        return hypotenuse;
    }

    double calcDistance(geometry_msgs::Point pnt1, geometry_msgs::Point pnt2 ) {
        double x1 = pnt1.x;
        double y1 = pnt1.y;
        double x2 = pnt2.x;
        double y2 = pnt2.y;
        double dx = x2 - x1;
        double dy = y2 - y1;

        double distance = std::sqrt(dx * dx + dy * dy);

        return distance;
    }
    
    bool findGateGreyScale(prop_mapper::Prop &green_marker, prop_mapper::Prop &red_marker)
    {
        ROS_DEBUG_STREAM(TAG << "findGate() called");


        int green_idx;
        int red_idx;

        //bool green_found = false;
        //bool red_found = false;
        bool gate_found = false;

        //int i = 0;

        std::vector<prop_mapper::Prop> marker_arr;

        // pull out markers
        for (int n =0; n< props_.props.size(); n++)
        {
            ROS_DEBUG_STREAM(TAG << "Pull out markers from array");
            if ( props_.props[n].prop_label == "red_marker" || props_.props[n].prop_label == "green_marker" || props_.props[n].prop_label == "marker" && props_.props[n].id != green_id_ && props_.props[n].id != red_id_  )
            {
                ROS_DEBUG_STREAM(TAG  << "marker added to marker array");
                marker_arr.push_back(props_.props[n]);
            }
        }

        // get closest marker
        prop_mapper::Prop closest_marker;
        closest_marker.point.x = 1000;
        closest_marker.point.y = 1000;
        int closest_idx = 0;
        bool can_erase = false;

        for ( int i = 0; i < marker_arr.size(); i++)
        {
            if ( calcHypotenuse(marker_arr[i]) < calcHypotenuse(closest_marker))
            {
                closest_marker = marker_arr[i];
                closest_idx = i;
                can_erase = true;
                
            }
        }

        //if (can_erase)
        //{
        //    marker_arr.erase(marker_arr.begin()+closest_idx); // remove closest element
        //}
        

        while(marker_arr.size() > 0 && !gate_found)
        {
            for ( int j = 0; j < marker_arr.size(); j++)
            {
                if ( calcDistance(closest_marker.point, marker_arr[j].point) < gate_max_width  && calcDistance(closest_marker.point, current_pos_.pose.position) < gate_max_dist )
                {
                    //valid gate found
                    green_marker = closest_marker;
                    red_marker = marker_arr[j];
                    green_id_ = closest_marker.id;
                    red_id_ = marker_arr[j].id;
                    gate_found = true;
                    closest_idx = j;
                }
                else
                {
                    ROS_DEBUG_STREAM(TAG << "maker not in gate");
                    //marker_arr.erase(marker_arr.begin()+closest_idx); // marker does not meet criteria - remove it
                }
            }
        }
        
        if (gate_found)
        {
            return true;
        }
        return false;

    }

    bool findGateWithColours(prop_mapper::Prop &green_marker, prop_mapper::Prop &red_marker)
    {
        ROS_DEBUG_STREAM(TAG << "findGate with colours called");

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

    bool findGate(prop_mapper::Prop &green_marker, prop_mapper::Prop &red_marker)
    {
        if (colour_blind_p)
        {
            return findGateGreyScale(green_marker, red_marker);
        }
        return findGateWithColours(green_marker, red_marker);
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
    
    geometry_msgs::Point findEndpoint(geometry_msgs::Point marker_1, geometry_msgs::Point midpnt, double dist)
    {
        ROS_DEBUG_STREAM(TAG << "findEndpoint() called");
        // Translate the a marker so we can get the rotation angle 
        marker_1 = translatePoint(marker_1, -midpnt.x, -midpnt.y);

        // Get rotation angle to align marker on x axis
        double angle = std::atan2(marker_1.y, marker_1.x);// get the amount to rotate by
        
        geometry_msgs::Point endpnt;
        endpnt.x = 0;
        endpnt.y = dist;

        endpnt = rotatePoint(endpnt, angle);

        endpnt = translatePoint(endpnt, midpnt.x, midpnt.y);

        ROS_DEBUG_STREAM(TAG << "Point set " << dist << "m past the gate at x: " << endpnt.x << ", y: " << endpnt.y );

        return endpnt;
    }

    geometry_msgs::Point findMidpoint(prop_mapper::Prop marker1, prop_mapper::Prop marker2)
    {
        geometry_msgs::Point midpnt;
        ROS_DEBUG_STREAM(TAG << "m1 x = " << marker1.point.x << ", m1 y = " << marker1.point.y);
        ROS_DEBUG_STREAM(TAG << "m2 x = " << marker2.point.x << ", m2 y = " << marker2.point.y);
        midpnt.x = (marker1.point.x+marker2.point.x)/2;
        midpnt.y = (marker1.point.y+marker2.point.y)/2;
        midpnt.z = 0;

        ROS_DEBUG_STREAM(TAG << "Midpoint x: " << midpnt.x << ", y: " << midpnt);
        return midpnt;
    }

    bool isReached() {
        //ROS_DEBUG_STREAM(TAG << "isReached() called");
        bool atDestination = false;
        if(use_pixawk_reached_p)
        {
            ROS_DEBUG_STREAM(TAG << "Using pixhawk for waypoint reached signal");
            atDestination = is_reached_;
        }
        else
        {
            ROS_DEBUG_STREAM(TAG << "Using current local pose for waypoint reached signal");
            // check to see it we are at the goal (within a set amount of error)
            
            ROS_DEBUG_STREAM(TAG << "wp_error_tolerance: " << wp_error_tolerance);
            ROS_DEBUG_STREAM(TAG << "isReached current x: " << current_pos_.pose.position.x << " goal x: " << goal_pos_.point.x << "current y:" <<  current_pos_.pose.position.y << " goal y: " << goal_pos_.point.y);
            if (current_pos_.pose.position.x < goal_pos_.point.x+wp_error_tolerance & current_pos_.pose.position.x > goal_pos_.point.x-wp_error_tolerance) {
                if (current_pos_.pose.position.y < goal_pos_.point.y+wp_error_tolerance & current_pos_.pose.position.y > goal_pos_.point.y-wp_error_tolerance) {
                    atDestination = true;
                    ROS_INFO_STREAM(TAG << "Goal position reached");
                }
            }
        }
        return atDestination;
    }

    void holdPose()
    {
        ROS_DEBUG_STREAM(TAG << "holdPose()");
        // set goal pose to current pose
        goal_pos_.point = current_pos_.pose.position;
        ROS_DEBUG_STREAM(TAG << "Holding position, goal pos set to x: " << current_pos_.pose.position.x << ", y: " << current_pos_.pose.position.y );
        //setDestination();
        return;
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
        
        current_state_ = *msg;
        if (prev_state_.mode == "MANUAL" && current_state_.mode == "GUIDED")
        {
            reached_count_ = 0;
        }
        prev_state_ = current_state_;
    }

    void missionReachedCallback(const mavros_msgs::WaypointReached msg)
    {
        ROS_INFO_STREAM(TAG << "reached count: " << reached_count_);
        if(current_state_.mode == "GUIDED" )//&& reached_count_ == freq_disc_count_p-1)
        {            
            ROS_INFO_STREAM(TAG << "Setpoint reached!");
            is_reached_ = true;
            reached_count_ = 0;
            return;
        }
        reached_count_++;
    }

    void navChannelCallback(const task_master::Task msg) {
        if(msg.current_task == task_master::Task::NAVIGATION_CHANNEL) {
            
            switch (status)
            {
            case States::NOT_STARTED: 
            {
                ROS_INFO_STREAM(TAG << "waiting for at least 2 markers");

                task_status_.status = task_master::TaskStatus::IN_PROGRESS;

                if (props_.props.size() >= 2)
                {
                    ROS_INFO_STREAM(TAG << "At least 2 markers detected, moving on to find gate 1");
                    status = States::FIND_GATE1;
                    is_reached_ = false;
                }
                break;
            }
            case States::FIND_GATE1: {
                // if have two good props, ie. red on left, green on right, within acceptable distance of each other, then go

                ROS_INFO_STREAM(TAG << "Looking for the 1st gate");

                //holdPose(); // maintain current position
                
                prop_mapper::Prop green_marker; // TODO change to Prop ID
                prop_mapper::Prop red_marker; // TODO change to Prop ID
                
                if (findGate(green_marker, red_marker)) 
                {
                    geometry_msgs::Point midpnt = findMidpoint(green_marker, red_marker);
                    before_gate_ = findEndpoint(green_marker.point, midpnt, -dist_from_gate); 
                    after_gate_ = findEndpoint(green_marker.point, midpnt, dist_from_gate); 
                    est_gate_2_ = findEndpoint(green_marker.point, midpnt, dist_to_est_gate_2); 
                    green_id_ = green_marker.id;
                    red_id_ = red_marker.id;
                    status = States::MOVE_BEFORE_GATE1;
                    goal_pos_.point = before_gate_;
                    ROS_INFO_STREAM(TAG << "Moving to point before first gate at x: " << goal_pos_.point.x << " ,y: " << goal_pos_.point.y); //TODO add angle to the midpoint
                    if ( freq_disc_mode_p ) { resetTimer();}  
              
                }
                }
                break;
            case States::MOVE_BEFORE_GATE1: {
                
                if(!isReached())
                {
                    ROS_DEBUG_STREAM(TAG << "Point before 1st gate not reached yet");
                    if (! freq_disc_mode_p) {setDestination();}
                    ros::Rate rate(10);
                    rate.sleep();
                }
            
                if (isReached()) {
                    is_reached_ = false;
                    if ( freq_disc_mode_p ) { resetTimer();}  
                    status = States::MOVE_AFTER_GATE1;
                    ROS_INFO_STREAM(TAG << "Point before 1st gate reached");
                    ROS_INFO_STREAM(TAG << "Moving past gate 1");
                    goal_pos_.point = after_gate_;
                    if ( freq_disc_mode_p ) { resetTimer();}

                };
                }
                break;

            case States::MOVE_AFTER_GATE1: {
                
                if(!isReached())
                {
                    ROS_DEBUG_STREAM(TAG << "Point after gate 1 not reached yet");
                    if (! freq_disc_mode_p) {setDestination();}
                    ros::Rate rate(10);
                    rate.sleep();
                }
            
                if (isReached()) {
                    is_reached_ = false;
                    status = States::FIND_GATE2;
                    ROS_INFO_STREAM(TAG << "Gate 1 passed");
                    ROS_INFO_STREAM(TAG << "Moving forwards " << dist_to_est_gate_2 << " m until gate 2 identified");
                    

                };
                }
                break;
            // should consider case where props are 100ft out, being out of lidar range

            case States::FIND_GATE2: {               

                // move forwards until we find second gate
                goal_pos_.point = est_gate_2_;
                
                
                prop_mapper::Prop green_marker; // TODO change to Prop ID
                prop_mapper::Prop red_marker; // TODO change to Prop ID                
                
                if (findGate(green_marker, red_marker)) 
                {
                    ROS_INFO_STREAM(TAG << "Gate 2 identified");
                    geometry_msgs::Point midpnt = findMidpoint(green_marker, red_marker);
                    before_gate_ = findEndpoint(green_marker.point, midpnt, -dist_from_gate); 
                    after_gate_ = findEndpoint(green_marker.point, midpnt, dist_from_gate); 
                    goal_pos_.point = before_gate_;//findEndpoint(green_marker.point, midpnt, dist_to_est_gate_2); // extends the midpoint to actually pass through the gate
                    ROS_INFO_STREAM(TAG << "Moving before 2nd gate at x: " << goal_pos_.point.x << " ,y: " << goal_pos_.point.y);
                    status = States::MOVE_BEFORE_GATE2;
                    if ( freq_disc_mode_p ) { resetTimer();}  
     
                }
                }
                break;

            case States::MOVE_BEFORE_GATE2: {

                if(!isReached())
                {
                    ROS_DEBUG_STREAM(TAG << "Point before gate 2 not reached yet");
                    if ( !freq_disc_mode_p ) { setDestination();}  

                    ros::Rate rate(10);
                    rate.sleep();
                }
            
                if (isReached()) {
                    is_reached_ = false;
                    goal_pos_.point = after_gate_;
                    status = States::MOVE_AFTER_GATE2;
                    ROS_DEBUG_STREAM(TAG << "Point before gate 2 reached");
                    ROS_INFO_STREAM(TAG << "Moving past gate 2");
                    if ( freq_disc_mode_p ) { resetTimer();}  

                };
                }
                break;
            case States::MOVE_AFTER_GATE2: {

                if(!isReached())
                {
                    ROS_DEBUG_STREAM(TAG << "Point past gate 2 not reached yet");
                    if (!freq_disc_mode_p) {setDestination();}
                    ros::Rate rate(10);
                    rate.sleep();
                }
            
                if (isReached()) {
                    is_reached_ = false;
                    status = States::COMPLETE;
                    ROS_DEBUG_STREAM(TAG << "Passed gate 2");
                    ROS_INFO_STREAM(TAG << "Navigation Channel Complete");
                    if(freq_disc_mode_p) {stopTimer();}

                };
                }
                break;

            case States::COMPLETE: {
                task_status_.status = task_master::TaskStatus::COMPLETE;
                }
                break;

            default:
                break;
            }
            
        }
        pub_task_status_.publish(task_status_);
    
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
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();

    NavChannel nav_channel;

    nav_channel.setInitialStatus();

    nav_channel.spin();

    return 0;
}