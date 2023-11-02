#include <ros/ros.h>
#include <nav_channel/PropArray.h>
#include <nav_channel/Prop.h>
#include <nav_channel/TaskStatus.h>

// ALSO INCLUDES FAKE TASK MANAGER

int main(int argc, char** argv) {
	ros::init(argc, argv, "fake_prop_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<nav_channel::PropArray>("props", 1000);
    ros::Publisher task_pub = nh.advertise<nav_channel::TaskStatus>("task_to_execute", 1000);
    ros::Rate loop_rate(2);

    nav_channel::Prop red_prop;
    red_prop.prop_label = "Red Prop";
    red_prop.vector.x = 10;     //
    red_prop.vector.y = 0;      // red prop at (10, 0, 10)
    red_prop.vector.z = 10;     //

    nav_channel::Prop green_prop;
    green_prop.prop_label = "Green Prop";
    green_prop.vector.x = 10;   //
    green_prop.vector.y = 0;    // green prop at (10, 0, 0)
    green_prop.vector.z = 0;    //

    nav_channel::PropArray prop_array;
    prop_array.props.push_back(red_prop);
    prop_array.props.push_back(green_prop);

    nav_channel::TaskStatus task;
    task.task.current_task = 1;

    while (ros::ok())
    {
        pub.publish(prop_array);
        task_pub.publish(task);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}