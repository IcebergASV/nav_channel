#include <ros/ros.h>
//#include <nav_channel/PropArray.h>
//#include <nav_channel/Prop.h>
#include <prop_mapper/PropArray.h>
#include <prop_mapper/Prop.h>
#include <nav_channel/TaskStatus.h>

// ALSO INCLUDES FAKE TASK MANAGER

int main(int argc, char** argv) {
	ros::init(argc, argv, "fake_prop_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<prop_mapper::PropArray>("props", 1000);
    ros::Publisher task_pub = nh.advertise<nav_channel::TaskStatus>("task_to_execute", 1000);
    ros::Rate loop_rate(2);

    // TEST PROP SET 1

    /*nav_channel::Prop red_prop_g1;
    red_prop_g1.prop_label = "Red Prop";
    red_prop_g1.vector.x = -1.25;  //
    red_prop_g1.vector.y = 5;      // red prop at (-1.25, 5, 0)
    red_prop_g1.vector.z = 0;      //

    nav_channel::Prop green_prop_g1;
    green_prop_g1.prop_label = "Green Prop";
    green_prop_g1.vector.x = 1.25; //
    green_prop_g1.vector.y = 5;    // green prop at (1.25, 5, 0)
    green_prop_g1.vector.z = 0;    //

    nav_channel::Prop red_prop_g2;
    red_prop_g2.prop_label = "Red Prop";
    red_prop_g2.vector.x = -1.4;   //
    red_prop_g2.vector.y = 25;     // red prop at (-1.4, 25, 0)
    red_prop_g2.vector.z = 0;      //

    nav_channel::Prop green_prop_g2;
    green_prop_g2.prop_label = "Green Prop";
    green_prop_g2.vector.x = 1;    //
    green_prop_g2.vector.y = 24;   // green prop at (1, 24, 0)
    green_prop_g2.vector.z = 0;    //

    nav_channel::PropArray prop_array;
    prop_array.props.push_back(red_prop_g1);
    prop_array.props.push_back(green_prop_g1);
    prop_array.props.push_back(red_prop_g2);
    prop_array.props.push_back(green_prop_g2);*/

    // TEST PROP SET 2

    prop_mapper::Prop red_prop_g1;
    red_prop_g1.prop_label = "Red Prop";
    red_prop_g1.vector.x = 1;       //
    red_prop_g1.vector.y = 5;       // red prop at (-1.25, 5, 0)
    red_prop_g1.vector.z = 0;       //

    prop_mapper::Prop green_prop_g1;
    green_prop_g1.prop_label = "Green Prop";
    green_prop_g1.vector.x = 2;     //
    green_prop_g1.vector.y = 3.5;   // green prop at (1.25, 5, 0)
    green_prop_g1.vector.z = 0;     //

    prop_mapper::Prop red_prop_g2;
    red_prop_g2.prop_label = "Red Prop";
    red_prop_g2.vector.x = 2;       //
    red_prop_g2.vector.y = 25;      // red prop at (-1.4, 25, 0)
    red_prop_g2.vector.z = 0;       //

    prop_mapper::Prop green_prop_g2;
    green_prop_g2.prop_label = "Green Prop";
    green_prop_g2.vector.x = 3.5;   //
    green_prop_g2.vector.y = 24;    // green prop at (1, 24, 0)
    green_prop_g2.vector.z = 0;     //

    prop_mapper::PropArray prop_array;
    prop_array.props.push_back(red_prop_g1);
    prop_array.props.push_back(green_prop_g1);
    prop_array.props.push_back(red_prop_g2);
    prop_array.props.push_back(green_prop_g2);

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