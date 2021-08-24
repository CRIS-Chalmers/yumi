#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iostream>


int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;
    
    // for multithreading ros
    ros::AsyncSpinner spinner(0);
    spinner.start();
    
    // Tf_camera_link tf_camera_link(&n);
    // sends out on tf tree with 100 Hz 
    ros::Rate r(100);

    //tf::TransformBroadcaster broadcaster;

    tf::TransformBroadcaster broadcaster2;
    tf::TransformBroadcaster broadcaster_r_gripper;
    tf::TransformBroadcaster broadcaster_l_gripper;

    while(n.ok()){

       // robot in realtion to world frame. can be used example for camera pose estimation if a tag is used as world coordinate. 
        broadcaster2.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0.7071, -0.7071), tf::Vector3(0.181, 0, 0)),
                ros::Time::now(), "yumi_base_link", "world"));
        // new frames at the tip of the grippers. Used by the controller 
        broadcaster_r_gripper.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.146)),
                ros::Time::now(), "yumi_link_7_r", "yumi_gripp_r"));

        broadcaster_l_gripper.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0.146)),
                ros::Time::now(), "yumi_link_7_l", "yumi_gripp_l"));

        r.sleep();
    }
}
