#include <string>
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

std::string pioneer_name;

void poseCallback(const nav_msgs::OdometryConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
    tf::Quaternion q;
    q.setW(msg->pose.pose.orientation.w);
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", pioneer_name+"/hokuyo_link"));
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pioneer_tf_broadcaster");
    if (argc != 2)
    {
        ROS_ERROR("need pioneer name as argument");
        return -1;
    }
    //pioneer_name = argv[1];
    pioneer_name = "/pioneer1";

    ros::NodeHandle node;

    bool static_tf = true;
    // true - create static tf between the laser scaner and the robot ("base_link")
    // false - create dynamic tf between the laser scanner and the global frame ("odom")

    if(!static_tf)
    {
        ros::Subscriber sub = node.subscribe(pioneer_name+"/odom", 10, &poseCallback);
    }
    else
    {
        ros::Rate r(100);

        tf::TransformBroadcaster br;

        while(node.ok())
        {
            tf::Quaternion q = tf::Quaternion(0, 0, 0, 1);
            tf::Vector3 vector = tf::Vector3(0.2, 0.2, 0.0);
            tf::Transform transform = tf::Transform(q, vector);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", pioneer_name+"/hokuyo_link"));
            r.sleep();
        }
    }


    ros::spin();
    return 0;
}
