/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//#include <thread>
//#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
//#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
//#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/tf.h>

#include <math.h>

#define MAX_DISTANCE_BETWEEN_COMMANDS 0.2
#define STOP_THRESHOLD 0.1
#define FLIGHT_ALTITUDE 1.0

void goToGoal(Eigen::Vector3d &desired_position, int &n_seq, ros::Publisher &trajectory_pub);

geometry_msgs::Pose current_pose;
void gtCallback(const geometry_msgs::PoseConstPtr& pose)
{
    current_pose = *pose;
    ROS_INFO("update current pose.");
}


int main(int argc, char** argv){
  ros::init(argc, argv, "mav_pose_controll");
  ros::NodeHandle nh;
/*
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
*/
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "/firefly/command/trajectory", 10);

  ros::Subscriber sub =
      nh.subscribe("/firefly/ground_truth/pose", 1, gtCallback);

  ROS_INFO("Started mav controller.");


  int n_seq = 0;
  std::vector<Eigen::Vector3d> goals;
  goals.push_back(Eigen::Vector3d (-9.22, -7.36, FLIGHT_ALTITUDE));
  goals.push_back(Eigen::Vector3d (+7.50, -8.00, FLIGHT_ALTITUDE));
  
  ROS_INFO("Wait for 5 seconds for initialization.");
  ros::Duration(5.0).sleep();

  for(int i = 0; i < goals.size(); i++)
  {
    goToGoal(goals[i], n_seq, trajectory_pub);
  }

  return 1;
}

void goToGoal(Eigen::Vector3d &desired_position, int &n_seq, ros::Publisher &trajectory_pub)
{
  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
  double desired_yaw = 0;
  double distance = STOP_THRESHOLD + 1.0;
  Eigen::Vector3d temp_waypoint;

  while (ros::ok() && distance > 0.25) // TODO: check why this condition is not taken into acount
  {
    ros::spinOnce();

    tf::Pose tf_pose;
    tf::poseMsgToTF(current_pose, tf_pose);
    double current_yaw = tf::getYaw(tf_pose.getRotation());
    ROS_INFO("Current Pose: [%f, %f, %f], yaw: %f.",
             current_pose.position.x,
             current_pose.position.y,
             current_pose.position.z,
             current_yaw);

    // calculate the distance to the goal
    double between_x = current_pose.position.x - desired_position.x();
    double between_y = current_pose.position.y - desired_position.y();
    double between_z = current_pose.position.z - desired_position.z();
    double distance = sqrt(between_x * between_x + between_y * between_y);

    // calculate the angle to the goal
    if (between_x > 0) // 1st & 4th quarters
    {
      desired_yaw = atan2(between_y, between_x);
    }
    else if (between_y > 0) // 2nd quarter
    {
      desired_yaw = M_PI + atan2(between_y, between_x);
    }
    else
    {
      desired_yaw = atan2(between_y, between_x) - M_PI;
    }

    // print to screen distance and angle to the goal
    ROS_INFO("vector to the goal: [%f, %f, %f].",
           between_x, between_y, between_z);
    ROS_INFO("distance to the goal: %f, angle to goal: %f.",
           distance, desired_yaw);

    // check if the distance is grater than MAX_DISTANCE_BETWEEN_COMMANDS
    // if true, calculate temp waypoint to go
    if (distance > MAX_DISTANCE_BETWEEN_COMMANDS)
    {
      distance = MAX_DISTANCE_BETWEEN_COMMANDS;
      double temp_waypoint_x = current_pose.position.x + distance * cos(desired_yaw);
      double temp_waypoint_y = current_pose.position.y + distance * sin(desired_yaw);
      temp_waypoint = Eigen::Vector3d(temp_waypoint_x, temp_waypoint_y, FLIGHT_ALTITUDE);
    }
    else
    {
      temp_waypoint = desired_position;
    }

    // publish new trajectory msg
    n_seq++;
    samples_array.header.seq = n_seq;
    samples_array.header.stamp = ros::Time::now();
    samples_array.points.clear();

    trajectory_point.position_W.x() = temp_waypoint.x();
    trajectory_point.position_W.y() = temp_waypoint.y();
    trajectory_point.position_W.z() = FLIGHT_ALTITUDE;
    tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), desired_yaw);
    trajectory_point.setFromYaw(tf::getYaw(quat));
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);

    ROS_INFO("Publishing waypoint: [%f, %f, %f].",
           temp_waypoint.x(),
           temp_waypoint.y(),
           FLIGHT_ALTITUDE);
    trajectory_pub.publish(samples_array);
    
    n_seq++;
    ros::Duration(1.0).sleep();
    
    if (distance < STOP_THRESHOLD) break;
  }
  ROS_INFO("The quad reach its destenation!!");
}
