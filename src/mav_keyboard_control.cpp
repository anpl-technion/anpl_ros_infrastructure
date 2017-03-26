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

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/tf.h>

#include <math.h>

#include <boost/asio.hpp>
#include <stdio.h>
#include <termio.h>
#include <unistd.h>

#define FORWARD_STEP_SIZE   0.5 // [m]
#define ANGULAR_STEP_SIZE   10.0 // [deg]
#define TAKEOFF_ALTITUDE    1.0  // [m]
#define ELEVATION_STEP_SIZE 0.1  // [m]

// arrows key definition
#define KEY_UP    65
#define KEY_DOWN  66
#define KEY_LEFT  68
#define KEY_RIGHT 67
#define NUMPAD_1  52
#define NUMPAD_3  54
#define NUMPAD_5  69
#define NUMPAD_7  49
#define NUMPAD_9  53
#define SPACE     32

// get char from user without pressing enter
// from: http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
char getch();

void takeoff() {};

void move(const char tav, const ros::Publisher &trajectory_pub);

geometry_msgs::Pose current_pose;
void gtCallback(const geometry_msgs::PoseConstPtr& pose)
{
    current_pose = *pose;
    //ROS_INFO("update current pose.");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mav_keyboard_controll");
  ros::NodeHandle nh;

  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
/*
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "/firefly/command/trajectory", 10);
*/
  ros::Subscriber sub =
      nh.subscribe("/firefly/ground_truth/pose", 1, gtCallback);

  ROS_INFO("Start mav keyboard controller.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  ROS_INFO("Wait for 5 seconds for initialization.");
  ros::Duration(5.0).sleep();
  ros::spinOnce();

  // type instruction
  std::cout << std::endl;
  std::cout << "Control Your MAV!" << std::endl;
  std::cout << "----------------------------------" << std::endl;
  std::cout << "Instructions:" << std::endl;
  std::cout << "ARROW KEY UP    - move forward    " << std::endl;
  std::cout << "ARROW KEY DOWN  - move backward   " << std::endl;
  std::cout << "ARROW KEY LEFT  - turn left       " << std::endl;
  std::cout << "ARROW KEY RIGHT - turn right      " << std::endl;
  std::cout << "SPACE KEY       - takeoff / land  " << std::endl;
  std::cout << "+ KEY           - elevate         " << std::endl;
  std::cout << "- KEY           - decline         " << std::endl;
  std::cout << std::endl;
  std::cout << "Press q to quit" << std::endl;
  std::cout << std::endl;

  char tav = SPACE;
  bool is_takeoff = false;

  //takeoff();
  move(tav, trajectory_pub);

  // receive input from user and execute 
  bool isCorrect = false;
  while('q' != tav && ros::ok()) {
    tav = getch();
    isCorrect = false;
    switch(tav) {
      case KEY_UP:
      case KEY_DOWN:
      case KEY_LEFT:
      case KEY_RIGHT:
      case SPACE:
      case '+':
      case '-':
        isCorrect = true;
	break;
    }

    if (isCorrect) // publish new trajectory msg
    {
      move(tav, trajectory_pub);
    }
  }

  // land
  move(tav, trajectory_pub);

  // pausing gazebo sim
  ROS_INFO("Wait for 5 seconds before pausing gazebo.");
  ros::Duration(5.0).sleep();
  bool paused = ros::service::call("/gazebo/pause_physics", srv);
  while (i <= 10 && !paused)
  {
    ROS_INFO("Wait for 1 second before trying to pause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    paused = ros::service::call("/gazebo/pause_physics", srv);
    ++i;
  }
}


void move(const char tav, const ros::Publisher &trajectory_pub)
{
  static int n_seq = 0;
  static bool is_takeoff = false;
  static double desired_z = TAKEOFF_ALTITUDE;

  ros::spinOnce();

  tf::Pose tf_pose;
  tf::poseMsgToTF(current_pose, tf_pose);
  double current_yaw = tf::getYaw(tf_pose.getRotation());
  //ROS_INFO("Current Pose: [%f, %f, %f], yaw: %f.",
  //          current_pose.position.x,
  //          current_pose.position.y,
  //          current_pose.position.z,
  //          current_yaw);

  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;

  static double desired_x;
  static double desired_y;
  static double desired_yaw;

  switch(tav) {
    case KEY_UP:
      desired_x = current_pose.position.x + FORWARD_STEP_SIZE * cos(current_yaw);
      desired_y = current_pose.position.y + FORWARD_STEP_SIZE * sin(current_yaw);
      //desired_z = current_pose.position.z;
      //desired_yaw = current_yaw;
      break;
    case KEY_DOWN:
      desired_x = current_pose.position.x - FORWARD_STEP_SIZE * cos(current_yaw);
      desired_y = current_pose.position.y - FORWARD_STEP_SIZE * sin(current_yaw);
      //desired_z = current_pose.position.z;
      //desired_yaw = current_yaw;
      break;
    case KEY_LEFT:
      //desired_x = current_pose.position.x;
      //desired_y = current_pose.position.y;
      //desired_z = current_pose.position.z;
      desired_yaw = current_yaw + ANGULAR_STEP_SIZE * (M_PI / 180);
      break;
    case KEY_RIGHT:
      //desired_x = current_pose.position.x;
      //desired_y = current_pose.position.y;
      //desired_z = current_pose.position.z;
      desired_yaw = current_yaw - ANGULAR_STEP_SIZE * (M_PI / 180);
      break;
    case '+':
      //desired_x = current_pose.position.x;
      //desired_y = current_pose.position.y;
      desired_z += ELEVATION_STEP_SIZE;
      //desired_yaw = current_yaw;
      break;
    case '-':
      //desired_x = current_pose.position.x;
      //desired_y = current_pose.position.y;
      desired_z -= ELEVATION_STEP_SIZE;
      //desired_yaw = current_yaw - ANGULAR_STEP_SIZE * (M_PI / 180);
      break;
    case 'q': // land the mav when the user want to exit
      is_takeoff = true;
    case SPACE:
      desired_x = current_pose.position.x;
      desired_y = current_pose.position.y;

      if (!is_takeoff)
      {
        desired_z = TAKEOFF_ALTITUDE;
        is_takeoff = true;
        ROS_INFO("Takeoff");
      }
      else
      {
        desired_z = 0.0;
        is_takeoff = false;
        ROS_INFO("Land");
      }
      
      if(n_seq == 0) // first takeoff
      {
        desired_yaw = M_PI / 2;
        ROS_INFO("Takeoff at: [%f, %f, %f], yaw: %f.", desired_x, desired_y, desired_z, desired_yaw);
      }
      else
      {
        desired_yaw = current_yaw;
      }

      break;
    }

  samples_array.header.seq = n_seq;
  samples_array.header.stamp = ros::Time::now();
  samples_array.points.clear();

  trajectory_point.position_W.x() = desired_x;
  trajectory_point.position_W.y() = desired_y;
  trajectory_point.position_W.z() = desired_z;
  tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), desired_yaw);
  trajectory_point.setFromYaw(tf::getYaw(quat));
  mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
  samples_array.points.push_back(trajectory_point_msg);

  //ROS_INFO("Publishing waypoint: [%f, %f, %f].", desired_x, desired_y, desired_z);
  trajectory_pub.publish(samples_array);
    
  n_seq++;
  //ros::Duration(1.0).sleep();
}


// get char from user without pressing enter
// from: http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}
