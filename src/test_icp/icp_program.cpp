/*
 * taken from http://www.pcl-users.org/Segmentation-fault-using-ICP-td4026620.html
 * link to zip file: http://www.super-resume.com/Index$SetST.jtp?p=1
 * This program grabs the pointcloud from the opnenni interface
 * The Cloud is then filtered
 * Keypoints and desctriptors are found in the filtered cloud
 * These are used to find the initial_alignment
 * The initial_alignment is then used in ICP
 * The cloud is transformed and added to global map
*/

#include "typedefs.h"
#include "filters.h"
#include "feature_estimation.h"
#include "registration.h"

#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl_conversions/pcl_conversions.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"


pcl::visualization::CloudViewer viewer("3D map");
PointCloudPtr global;
bool first = true;

// Filter Parameters
float min_depth = 0;
float max_depth = 2;
float leaf_size = 0.01;
float radius = 0.05;
float min_neighbors = 8;

// Pose Estimation Parameters
ObjectFeatures src_features;
float min_sample_dist = 0.05;
float max_correspondence_dist = 0.1;
int nr_iters = 500;

// ICP Paramenters
float max_correspondence_distance = 0.1;
float outlier_rejection_threshold = 0.05;
float transformation_epsilon = 1e-5;
int max_iterations = 16;

Eigen::Matrix4f global_tform = Eigen::Matrix4f::Identity();


// Printing transformation
void print4x4Matrix (const Eigen::Matrix4f & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


// Callback
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg,*cloud_in);

	PointCloudPtr cloud_;
	cloud_.reset(new PointCloud());
	pcl::copyPointCloud(*cloud_in,*cloud_);

	PointCloudPtr cloud_filtered;
	cloud_filtered.reset(new PointCloud());
	pcl::copyPointCloud(*applyFilters(cloud_, min_depth, max_depth, leaf_size, radius, min_neighbors), *cloud_filtered);

	ObjectFeatures tgt_features;
	// Points, Normals, Keypoints and Descriptors
	tgt_features = computeFeatures(cloud_filtered);

	if(first)
	{
		*global += *(cloud_filtered);
		first = false;
	}
	else
	{
		Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();

		// Compute Initial Alignment
		tform = computeInitialAlignment(src_features.keypoints, src_features.local_descriptors, tgt_features.keypoints, tgt_features.local_descriptors, min_sample_dist, max_correspondence_dist, nr_iters);

		// Use ICP with Initial Alignment
		tform = refineAlignment(src_features.points, tgt_features.points, tform, max_correspondence_distance, outlier_rejection_threshold, transformation_epsilon, max_iterations);

		// Add to Global Map
		Eigen::Matrix4f relative_tform = tform.inverse();
		global_tform = global_tform * relative_tform;

		pcl::PointCloud<pcl::PointXYZRGBA> transformedCloud;
		pcl::transformPointCloud (*src_features.points, transformedCloud, global_tform);

		*global += transformedCloud; 
		print4x4Matrix(global_tform);
	}
	viewer.showCloud(global);
	src_features = tgt_features; // Get Source data for next iteration
	print4x4Matrix(global_tform);
}

void run_ros(int argc, char ** argv)
{
	   /**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line.
	* For programmatic remappings you can use a different version of init() which takes
	* remappings directly, but for most command-line programs, passing argc and argv is
	* the easiest way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "icp_program");

   /**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

   /**
	* The subscribe() call is how you tell ROS that you want to receive messages
	* on a given topic.  This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing.  Messages are passed to a callback function, here
	* called chatterCallback.  subscribe() returns a Subscriber object that you
	* must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	* object go out of scope, this callback will automatically be unsubscribed from
	* this topic.
	*
	* The second parameter to the subscribe() function is the size of the message
	* queue.  If messages are arriving faster than they are being processed, this
	* is the number of messages that will be buffered up before beginning to throw
	* away the oldest ones.
	*/
	ros::Subscriber sub = n.subscribe("/camera/depth/points", 1, pointCloudCallback);

   /**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/
	ros::spin();
}

// Main
int main (int argc, char ** argv)
{
	global.reset(new PointCloud);
	run_ros(argc,argv);
	return (0);
}
