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


pcl::visualization::CloudViewer viewer("3D map");
boost::mutex mtx_;
PointCloudPtr global;
bool first = true;

// Filter Parameters
float min_depth = 0.5;
float max_depth = 5;
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

// Callback
void cloud_cb_(const PointCloudConstPtr& cloud_in)
{
	boost::mutex::scoped_lock lock (mtx_);

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
	}
	viewer.showCloud(global);
	src_features = tgt_features; // Get Source data for next iteration
}


// Main
int main (int argc, char ** argv)
{
	pcl::OpenNIGrabber* interface = new pcl::OpenNIGrabber("", pcl::OpenNIGrabber::OpenNI_QVGA_30Hz, pcl::OpenNIGrabber::OpenNI_QVGA_30Hz);

	global.reset(new PointCloud);

	boost::function<void (const PointCloudConstPtr&)> f = boost::bind (&cloud_cb_, _1);
	boost::signals2::connection c = interface->registerCallback (f);

	interface->start();

	while(!viewer.wasStopped ())
	{
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}
	interface->stop ();
	return (0);
}
