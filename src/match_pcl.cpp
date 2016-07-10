#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

using namespace std;

ros::Subscriber sub;

//from http://pointclouds.org/documentation/tutorials/using_pcl_with_eclipse.php
//from http://answers.ros.org/question/60095/importerror-no-module-named-genmsgtemplate_tools/
//from http://wiki.ros.org/catkin/Tutorials/using_a_workspace
//from http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
//from http://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/

//from http://answers.ros.org/question/98011/how-to-convert-pclpointcloud2-to-pointcloudt-in-hydro/
//from http://docs.ros.org/hydro/api/pcl_conversions/html/namespacepcl.html#af662c7d46db4cf6f7cfdc2aaf4439760

 pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget (new pcl::PointCloud<pcl::PointXYZ>);

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void printCalculateMatch(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudSource,
		 	 	 	 	 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudTarget)
{
	//remove NAN points from the cloud
	std::vector<int> indices1;
	std::vector<int> indices2;
	pcl::removeNaNFromPointCloud(*cloudSource,*cloudSource, indices1);
	pcl::removeNaNFromPointCloud(*cloudTarget,*cloudTarget, indices2);
	std::cout << "PointCloud in  Size: " << cloudSource->points.size() << std::endl;
	std::cout << "PointCloud out Size: " << cloudTarget->points.size() << std::endl;
	

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaximumIterations (50);
	icp.setInputSource(cloudSource);
	icp.setInputTarget(cloudTarget);
	//pcl::PointCloud<pcl::PointXYZ> Final;


	//from http://www.pcl-users.org/Segmentation-fault-using-ICP-td4026620.html
	Eigen::Matrix4f guess = Eigen::Matrix4f::Identity ();
	guess(2, 3) = 0.1;
	icp.setMaximumIterations (1);
	icp.align(*cloudSource, guess);


	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	print4x4Matrix(icp.getFinalTransformation().cast<double>());
 	pcl::PointCloud<pcl::PointXYZ>::Ptr tempNewCloud (new pcl::PointCloud<pcl::PointXYZ>);
 	*cloudSource  = *tempNewCloud;
	*cloudTarget = *tempNewCloud;
}

void updateAndPrintCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud)
{
	*cloudTarget 	= *cloudSource;
	*cloudSource 	= *pcl_cloud;
	if(cloudTarget->points.size() != 0)
	{
		printCalculateMatch(cloudSource,cloudTarget);
	}

}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg,*pcl_cloud);
    updateAndPrintCloud(pcl_cloud);
    sub.shutdown();
}

int main (int argc, char** argv)
{
	string home =  getenv ("HOME");

	string path = home + "/ANPL/data/pcd/2.pcd";

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (path.c_str(), *cloudSource) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file 2.pcd \n");
		return (-1);
	}
	path = home + "/ANPL/data/pcd/1.pcd";

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (path.c_str(), *cloudTarget) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file 1.pcd \n");
		return (-1);
	}
	printCalculateMatch(cloudSource,cloudTarget);
	return 0;
}
