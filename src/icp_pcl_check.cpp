#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

int main (int argc, char* argv[]) {

	// The point clouds we will be using
	PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
	PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

	int iterations = 50;  // Default number of ICP iterations

	pcl::console::TicToc time;
	time.tic ();

	string home =  getenv ("HOME");

	string path = home + "/ANPL/data/pcd/1.pcd";

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (path.c_str(), *cloud_in) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file 1.pcd \n");
		return (-1);
	}

	 std::cout << "\nLoaded file " << "1.pcd" << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;

	// Defining a rotation matrix and translation vector
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

	// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	double theta = M_PI / 8;  // The angle of rotation in radians
	transformation_matrix (0, 0) = cos (theta);
	transformation_matrix (0, 1) = -sin (theta);
	transformation_matrix (1, 0) = sin (theta);
	transformation_matrix (1, 1) = cos (theta);

	// A translation on Z axis (0.4 meters)
	transformation_matrix (2, 3) = 0.4;

	// Display in terminal the transformation matrix
	std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	print4x4Matrix (transformation_matrix);

	// Executing the transformation
	pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
	*cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use
	
	//remove NAN points from the cloud
	std::vector<int> indices1;
	std::vector<int> indices2;
	pcl::removeNaNFromPointCloud(*cloud_icp,*cloud_icp, indices1);
	pcl::removeNaNFromPointCloud(*cloud_in,*cloud_in, indices2);
	std::cout << "PointCloud icp  Size: " << cloud_icp->points.size() << std::endl;
	std::cout << "PointCloud in Size: "   << cloud_in ->points.size() << std::endl;

	// The Iterative Closest Point algorithm
	time.tic ();
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations (iterations);
	icp.setInputSource (cloud_in);
	icp.setInputTarget (cloud_icp);
	icp.align (*cloud_in);
	icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
	std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

	if (icp.hasConverged ())
	{
	    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
	    std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
	    transformation_matrix = icp.getFinalTransformation ().cast<double>();
	    print4x4Matrix (transformation_matrix);
	}
	else
	{
	    PCL_ERROR ("\nICP has not converged.\n");
	    return (-1);
	}
}
