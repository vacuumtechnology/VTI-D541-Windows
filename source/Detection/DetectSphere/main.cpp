#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/uniform_sampling.h>

typedef pcl::PointXYZRGB PointT;

int main(int argc, char* argv[])
{
	// All the objects needed
	pcl::PCDReader reader;
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::PCDWriter writer;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

	// Read in the cloud data
	std::string filename = argv[1];
	reader.read(filename, *cloud);
	std::cerr << "PointCloud has: " << cloud->size() << " data points." << std::endl;


	pcl::UniformSampling<PointT> uniform_sampling;
	uniform_sampling.setInputCloud(cloud);
	uniform_sampling.setRadiusSearch(1);
	uniform_sampling.filter(*cloud);

	std::cerr << "PointCloud has: " << cloud->size() << " data points." << std::endl;

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_SPHERE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.8);
	seg.setMaxIterations(50000000);
	seg.setDistanceThreshold(.5);
	seg.setRadiusLimits(0, 40);
	seg.setInputCloud(cloud);
	seg.setInputNormals(cloud_normals);

	// Obtain the cylinder inliers and coefficients
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

	// Write the cylinder inliers to disk
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty())
		std::cerr << "Can't find the cylindrical component." << std::endl;
	else
	{
		std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size() << " data points." << std::endl;
		pcl::visualization::PCLVisualizer viewer("Cylinder Grouping");

		viewer.setSize(900, 1000);
		viewer.setBackgroundColor(.3, .3, .3);
		viewer.setCameraPosition(0, 0, -50, 0, -1, 0);
		viewer.resetCamera();

		viewer.addPointCloud(cloud, "scene_cloud");
		pcl::visualization::PointCloudColorHandlerCustom<PointT> rotated_model_color_handler(cloud_cylinder, 255, 0, 0);
		viewer.addPointCloud(cloud_cylinder, rotated_model_color_handler, "cyl_cloud");
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
		}
	}
	return (0);
}