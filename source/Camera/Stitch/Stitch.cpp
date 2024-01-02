#include <iostream>                                                
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>

// --------------------
// -----Parameters-----
// --------------------
// SIFT Keypoint parameters
const float min_scale = 0.4f; // the standard deviation of the smallest scale in the scale space
const int n_octaves = 10;  // the number of octaves (i.e. doublings of scale) to compute
const int n_scales_per_octave = 10; // the number of scales to compute within each octave
const float min_contrast = 0.1f; // the minimum contrast required for detection

// Sample Consensus Initial Alignment parameters (explanation below)
const float min_sample_dist = 0.025f;
const float max_correspondence_dist = 1.0f;
const int nr_iters = 500000;

// ICP parameters (explanation below)
const float max_correspondence_distance = 50;
const float outlier_rejection_threshold = 10.0f;
const float transformation_epsilon = 1e-30;
const float transformation_rotation_epsilon = 1e-30;
const float euclidean_fitness_epsilon = 1e-30;
const int max_iterations = 50000000;

// --------------
// -----Help-----
// --------------
void
printUsage(const char* progName)
{
    std::cout << "\n\nUsage: " << progName << " [options] <file.pcd> <file.pcd>\n\n"
        << "Options:\n"
        << "-------------------------------------------\n"
        << "-h           this help\n"
        << "\n\n";
}

void
setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
    viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
        look_at_vector[0], look_at_vector[1], look_at_vector[2],
        up_vector[0], up_vector[1], up_vector[2]);
}

/* Use SampleConsensusInitialAlignment to find a rough alignment from the source cloud to the target cloud by finding
 * correspondences between two sets of local features
 * Inputs:
 *   source_points
 *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
 *   source_descriptors
 *     The local descriptors for each source point
 *   target_points
 *     The "target" points, i.e., the points to which the source point cloud will be aligned
 *   target_descriptors
 *     The local descriptors for each target point
 *   min_sample_distance
 *     The minimum distance between any two random samples
 *   max_correspondence_distance
 *     The maximum distance between a point and its nearest neighbor correspondent in order to be considered
 *     in the alignment process
 *   nr_interations
 *     The number of RANSAC iterations to perform
 * Return: A transformation matrix that will roughly align the points in source to the points in target
 */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::FPFHSignature33 LocalDescriptorT;
typedef pcl::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;
Eigen::Matrix4f
computeInitialAlignment(const PointCloudPtr& source_points, const LocalDescriptorsPtr& source_descriptors,
    const PointCloudPtr& target_points, const LocalDescriptorsPtr& target_descriptors,
    float min_sample_distance, float max_correspondence_distance, int nr_iterations)
{
    pcl::SampleConsensusInitialAlignment<PointT, PointT, LocalDescriptorT> sac_ia;
    sac_ia.setMinSampleDistance(min_sample_distance);
    sac_ia.setMaxCorrespondenceDistance(max_correspondence_distance);
    sac_ia.setMaximumIterations(nr_iterations);

    sac_ia.setInputSource(source_points);
    sac_ia.setSourceFeatures(source_descriptors);

    sac_ia.setInputTarget(target_points);
    sac_ia.setTargetFeatures(target_descriptors);

    PointCloud registration_output;
    sac_ia.align(registration_output);

    return (sac_ia.getFinalTransformation());
}

/* Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud,
 * starting with an intial guess
 * Inputs:
 *   source_points
 *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
 *   target_points
 *     The "target" points, i.e., the points to which the source point cloud will be aligned
 *   intial_alignment
 *     An initial estimate of the transformation matrix that aligns the source points to the target points
 *   max_correspondence_distance
 *     A threshold on the distance between any two corresponding points.  Any corresponding points that are further
 *     apart than this threshold will be ignored when computing the source-to-target transformation
 *   outlier_rejection_threshold
 *     A threshold used to define outliers during RANSAC outlier rejection
 *   transformation_epsilon
 *     The smallest iterative transformation allowed before the algorithm is considered to have converged
 *   max_iterations
 *     The maximum number of ICP iterations to perform
 * Return: A transformation matrix that will precisely align the points in source to the points in target
 */
typedef pcl::PointXYZRGB ICPPointT;
typedef pcl::PointCloud<ICPPointT> ICPPointCloud;
typedef pcl::PointCloud<ICPPointT>::Ptr ICPPointCloudPtr;
Eigen::Matrix4f
refineAlignment(const ICPPointCloudPtr& source_points, const ICPPointCloudPtr& target_points,
    const Eigen::Matrix4f initial_alignment, float max_correspondence_distance,
    float outlier_rejection_threshold, float transformation_epsilon, float max_iterations, float &score) {

    pcl::IterativeClosestPoint<ICPPointT, ICPPointT> icp;
    icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    icp.setRANSACOutlierRejectionThreshold(outlier_rejection_threshold);
    icp.setTransformationEpsilon(transformation_epsilon);
    icp.setTransformationRotationEpsilon(transformation_rotation_epsilon);
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    icp.setMaximumIterations (max_iterations);

    ICPPointCloudPtr source_points_transformed(new ICPPointCloud);
    pcl::transformPointCloud(*source_points, *source_points_transformed, initial_alignment);

    icp.setInputSource(source_points);
    icp.setInputTarget(target_points);

    ICPPointCloud registration_output;
    icp.align(registration_output, initial_alignment);

    score = icp.getFitnessScore();
    std::cout << "score: " << icp.getFitnessScore() << endl;

    return (icp.getFinalTransformation());
}


// --------------
// -----Main-----
// --------------
int 
main (int argc, char** argv) {



    // Parse Command Line Arguments
    if (pcl::console::find_argument (argc, argv, "-h") >= 0) {
        printUsage (argv[0]);
        return 0;
    }
  
    // Read pcd file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>& source_cloud = *source_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZRGB>& target_cloud = *target_cloud_ptr;
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");

    if (!pcd_filename_indices.empty ()) {
        std::string src_filename = argv[pcd_filename_indices[0]];
        if (pcl::io::loadPCDFile (src_filename, source_cloud) == -1) {
            cerr << "Was not able to open file \""<<src_filename<<"\".\n";
            printUsage (argv[0]);
            return 0;
        }
        std::string tar_filename = argv[pcd_filename_indices[1]];
        if (pcl::io::loadPCDFile (tar_filename, target_cloud) == -1) {
            cerr << "Was not able to open file \""<<tar_filename<<"\".\n";
            printUsage (argv[0]);
            return 0;
        }
    } else {
        cout << "\nNo *.pcd file given.\n\n";
        return 0;
    }
  
    // Downsample input clouds
    /*  
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.setInputCloud(source_cloud_ptr);
    sor.filter(source_cloud);
    sor.setInputCloud(target_cloud_ptr);
    sor.filter(target_cloud);
    */

    // Remove NaN points from point clouds
    // (this is necessary to avoid a segfault when running ICP)
    std::vector<int> nan_idx;
    pcl::removeNaNFromPointCloud(source_cloud, source_cloud, nan_idx);
    pcl::removeNaNFromPointCloud(target_cloud, target_cloud, nan_idx);

    // Estimate cloud normals
    cout << "Computing source cloud normals\n";
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr src_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>& src_normals = *src_normals_ptr;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_XYZRGB (new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setInputCloud(source_cloud_ptr);
    ne.setSearchMethod(tree_XYZRGB);
    ne.setKSearch(15);
    ne.compute(*src_normals_ptr);
    for(size_t i = 0;  i < src_normals.points.size(); ++i) {
        src_normals.points[i].x = source_cloud.points[i].x;
        src_normals.points[i].y = source_cloud.points[i].y;
        src_normals.points[i].z = source_cloud.points[i].z;
    }

    cout << "Computing target cloud normals\n";
    pcl::PointCloud<pcl::PointNormal>::Ptr tar_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>& tar_normals = *tar_normals_ptr;
    ne.setInputCloud(target_cloud_ptr);
    ne.compute(*tar_normals_ptr);
    for(size_t i = 0;  i < tar_normals.points.size(); ++i) {
        tar_normals.points[i].x = target_cloud.points[i].x;
        tar_normals.points[i].y = target_cloud.points[i].y;
        tar_normals.points[i].z = target_cloud.points[i].z;
    }
  
    // Extract FPFH features from SIFT keypoints
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_keypoints_XYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);                           
    //pcl::copyPointCloud(src_keypoints, *src_keypoints_XYZRGB);
    pcl::copyPointCloud (source_cloud, *src_keypoints_XYZRGB);
    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setSearchSurface (source_cloud_ptr);
    fpfh.setInputCloud (src_keypoints_XYZRGB);
    fpfh.setInputNormals (src_normals_ptr);
    fpfh.setSearchMethod (tree_XYZRGB);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>& src_features = *src_features_ptr;
    fpfh.setRadiusSearch(8.8);
    fpfh.compute(src_features);
    cout << "Computed " << src_features.size() << " FPFH features for source cloud\n";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tar_keypoints_XYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);                           
    //pcl::copyPointCloud(tar_keypoints, *tar_keypoints_XYZRGB);
    pcl::copyPointCloud (target_cloud, *tar_keypoints_XYZRGB);
    fpfh.setSearchSurface (target_cloud_ptr);
    fpfh.setInputCloud (tar_keypoints_XYZRGB);
    fpfh.setInputNormals (tar_normals_ptr);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr tar_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>& tar_features = *tar_features_ptr;
    fpfh.compute(tar_features);
    cout << "Computed " << tar_features.size() << " FPFH features for target cloud\n";
  
    // Compute the transformation matrix for alignment
    Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
    tform = computeInitialAlignment (source_cloud_ptr, src_features_ptr, target_cloud_ptr,
        tar_features_ptr, min_sample_dist, max_correspondence_dist, nr_iters);


    /* ICP */
    std::map<float, Eigen::Matrix4f> transforms;
    float score;
    //for (int i = 0; i < 200; i++) {
        tform = refineAlignment (source_cloud_ptr, target_cloud_ptr, tform, max_correspondence_distance,
            outlier_rejection_threshold, transformation_epsilon, max_iterations, score);
        transforms.insert(std::make_pair(score, tform));
    //}
    
    cout << "final score: " << transforms.begin()->first << endl;
    tform = transforms.begin()->second;
 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>& transformed_cloud = *transformed_cloud_ptr;
    pcl::transformPointCloud(source_cloud, transformed_cloud, tform);
    cout << "Calculated transformation\n";

    /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    *combined_cloud = target_cloud + transformed_cloud;*/

    /*cout << "organized: " << combined_cloud->isOrganized() << endl;

    pcl::io::savePCDFileBinary("../../pcd/combined.pcd", *combined_cloud);*/

    // Create 3D viewer and add point clouds
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");

    viewer.setSize(900, 1000);
    viewer.setBackgroundColor(.3, .3, .3);
    viewer.setCameraPosition(0, 0, -40, 0, -1, 0);
    //viewer.addPointCloud(combined_cloud);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> tar_cloud_color_handler (target_cloud_ptr, 255, 0, 0);
    viewer.addPointCloud (target_cloud_ptr, tar_cloud_color_handler, "target cloud v2");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "target cloud v2");
  
    // Add transformed point cloud to viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> tf_cloud_color_handler (transformed_cloud_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZRGB> (transformed_cloud_ptr, tf_cloud_color_handler, "initial aligned cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "initial aligned cloud");
    viewer.resetCamera();

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
        pcl_sleep(0.01);
    }
}