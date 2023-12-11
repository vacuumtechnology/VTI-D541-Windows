#include "Stitch.hpp" 



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

    // Estimate the SIFT keypoints
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale>::Ptr src_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PointWithScale>& src_keypoints = *src_keypoints_ptr;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    sift.setSearchMethod(tree_normal);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(source_cloud_ptr);
    sift.compute(src_keypoints);

    cout << "Found " << src_keypoints.points.size () << " SIFT keypoints in source cloud\n";
 
    pcl::PointCloud<pcl::PointWithScale>::Ptr tar_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PointWithScale>& tar_keypoints = *tar_keypoints_ptr;
    sift.setInputCloud(target_cloud_ptr);
    sift.compute(tar_keypoints);

    cout << "Found " << tar_keypoints.points.size () << " SIFT keypoints in target cloud\n";
  
    // Extract FPFH features from SIFT keypoints
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_keypoints_XYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);                           
    pcl::copyPointCloud (src_keypoints, *src_keypoints_XYZRGB);
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
    pcl::copyPointCloud (tar_keypoints, *tar_keypoints_XYZRGB);
    fpfh.setSearchSurface (target_cloud_ptr);
    fpfh.setInputCloud (tar_keypoints_XYZRGB);
    fpfh.setInputNormals (tar_normals_ptr);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr tar_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>& tar_features = *tar_features_ptr;
    fpfh.compute(tar_features);
    cout << "Computed " << tar_features.size() << " FPFH features for target cloud\n";
  
    // Compute the transformation matrix for alignment
    Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
    tform = computeInitialAlignment (src_keypoints_ptr, src_features_ptr, tar_keypoints_ptr,
        tar_features_ptr, min_sample_dist, max_correspondence_dist, nr_iters);
  
    /* Uncomment this code to run ICP */
    tform = refineAlignment (source_cloud_ptr, target_cloud_ptr, tform, max_correspondence_distance,
        outlier_rejection_threshold, transformation_epsilon, max_iterations);
  
 
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
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> tar_cloud_color_handler (target_cloud_ptr, 0, 255, 255);
    viewer.addPointCloud (target_cloud_ptr, tar_cloud_color_handler, "target cloud v2");
  
    // Add transformed point cloud to viewer
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> tf_cloud_color_handler (transformed_cloud_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZRGB> (transformed_cloud_ptr, tf_cloud_color_handler, "initial aligned cloud");
    viewer.resetCamera();

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer.wasStopped ()) {
        viewer.spinOnce ();
        pcl_sleep(0.01);
    }
}