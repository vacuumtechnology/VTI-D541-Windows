#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

int main(int argc, char** argv) {
    if (argc < 10) {
        std::cout << "Usage: " << argv[0] << " <manifold.pcd> <feature.pcd> <voxel_grid_leaf_size> <k_search_normals> <fpfh_radius> <match_threshold> <cluster_tolerance> <min_cluster_size> <max_cluster_size>\n";
        return -1;
    }

    // Load point clouds from PCD files
    pcl::PointCloud<pcl::PointXYZ>::Ptr manifold (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr feature (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *manifold) == -1 || pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *feature) == -1) {
        std::cout << "Couldn't read the PCD file. Ensure the file path is correct.\n";
        return -1;
    }

    float voxel_grid_leaf_size = std::stof(argv[3]);
    int k_search_normals = std::stoi(argv[4]);
    float fpfh_radius = std::stof(argv[5]);
    float match_threshold = std::stof(argv[6]);
    float cluster_tolerance = std::stof(argv[7]);
    int min_cluster_size = std::stoi(argv[8]);
    int max_cluster_size = std::stoi(argv[9]);

    // Preprocessing: Downsampling
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(manifold);
    sor.setLeafSize(voxel_grid_leaf_size, voxel_grid_leaf_size, voxel_grid_leaf_size);
    sor.filter(*manifold);

    // Compute normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr manifold_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr feature_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(manifold);
    ne.setKSearch(k_search_normals);
    ne.compute(*manifold_normals);
    ne.setInputCloud(feature);
    ne.compute(*feature_normals);

    // Feature extraction
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr manifold_fpfh (new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr feature_fpfh (new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh.setNumberOfThreads(8);  // Assuming 8 threads for OpenMP
    fpfh.setInputCloud(manifold);
    fpfh.setInputNormals(manifold_normals);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(fpfh_radius);
    fpfh.compute(*manifold_fpfh);
    fpfh.setInputCloud(feature);
    fpfh.setInputNormals(feature_normals);
    fpfh.compute(*feature_fpfh);

    // Feature matching and geometric verification
    pcl::KdTreeFLANN<pcl::FPFHSignature33> match_search;
    pcl::CorrespondencesPtr correspondences (new pcl::Correspondences());
    match_search.setInputCloud(feature_fpfh);
    for (size_t i = 0; i < manifold_fpfh->size(); ++i) {
        std::vector<int> neigh_indices;
        std::vector<float> neigh_sqr_dists;
        if (!pcl_isfinite(manifold_fpfh->at(i).histogram[0])) 
            continue;
        int found_neighs = match_search.nearestKSearch(manifold_fpfh->at(i), 1, neigh_indices, neigh_sqr_dists);
        if (found_neighs == 1 && neigh_sqr_dists[0] < match_threshold) {
            pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
            correspondences->push_back(corr);
        }
    }

    pcl::CorrespondencesPtr refined_correspondences (new pcl::Correspondences());
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> refine;
    refine.setInputSource(feature);
    refine.setInputTarget(manifold);
    refine.setInputCorrespondences(correspondences);
    refine.getCorrespondences(*refined_correspondences);

    // Clustering & Detection
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.setInputCloud(manifold);
    ec.extract(cluster_indices);

    // Visualization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(manifold, "manifold");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "manifold");
    int j = 0;
    for (const auto& cluster : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& index : cluster.indices)
            cloud_cluster->points.push_back(manifold->points[index]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        std::stringstream ss;
        ss << "Cloud" << j++;
        viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, ss.str());
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
    }
    viewer->spin();

    return 0;
}