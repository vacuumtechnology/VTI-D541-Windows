#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <map>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

class Model{
public:
    Model(std::string modelPath, float resolution);
    pcl::PointCloud<PointType>::Ptr cloud;
    void RemoveOutliers();
    void ComputeNormals();
    void Downsample();
    void ComputeDescriptors();

    pcl::PointCloud<NormalType>::Ptr model_normals;
    pcl::PointCloud<PointType>::Ptr model_keypoints;
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors;
    pcl::CorrespondencesPtr model_scene_corrs;
    pcl::StatisticalOutlierRemoval<PointType> sor;
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    pcl::UniformSampling<PointType> uniform_sampling;
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;
    std::multimap<size_t, std::pair <int, pcl::PointCloud<PointType>::Ptr>> bestMatches;


    float model_ss;
    float descr_rad;
    float cg_size;
    float cg_thresh;
    float rf_rad;
    int max_objects;
    float min_distance;
    float out_thresh;
    int corr_thresh;
    std::vector<PointType> pick_points;
};

class ObjectDetector {
	public:
        ObjectDetector(pcl::PointCloud<PointType>::Ptr sceneCloud);
        float CalculateResolution();
        void LoadParams(float scene_ss, float descr_rad, float cg_size, float cg_thresh, float rf_rad, float out_thresh, int num_threads);
        void ProcessScene();
        void Detect();

        void SearchThread(int i, Model *mod);
        void FindCorrespondences(Model *mod);

        void DetermineBestMatches(Model *mod);
        void PrintInstances();
        int VisualizeResults();
        void LoadModel(std::string modelFile);

        pcl::PointCloud<PointType>::Ptr scene;
        std::vector<Model *> models;

        pcl::StatisticalOutlierRemoval<PointType> sor;

        pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
        pcl::PointCloud<NormalType>::Ptr scene_normals;

        pcl::UniformSampling<PointType> uniform_sampling;
        pcl::PointCloud<PointType>::Ptr scene_keypoints;

        pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
        pcl::PointCloud<DescriptorType>::Ptr scene_descriptors;
        pcl::KdTreeFLANN<DescriptorType> match_search;
        std::multimap<size_t, std::pair <int, pcl::PointCloud<PointType>::Ptr>>::reverse_iterator it;
        std::multimap<size_t, std::pair <int, pcl::PointCloud<PointType>::Ptr>>::reverse_iterator it2;

        float model_ss;
        float scene_ss;
        float descr_rad;
        float cg_size;
        float cg_thresh; 
        float rf_rad;
        int max_objects;
		float min_distance;
        float resolution;
        float out_thresh;
        int num_threads;
        std::string config;
};
