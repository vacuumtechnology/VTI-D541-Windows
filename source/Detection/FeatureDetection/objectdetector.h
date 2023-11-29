#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <vtkRenderWindow.h>
#include <map>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <fstream>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

class Model {
public:
    Model(std::string pcdFile, std::string configFile);
    pcl::PointCloud<PointType>::Ptr cloud;
    void Process(std::string keypointType);
    void RemoveOutliers();
    void ComputeNormals();
    void SelectKeypoints(std::string keypointType);
    void ComputeDescriptors();

    pcl::PointCloud<NormalType>::Ptr model_normals;
    pcl::PointCloud<PointType>::Ptr model_keypoints;
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors;
    pcl::PointCloud<RFType>::Ptr model_rf;
    pcl::CorrespondencesPtr model_scene_corrs;
    pcl::StatisticalOutlierRemoval<PointType> sor;
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    pcl::UniformSampling<PointType> uniform_sampling;
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    float model_ss;
    float descr_rad;
    float cg_size;
    float cg_thresh;
    float rf_rad;
    int max_objects;
    float min_distance;
    float out_thresh;
    int corr_thresh;
    std::string name;
    pcl::PointCloud<PointType>::Ptr pick_points;
};

struct Match {
    Model* model;
    size_t correspondences;
    pcl::PointCloud<PointType>::Ptr rotated_model;
    Eigen::Matrix4f rototranslation;
    pcl::PointCloud<PointType>::Ptr transformedPickPoints;
};

struct ModelGroup {
    std::vector<Model*> models;
    std::multimap<size_t, Match *> bestMatches;
    int size;
    int max_objects;
};

class ObjectDetector {
	public:
        ObjectDetector(pcl::PointCloud<PointType>::Ptr sceneCloud, std::string sceneConfig, std::string cylConfig);
        float CalculateResolution(pcl::PointCloud<PointType>::Ptr sceneCloud);
        void SwitchView();
        void LoadScene(pcl::PointCloud<PointType>::Ptr scene);
        void ProcessScene(std::string keypointType);
        void ProcessSceneCylinder();
        void LoadModel(std::string modelFile, int occurences);
        std::vector<PointType> Detect();
        PointType DetectCylinder();
        int VisualizeResults();
        void SortMatches(ModelGroup* modGroup);
        void ResetAllModels();
        pcl::PointCloud<PointType>::Ptr scene_keypoints;

    private:
        void LoadParams(std::string sceneConfig, std::string cylConfig);
        void SelectKeypoints(std::string keypointType);
        void SearchThread(int i, Model *mod);
        void FindCorrespondences(Model *mod);
        void ResetModels(ModelGroup* modGroup);

        bool DetermineBestMatches(ModelGroup* modGroup);
        void PrintInstances();
        bool CheckUnmoved(Eigen::Matrix3f rotation, Eigen::Vector3f translation);
        pcl::visualization::PCLVisualizer::Ptr viewer;
        pcl::visualization::PCLVisualizer::Ptr previewer;

        pcl::PointCloud<PointType>::Ptr scene;
        std::vector<ModelGroup*> modelGroups;

        //Cylinder
        pcl::NormalEstimationOMP<PointType, pcl::Normal> cyl_norm;
        pcl::PointCloud<PointType>::Ptr cloud_cylinder;
        pcl::PointCloud<NormalType>::Ptr cyl_normals;

        // Detection Objects
        pcl::StatisticalOutlierRemoval<PointType> sor;

        pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
        pcl::PointCloud<NormalType>::Ptr scene_normals;

        pcl::UniformSampling<PointType> uniform_sampling;

        pcl::PointCloud<RFType>::Ptr scene_rf;
        pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;

        pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType>::Ptr descr_est;
        pcl::PointCloud<DescriptorType>::Ptr scene_descriptors;
        pcl::KdTreeFLANN<DescriptorType> match_search;

        std::multimap<size_t, Match*>::reverse_iterator it;
        std::multimap<size_t, Match*>::reverse_iterator it2;

        std::vector<PointType> sniffPoints;
        pcl::PointCloud<PointType>::Ptr sniffPointCloud;

        // Scene params
        float scene_ss;
        float descr_rad;
        float cg_size;
        float cg_thresh; 
        float rf_rad;
		float min_distance;
        float resolution;
        float out_thresh;
        int num_threads;
        std::string keypointType;

        // Cylinder params
        float norm_weight;
        int max_iter;
        float dist_thresh;
        int rad_min;
        int rad_max;

        std::string config;
        bool switchScene;
};
