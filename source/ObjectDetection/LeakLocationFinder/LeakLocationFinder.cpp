/*
    Detects Instances of a model pointcloud in a scene pointcloud, takes parameters from a config file
    Eli Wilson - VTI
*/

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <map>
#include <cmath>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

class ObjectDetector {
	public:
		ObjectDetector(pcl::PointCloud<PointType>::Ptr sceneCloud, pcl::PointCloud<PointType>::Ptr modelCloud, std::string configFile);
        float CalculateResolution();
        void RemoveOutliers(bool processScene, bool processModel);
        void LoadParams(float model_ss, float scene_ss, float descr_rad, float cg_size, float cg_thresh, float rf_rad);
        void Detect();

        void ComputeNormals(pcl::PointCloud<NormalType>::Ptr model_normals, pcl::PointCloud<NormalType>::Ptr scene_normals);
        void Downsample(pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<PointType>::Ptr scene_keypoints);
        void ComputeDescriptors(pcl::PointCloud<NormalType>::Ptr model_normals, pcl::PointCloud<NormalType>::Ptr scene_normals,
                                pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<PointType>::Ptr scene_keypoints,
                                pcl::PointCloud<DescriptorType>::Ptr model_descriptors, pcl::PointCloud<DescriptorType>::Ptr scene_descriptors);
        void FindCorrespondences(pcl::PointCloud<NormalType>::Ptr model_normals, pcl::PointCloud<NormalType>::Ptr scene_normals, 
                                 pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<PointType>::Ptr scene_keypoints,
                                 pcl::PointCloud<DescriptorType>::Ptr model_descriptors, pcl::PointCloud<DescriptorType>::Ptr scene_descriptors);

        void DetermineBestMatches(int max_objects);
        void PrintInstances();
        void VisualizeResults();

    private:
        pcl::PointCloud<PointType>::Ptr scene;
        pcl::PointCloud<PointType>::Ptr model;
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
        std::vector<pcl::Correspondences> clustered_corrs;
        std::multimap<size_t, std::pair <int, pcl::PointCloud<PointType>::Ptr>> bestMatches;
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
        std::string config;
};

//
// Constructor
//
ObjectDetector::ObjectDetector(pcl::PointCloud<PointType>::Ptr sceneCloud, pcl::PointCloud<PointType>::Ptr modelCloud, std::string configFile) {
    this->scene = sceneCloud;
    this->model = modelCloud;
    this->config = configFile;
    this->min_distance = 5;
}

//
// Calculates resolution of model which is later used to adjust parameters
//
float ObjectDetector::CalculateResolution() {
    float resolution = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<PointType> tree;
    tree.setInputCloud(scene);

    std::cout << "Calculating Resolution" << endl;

    for (std::size_t i = 0; i < scene->size(); ++i)
    {
        if (!std::isfinite((*scene)[i].x))
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            resolution += sqrt(sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        resolution /= n_points;
    }

    return resolution;
}

//
// Removes Statistical Outliers to improve accuracy and decrease number of points to be processed
//
void ObjectDetector::RemoveOutliers(bool processScene, bool processModel) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    std::cout << "RemoveOutliers" << endl;

    if (processScene) {
        sor.setInputCloud(scene);
        sor.setMeanK(50);
        sor.setStddevMulThresh(.85);
        sor.filter(*scene);
    }

    if (processModel) {
        sor.setInputCloud(model);
        sor.setMeanK(50);
        sor.setStddevMulThresh(.85);
        sor.filter(*model);
    }
}

//
// Calculates resolution of model which is later used to adjust parameters
//
void ObjectDetector::LoadParams(float model_ss, float scene_ss, float descr_rad, float cg_size, float cg_thresh, float rf_rad) {
    float resolution = CalculateResolution();
    this->model_ss = model_ss * resolution;
    this->scene_ss = scene_ss * resolution;
    this->descr_rad = descr_rad * resolution;
    this->cg_size = cg_size * resolution;
    this->cg_thresh = cg_thresh;
    this->rf_rad = rf_rad * resolution;

    std::cout << "Model resolution:       " << resolution << std::endl;
    std::cout << "Model sampling size:    " << this->model_ss << std::endl;
    std::cout << "Scene sampling size:    " << this->scene_ss << std::endl;
    std::cout << "LRF support radius:     " << this->rf_rad << std::endl;
    std::cout << "SHOT descriptor radius: " << this->descr_rad << std::endl;
    std::cout << "Clustering bin size:    " << this->cg_size << std::endl << std::endl;
}

//
// Calls all member functions necessary for detection
//
void ObjectDetector::Detect() {
    pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
    pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
    pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());

    this->ComputeNormals(model_normals, scene_normals);
    this->Downsample(model_keypoints, scene_keypoints);
    this->ComputeDescriptors(model_normals, scene_normals, model_keypoints, scene_keypoints, model_descriptors, scene_descriptors);
    this->FindCorrespondences(model_normals, scene_normals, model_keypoints, scene_keypoints, model_descriptors, scene_descriptors);

}

//
// Computes normal vectors of all points in model and scene
//
void ObjectDetector::ComputeNormals(pcl::PointCloud<NormalType>::Ptr model_normals, pcl::PointCloud<NormalType>::Ptr scene_normals) {
    std::cout << "ComputeNormals" << endl;

    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch(15);
    norm_est.setInputCloud(model);
    norm_est.compute(*model_normals);
    norm_est.setInputCloud(scene);
    norm_est.compute(*scene_normals);

}

//
//  Downsample Clouds to Extract keypoints
//
void ObjectDetector::Downsample(pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<PointType>::Ptr scene_keypoints) {
    std::cout << "Downsample" << endl;

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud(model);
    uniform_sampling.setRadiusSearch(model_ss);
    uniform_sampling.filter(*model_keypoints);
    std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

    uniform_sampling.setInputCloud(scene);
    uniform_sampling.setRadiusSearch(scene_ss);
    uniform_sampling.filter(*scene_keypoints);
    std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

}


//
//  Compute Descriptor for keypoints
//
void ObjectDetector::ComputeDescriptors(pcl::PointCloud<NormalType>::Ptr model_normals, pcl::PointCloud<NormalType>::Ptr scene_normals,
                                        pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<PointType>::Ptr scene_keypoints,
                                        pcl::PointCloud<DescriptorType>::Ptr model_descriptors, pcl::PointCloud<DescriptorType>::Ptr scene_descriptors) {
    std::cout << "ComputeDescriptors" << endl;

    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch(descr_rad);

    descr_est.setInputCloud(model_keypoints);
    descr_est.setInputNormals(model_normals);
    descr_est.setSearchSurface(model);
    descr_est.compute(*model_descriptors);

    descr_est.setInputCloud(scene_keypoints);
    descr_est.setInputNormals(scene_normals);
    descr_est.setSearchSurface(scene);
    descr_est.compute(*scene_descriptors);
}

void ObjectDetector::FindCorrespondences(pcl::PointCloud<NormalType>::Ptr model_normals, pcl::PointCloud<NormalType>::Ptr scene_normals,
                                         pcl::PointCloud<PointType>::Ptr model_keypoints, pcl::PointCloud<PointType>::Ptr scene_keypoints,
                                         pcl::PointCloud<DescriptorType>::Ptr model_descriptors, pcl::PointCloud<DescriptorType>::Ptr scene_descriptors) {

    std::cout << "FindCorrespondences" << endl;

    //
    //  Find Model-Scene Correspondences with KdTree
    //
    pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud(model_descriptors);

    //  For each scene descriptor, find nearest neighbor into the model descriptor cloud and add it to the correspondences vector.
    for (std::size_t i = 0; i < scene_descriptors->size(); ++i)
    {
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        if (!std::isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);

        //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) 
        {
            pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back(corr);
        }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

    //
    //  Compute (Keypoints) Reference Frames for Hough
    //
    pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
    pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles(false);
    rf_est.setRadiusSearch(rf_rad);

    rf_est.setInputCloud(model_keypoints);
    rf_est.setInputNormals(model_normals);
    rf_est.setSearchSurface(model);
    rf_est.compute(*model_rf);

    rf_est.setInputCloud(scene_keypoints);
    rf_est.setInputNormals(scene_normals);
    rf_est.setSearchSurface(scene);
    rf_est.compute(*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize(cg_size);
    clusterer.setHoughThreshold(cg_thresh);
    clusterer.setUseInterpolation(true);
    clusterer.setUseDistanceWeight(false);

    clusterer.setInputCloud(model_keypoints);
    clusterer.setInputRf(model_rf);
    clusterer.setSceneCloud(scene_keypoints);
    clusterer.setSceneRf(scene_rf);
    clusterer.setModelSceneCorrespondences(model_scene_corrs);

    clusterer.recognize(rototranslations, clustered_corrs);

    std::cout << "Correspondences Found" << endl;

    // Sort matches by # of correspondences,
    // Create transformed model cloud, store as ( # of correspondences, (index, rotated_model) )
    for (std::size_t i = 0; i < rototranslations.size(); ++i) {
        pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);
        bestMatches.insert(std::make_pair(clustered_corrs[i].size(), std::make_pair(i, rotated_model)));
    }

}

void ObjectDetector::DetermineBestMatches(int max_objects) {
    std::cout << "DetermineBestMatches" << endl;

    int i = 0;
    int j = 0;
    float dist;
    Eigen::Vector4d diff;


    // Detects duplicate detections
    for (it = bestMatches.rbegin(); it != bestMatches.rend(); it++) {

        Eigen::Vector4d centroid1;
        pcl::compute3DCentroid(*it->second.second, centroid1);

        std::cout << "Instance " << i << ": " << centroid1.x() << ",  " << centroid1.y() << ",  " << centroid1.z() << endl;

        for (it2 = it; it2 != bestMatches.rend(); it2++) {
            if (j >= max_objects) break;

            Eigen::Vector4d centroid2;
            pcl::compute3DCentroid(*it2->second.second, centroid2);

            // Calculate distance between two translations
            diff = centroid2 - centroid1;

            dist = sqrt(pow(diff.x(), 2) + pow(diff.y(), 2) + pow(diff.z(), 2));
            std::cout << "Instances " << i << " " << j << ". Distance: " << dist << endl;

            if (dist < min_distance && i != j) {
                std::cout << "\nDuplicate Detected\n" << endl;
                // Erase duplicate instance and call recursively
                it2 = decltype(it2)(bestMatches.erase(std::next(it2).base()));
                DetermineBestMatches(max_objects);
                return;
            }

            j++;
        }
        i++;
        j = i;
        if (i >= max_objects) break;
    }
    this->max_objects = max_objects;
}

//
//  Output results
//
void ObjectDetector::PrintInstances() {
    
    int c = 0;
    for (it = bestMatches.rbegin(); it != bestMatches.rend(); it++) {
        std::cout << "\n    Instance " << c << ":" << std::endl;
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[it->second.first].size() << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = rototranslations[it->second.first].block<3, 3>(0, 0);
        Eigen::Vector3f translation = rototranslations[it->second.first].block<3, 1>(0, 3);

        printf("\n");
        printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
        printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
        printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
        printf("\n");
        printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
        c++;
        if (c >= max_objects) break;
    }
}

//
//  Visualization
//
void ObjectDetector::VisualizeResults() {
    pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
    viewer.setBackgroundColor(100, 114, 156);
    viewer.addPointCloud(scene, "scene_cloud");

    pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());

    int c = 0;
    for (it = bestMatches.rbegin(); it != bestMatches.rend(); it++) {

        std::stringstream ss_cloud;
        ss_cloud << "instance" << c;

        pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(it->second.second, 255, 0, 0);
        viewer.addPointCloud(it->second.second, rotated_model_color_handler, ss_cloud.str());

        c++;
        if (c >= max_objects) break;
    }

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}


void                        
showHelp(char* filename)
{
    std::cout << endl;
    std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd config_filename.txt" << std::endl << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "     model_ss = val         Model uniform sampling radius" << std::endl;
    std::cout << "     scene_ss = val         Scene uniform sampling radius" << std::endl;
    std::cout << "     rf_rad = val           Reference frame radius" << std::endl;
    std::cout << "     descr_rad = val        Descriptor radius" << std::endl;
    std::cout << "     cg_size = val          Cluster size" << std::endl;
    std::cout << "     max_objects = val      Number of objects to detect" << std::endl;
    std::cout << "     cg_thresh = val        Clustering threshold" << std::endl << std::endl;
}

int main(int argc, char **argv) {
    pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
    float model_ss;
    float scene_ss;
    float rf_rad;
    float descr_rad;
    float cg_size;
    float cg_thresh;
    int max_objects;
    bool view_result = true;

    std::string modelFile = argv[1];
    std::string sceneFile = argv[2];
    std::string configFile = argv[3];

   //  Load clouds
    if (pcl::io::loadPCDFile(modelFile, *model) < 0) {
        std::cout << "Error loading model cloud." << std::endl;
        showHelp(argv[0]);
        return (-1);
    }
    if (pcl::io::loadPCDFile(sceneFile, *scene) < 0) {
        std::cout << "Error loading scene cloud." << std::endl;
        showHelp(argv[0]);
        return (-1);
    }

    // Load config
    std::ifstream cFile(configFile);
    if (cFile.is_open())
    {
        std::string line;
        while (getline(cFile, line)) {
            line.erase(std::remove_if(line.begin(), line.end(), isspace),
                line.end());
            if (line[0] == '#' || line.empty())
                continue;
            auto delimiterPos = line.find("=");
            auto name = line.substr(0, delimiterPos);
            auto value = line.substr(delimiterPos + 1);

            if (name == "model_ss") model_ss = atof(value.c_str());
            else if (name == "scene_ss") scene_ss = atof(value.c_str());
            else if (name == "rf_rad") rf_rad = atof(value.c_str());
            else if (name == "descr_rad") descr_rad = atof(value.c_str());
            else if (name == "cg_size") cg_size = atof(value.c_str());
            else if (name == "cg_thresh") cg_thresh = atof(value.c_str());
            else if (name == "max_objects") max_objects = atoi(value.c_str());
            else if (name == "view_result") view_result = (value == "true");

        }

    } else {
        std::cerr << "Couldn't open config file for reading.\n";
    }

    ObjectDetector *obj = new ObjectDetector(scene, model, configFile);

    obj->RemoveOutliers(true, true);

    obj->LoadParams(model_ss, scene_ss, descr_rad, cg_size, cg_thresh, rf_rad);
    obj->Detect();

    obj->DetermineBestMatches(max_objects);
   
    obj->PrintInstances();

    if(view_result)
        obj->VisualizeResults();

    delete(obj);
    return 0;
}