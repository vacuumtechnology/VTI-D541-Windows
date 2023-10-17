/*
    Detects Instances of a model pointcloud in a scene pointcloud, takes parameters from a config file
    Usage: ./ObjectDetector scene.pcd sceneconfig.txt
        - use "find model" button to select folder with model pointcloud and model config file
    Eli Wilson - VTI
*/

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
#include <map>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include "pclviewer.h"
#include <QApplication>
#include <QMainWindow>
#include "objectdetector.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::mutex mtx;

//
// Constructor
//
ObjectDetector::ObjectDetector(pcl::PointCloud<PointType>::Ptr sceneCloud) {
    this->scene = sceneCloud;
    this->min_distance = 6;
    scene_normals.reset(new pcl::PointCloud<NormalType>);
    scene_keypoints.reset(new pcl::PointCloud<PointType>);
    scene_descriptors.reset(new pcl::PointCloud<DescriptorType>);

}

//
// Model Constructor
//
Model::Model(std::string modelPath, float resolution){
    cloud.reset(new pcl::PointCloud<PointType>);
    model_normals.reset(new pcl::PointCloud<NormalType>);
    model_keypoints.reset(new pcl::PointCloud<PointType>);
    model_descriptors.reset(new pcl::PointCloud<DescriptorType>);
    model_scene_corrs.reset(new pcl::Correspondences());

    // Load Cloud
    std::string folder = modelPath.substr(modelPath.find_last_of("/") + 1);
    std::string modelFile = modelPath + "/" + folder + ".pcd";
    if (pcl::io::loadPCDFile(modelFile, *cloud) < 0) {
        std::cout << "Error loading model cloud." << std::endl;
        return;
    }

    // Load config
    corr_thresh = 0;
    std::string configFile = modelPath + "/" + folder + ".config";
    std::ifstream cFile(configFile);
    if (cFile.is_open())
    {
        std::string line;
        while (getline(cFile, line)) {
            line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
            if(line != "pick_points"){
                if (line[0] == '#' || line.empty())
                    continue;
                auto delimiterPos = line.find("=");
                auto name = line.substr(0, delimiterPos);
                auto value = line.substr(delimiterPos + 1);

                if (name == "model_ss") model_ss = atof(value.c_str()) * resolution;
                else if (name == "rf_rad") rf_rad = atof(value.c_str()) * resolution;
                else if (name == "descr_rad") descr_rad = atof(value.c_str()) * resolution;
                else if (name == "cg_size") cg_size = atof(value.c_str()) * resolution;
                else if (name == "cg_thresh") cg_thresh = atof(value.c_str());
                else if (name == "max_objects") max_objects = atoi(value.c_str());
                else if (name == "out_thresh") out_thresh = atof(value.c_str());
                else if (name == "corr_thresh") corr_thresh = atoi(value.c_str());
            }else{
                while (getline(cFile, line)){
                    float x, y, z;
                    sscanf(line.c_str(), "(%f,%f,%f)", &x, &y, &z);
                    PointT pickPoint(x, y, z);
                    pick_points.push_back(pickPoint);
                }
            }
        }

    } else {
        std::cerr << "Couldn't open config file for reading.\n";
    }


}

void ObjectDetector::LoadModel(std::string modelFile){
    Model *model = new Model(modelFile, resolution);
    models.push_back(model);
}

//
// Calculates resolution of model which is later used to adjust parameters
//
float ObjectDetector::CalculateResolution() {
    resolution = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<PointType> tree;
    tree.setInputCloud(scene);

    std::cout << "Calculating Resolution" << std::endl;

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
void Model::RemoveOutliers() {
    std::cout << "RemoveOutliers" << std::endl;

//    std::cout << "after: " << scene_keypoints->size() << endl;

    sor.setInputCloud(model_keypoints);
    sor.setMeanK(50);
    sor.setStddevMulThresh(out_thresh);
    sor.filter(*model_keypoints);
}

//
// Loads config parameters for scene
//
void ObjectDetector::LoadParams(float scene_ss, float descr_rad, float cg_size, float cg_thresh, float rf_rad, float out_thresh, int num_threads) {
    resolution = CalculateResolution();

    this->scene_ss = scene_ss * resolution;
    this->descr_rad = descr_rad * resolution;
    this->cg_size = cg_size * resolution;
    this->cg_thresh = cg_thresh;
    this->rf_rad = rf_rad * resolution;
    this->out_thresh = out_thresh;
    this->num_threads = num_threads;

//    std::cout << "Model sampling size:    " << this->model_ss << std::endl;
//    std::cout << "Scene sampling size:    " << this->scene_ss << std::endl;
//    std::cout << "LRF support radius:     " << this->rf_rad << std::endl;
//    std::cout << "SHOT descriptor radius: " << this->descr_rad << std::endl;
//    std::cout << "Clustering bin size:    " << this->cg_size << std::endl << std::endl;
}

void ObjectDetector::ProcessScene(){

    norm_est.setKSearch(15);
    norm_est.setInputCloud(scene);
    norm_est.compute(*scene_normals);

    uniform_sampling.setInputCloud(scene);
    uniform_sampling.setRadiusSearch(scene_ss);
    uniform_sampling.filter(*scene_keypoints);
    std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

    sor.setInputCloud(scene_keypoints);
    sor.setMeanK(10);
    sor.setStddevMulThresh(out_thresh);
    sor.filter(*scene_keypoints);

    descr_est.setNumberOfThreads(100);
    descr_est.setRadiusSearch(descr_rad);
    descr_est.setInputCloud(scene_keypoints);
    descr_est.setInputNormals(scene_normals);
    descr_est.setSearchSurface(scene);
    descr_est.compute(*scene_descriptors);

}

//
// Computes normal vectors of all points in model and scene
//
void Model::ComputeNormals() {
    std::cout << "ComputeNormals" << std::endl;

    norm_est.setKSearch(15);
    norm_est.setInputCloud(cloud);
    norm_est.compute(*model_normals);

}

//
//  Downsample Clouds to Extract keypoints
//
void Model::Downsample() {
    std::cout << "Downsample" << std::endl;

    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(model_ss);
    uniform_sampling.filter(*model_keypoints);
    std::cout << "Model total points: " << cloud->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;


}


//
//  Compute Descriptor for keypoints
//
void Model::ComputeDescriptors() {
    std::cout << "ComputeDescriptors" << std::endl;


    descr_est.setNumberOfThreads(100);
    descr_est.setRadiusSearch(descr_rad);
    descr_est.setInputCloud(model_keypoints);
    descr_est.setInputNormals(model_normals);
    descr_est.setSearchSurface(cloud);
    descr_est.compute(*model_descriptors);

}

void ObjectDetector::SearchThread(int i, Model *mod){
    int start, finish, segmentSize;

    segmentSize = scene_descriptors->size() / num_threads;
    start = i * segmentSize;
    finish = start + segmentSize;

    for(int j = start; j < finish; j++){
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        if (!std::isfinite(scene_descriptors->at(j).descriptor[0])) //skipping NaNs
        {
            return;
        }
        int found_neighs = match_search.nearestKSearch(scene_descriptors->at(j), 1, neigh_indices, neigh_sqr_dists);

        //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
        {
            pcl::Correspondence corr(neigh_indices[0], static_cast<int> (j), neigh_sqr_dists[0]);
            mtx.lock();
            mod->model_scene_corrs->push_back(corr);
            mtx.unlock();
        }
    }

}

void ObjectDetector::FindCorrespondences(Model *mod) {

    std::cout << "FindCorrespondences" << std::endl;

    //
    //  Find Model-Scene Correspondences with KdTree
    //
    mod->model_scene_corrs.reset(new pcl::Correspondences());


    match_search.setInputCloud(mod->model_descriptors);

    auto start = std::chrono::steady_clock::now();
    std::vector<std::thread> threads;

    //  For each scene descriptor, find nearest neighbor into the model descriptor cloud and add it to the correspondences vector.
    for(int i = 0; i < num_threads; i++){
        threads.emplace_back(std::thread(&ObjectDetector::SearchThread, this, i, mod));
    }

    for(int i = 0; i < threads.size(); i++){
        threads[i].join();
    }

    auto end = std::chrono::steady_clock::now();

    std::cout << "Correspondences found: " << mod->model_scene_corrs->size() << " time taken: " << std::chrono::duration <double, std::milli> (end-start).count() << " ms" << std::endl;

    //
    //  Compute (Keypoints) Reference Frames for Hough
    //
    pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
    pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles(false);
    rf_est.setRadiusSearch(mod->rf_rad);

    rf_est.setInputCloud(mod->model_keypoints);
    rf_est.setInputNormals(mod->model_normals);
    rf_est.setSearchSurface(mod->cloud);
    rf_est.compute(*model_rf);

    rf_est.setInputCloud(scene_keypoints);
    rf_est.setInputNormals(scene_normals);
    rf_est.setSearchSurface(scene);
    rf_est.compute(*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize(mod->cg_size);
    clusterer.setHoughThreshold(mod->cg_thresh);
    clusterer.setUseInterpolation(true);
    clusterer.setUseDistanceWeight(false);

    clusterer.setInputCloud(mod->model_keypoints);
    clusterer.setInputRf(model_rf);
    clusterer.setSceneCloud(scene_keypoints);
    clusterer.setSceneRf(scene_rf);
    clusterer.setModelSceneCorrespondences(mod->model_scene_corrs);

    clusterer.recognize(mod->rototranslations, mod->clustered_corrs);

    std::cout << "Correspondences Found" << std::endl;
    // Sort matches by # of correspondences,
    // Create transformed model cloud, store as ( # of correspondences, (index, rotated_model) )
    for (std::size_t x = 0; x < mod->rototranslations.size(); ++x) {
        if(mod->clustered_corrs[x].size() >= mod->corr_thresh){
            pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*mod->cloud, *rotated_model, mod->rototranslations[x]);
            mod->bestMatches.insert(std::make_pair(mod->clustered_corrs[x].size(), std::make_pair(x, rotated_model)));
        }
    }
}

void ObjectDetector::DetermineBestMatches(Model *mod) {
    std::cout << "DetermineBestMatches" << std::endl;

    int i = 0;
    int j = 0;
    float dist;
    Eigen::Vector4d diff;



    // Detects duplicate detections
    for (it = mod->bestMatches.rbegin(); it != mod->bestMatches.rend(); it++) {

        Eigen::Vector4d centroid1;
        pcl::compute3DCentroid(*it->second.second, centroid1);

        //std::cout << "Instance i" << i << ": " << centroid1.x() << ",  " << centroid1.y() << ",  " << centroid1.z() << std::endl;

        for (it2 = it; it2 != mod->bestMatches.rend(); it2++) {
            if (j >= mod->max_objects) break;

            Eigen::Vector4d centroid2;
            pcl::compute3DCentroid(*it2->second.second, centroid2);

            // Calculate distance between two translations
            diff = centroid2 - centroid1;

            dist = sqrt(pow(diff.x(), 2) + pow(diff.y(), 2) + pow(diff.z(), 2));
            //std::cout << "Instances " << i << " " << j << ". Distance: " << dist << std::endl;

            if (dist < min_distance && i != j) {
                std::cout << "\nDuplicate Detected\n" << std::endl;
                // Erase duplicate instance and call recursively
                it2 = decltype(it2)(mod->bestMatches.erase(std::next(it2).base()));
                DetermineBestMatches(mod);
                return;
            }

            j++;
        }
        i++;
        j = i;
        if (i >= mod->max_objects) break;
    }
}

//
//  Output results
//
void ObjectDetector::PrintInstances() {
    for(int i = 0; i < models.size(); i++){
        std::cout << "\nModel: " << i << endl;
        int c = 0;
        for (it = models[i]->bestMatches.rbegin(); it != models[i]->bestMatches.rend(); it++) {
            std::cout << "\n    Instance " << c << ":" << std::endl;
            std::cout << "        Correspondences belonging to this instance: " << models[i]->clustered_corrs[it->second.first].size() << std::endl;

            // Print the rotation matrix and translation vector
            Eigen::Matrix3f rotation = models[i]->rototranslations[it->second.first].block<3, 3>(0, 0);
            Eigen::Vector3f translation = models[i]->rototranslations[it->second.first].block<3, 1>(0, 3);

            printf("\n");
            printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
            printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
            printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
            printf("\n");
            printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
            c++;
            if (c >= models[i]->max_objects) break;
        }
    }
}


void                        
showHelp(char* filename)
{
    std::cout << std::endl;
    std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd config_filename.txt" << std::endl << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "     model_ss = val         Model uniform sampling radius" << std::endl;
    std::cout << "     scene_ss = val         Scene uniform sampling radius" << std::endl;
    std::cout << "     rf_rad = val           Reference frame radius" << std::endl;
    std::cout << "     descr_rad = val        Descriptor radius" << std::endl;
    std::cout << "     cg_size = val          Cluster size" << std::endl;
    std::cout << "     max_objects = val      Number of objects to detect" << std::endl;
    std::cout << "     cg_thresh = val        Clustering threshold" << std::endl << std::endl;
    std::cout << "     out_thresh = val       Outlier filtering threshold" << std::endl << std::endl;
}

int main (int argc, char *argv[])
{
	QApplication a (argc, argv);
	
    pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
    float model_ss;
    float scene_ss;
    float rf_rad;
    float descr_rad;
    float cg_size;
    float cg_thresh;
    float out_thresh;
    int max_objects;
    float num_threads = 20;
    bool view_result = true;
    
    if(argc < 3){
        cout << "Usage: ./gpuObjectDetector model.pcd scene.pcd config.txt" << endl;
    	return 0;
    }

    std::string sceneFile = argv[1];
    std::string configFile = argv[2];

   //  Load clouds
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
            else if (name == "out_thresh") out_thresh = atof(value.c_str());
            else if (name == "num_threads") num_threads = atoi(value.c_str());

        }

    } else {
        std::cerr << "Couldn't open config file for reading.\n";
    }

    ObjectDetector *obj = new ObjectDetector(scene);
    obj->LoadParams(scene_ss, descr_rad, cg_size, cg_thresh, rf_rad, out_thresh, num_threads);
    obj->ProcessScene();

    if(view_result){
        PCLViewer w(0, obj);
        w.show ();
        return a.exec ();
    }

    return 0;
}

void ObjectDetector::Detect(){

    std::chrono::steady_clock::time_point t0, tf;
    t0 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t1, t2;


    for(int i = 0; i < models.size(); i++){
        if(models[i] == NULL){
            continue;
        }
        Model *model = models[i];

        t1 = std::chrono::steady_clock::now();
        model->ComputeNormals();
        t2 = std::chrono::steady_clock::now();
        auto CN = std::chrono::duration <double, std::milli> (t2-t1).count();

        t1 = std::chrono::steady_clock::now();
        model->Downsample();
        t2 = std::chrono::steady_clock::now();
        auto DS = std::chrono::duration <double, std::milli> (t2-t1).count();

        t1 = std::chrono::steady_clock::now();
        model->RemoveOutliers();
        t2 = std::chrono::steady_clock::now();
        auto RO = std::chrono::duration <double, std::milli> (t2-t1).count();

        t1 = std::chrono::steady_clock::now();
        model->ComputeDescriptors();
        t2 = std::chrono::steady_clock::now();
        auto CD = std::chrono::duration <double, std::milli> (t2-t1).count();

        t1 = std::chrono::steady_clock::now();
        FindCorrespondences(model);
        t2 = std::chrono::steady_clock::now();
        auto FC = std::chrono::duration <double, std::milli> (t2-t1).count();

        t1 = std::chrono::steady_clock::now();
        DetermineBestMatches(model);
        t2 = std::chrono::steady_clock::now();
        auto DB = std::chrono::duration <double, std::milli> (t2-t1).count();
    }

   
    PrintInstances();

    tf = std::chrono::steady_clock::now();
    auto totalTime = std::chrono::duration <double, std::milli> (tf-t0).count();

//    cout << "\n\nTIMING\nRemoveOutliers: " << RO << "ms\nComputeNormals: " << CN << "ms\nDownsample: " << DS << "ms\nComputeDescriptors: " << CD << "ms\nFindCorrespondences: " << FC << "ms\nDetermineBest: " << DB << endl;
    cout << "Total Time: " << totalTime << endl;
  
}
