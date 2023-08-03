/*


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
        void Detect(float model_ss, float scene_ss, float descr_rad, float cg_size, float cg_thresh, float rf_rad);

        void ComputeNormals();
        void Downsample(float model_ss, float scene_ss);
        void ComputeDescriptors(float descr_rad);
        void FindCorrespondences(float cg_size, float cg_thresh, float rf_rad);

        void DetermineBestMatches(int max_objects);
        void PrintInstances();
        void VisualizeResults();
    private:
        pcl::PointCloud<PointType>::Ptr scene;
        pcl::PointCloud<PointType>::Ptr model;
        pcl::PointCloud<NormalType> model_normals; // MAY NEED TO BE PTR
        pcl::PointCloud<NormalType> scene_normals;
        pcl::PointCloud<PointType> model_keypoints;
        pcl::PointCloud<PointType> scene_keypoints;
        pcl::PointCloud<DescriptorType> model_descriptors;
        pcl::PointCloud<DescriptorType> scene_descriptors;
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
        std::vector<pcl::Correspondences> clustered_corrs;
        std::multimap<size_t, int> bestMatches;
        std::multimap<size_t, int>::reverse_iterator it;
        std::multimap<size_t, int>::reverse_iterator it2;

        int max_objects;
		int min_distance;
        std::string config;


};

ObjectDetector::ObjectDetector(pcl::PointCloud<PointType>::Ptr sceneCloud, pcl::PointCloud<PointType>::Ptr modelCloud, std::string configFile) {
    this->scene = sceneCloud;
    this->model = modelCloud;
    this->config = configFile;
    CalculateResolution();
}

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

void ObjectDetector::ComputeNormals() {
    std::cout << "ComputeNormals" << endl;

    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch(15);
    norm_est.setInputCloud(model);
    norm_est.compute(model_normals);
    norm_est.setInputCloud(scene);
    norm_est.compute(scene_normals);

}

//
//  Downsample Clouds to Extract keypoints
//
void ObjectDetector::Downsample(float model_ss, float scene_ss) {
    std::cout << "Downsample" << endl;

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud(model);
    uniform_sampling.setRadiusSearch(model_ss);
    uniform_sampling.filter(model_keypoints);
    std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints.size() << std::endl;

    uniform_sampling.setInputCloud(scene);
    uniform_sampling.setRadiusSearch(scene_ss);
    uniform_sampling.filter(scene_keypoints);
    std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints.size() << std::endl;

}


//
//  Compute Descriptor for keypoints
//
void ObjectDetector::ComputeDescriptors(float descr_rad) {
    std::cout << "ComputeDescriptors" << endl;

    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch(descr_rad);

    descr_est.setInputCloud(model_keypoints.makeShared());
    descr_est.setInputNormals(model_normals.makeShared());
    descr_est.setSearchSurface(model);
    descr_est.compute(model_descriptors);

    descr_est.setInputCloud(scene_keypoints.makeShared());
    descr_est.setInputNormals(scene_normals.makeShared());
    descr_est.setSearchSurface(scene);
    descr_est.compute(scene_descriptors);
}

void ObjectDetector::FindCorrespondences(float cg_size, float cg_thresh, float rf_rad) {
    std::cout << "FindCorrespondences" << endl;

    //
    //  Find Model-Scene Correspondences with KdTree
    //
    pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud(model_descriptors.makeShared());

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (std::size_t i = 0; i < scene_descriptors.makeShared()->size(); ++i)
    {
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        if (!std::isfinite(scene_descriptors.makeShared()->at(i).descriptor[0])) //skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch(scene_descriptors.makeShared()->at(i), 1, neigh_indices, neigh_sqr_dists);
        if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
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

    rf_est.setInputCloud(model_keypoints.makeShared());
    rf_est.setInputNormals(model_normals.makeShared());
    rf_est.setSearchSurface(model);
    rf_est.compute(*model_rf);

    rf_est.setInputCloud(scene_keypoints.makeShared());
    rf_est.setInputNormals(scene_normals.makeShared());
    rf_est.setSearchSurface(scene);
    rf_est.compute(*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize(cg_size);
    clusterer.setHoughThreshold(cg_thresh);
    clusterer.setUseInterpolation(true);
    clusterer.setUseDistanceWeight(false);

    clusterer.setInputCloud(model_keypoints.makeShared());
    clusterer.setInputRf(model_rf);
    clusterer.setSceneCloud(scene_keypoints.makeShared());
    clusterer.setSceneRf(scene_rf);
    clusterer.setModelSceneCorrespondences(model_scene_corrs);

    clusterer.recognize(rototranslations, clustered_corrs);

    std::cout << "Correspondences Found" << endl;

}

void ObjectDetector::DetermineBestMatches(int max_objects) {
    // 
    // Sort matches by # of correspondences
    //
    for (std::size_t i = 0; i < rototranslations.size(); ++i) {
        bestMatches.insert(std::make_pair(clustered_corrs[i].size(), i));
    }

    int i = 0;
    int j = 0;
    float xdiff, ydiff, zdiff, dist;
    Eigen::Vector3f translation, translation2;

    // Detects duplicate detections
    for (it = bestMatches.rbegin(); it != bestMatches.rend(); it++) {
        translation = rototranslations[it->second].block<3, 1>(0, 3);
        for (it2 = it; it2 != bestMatches.rend(); it2++) {
            if (j >= max_objects) break;
            translation2 = rototranslations[it2->second].block<3, 1>(0, 3);

            // Calculate distance between two translations
            xdiff = translation2.x() - translation.x();
            ydiff = translation2.y() - translation.y();
            zdiff = translation2.z() - translation.z();


            std::cout << "xdiff " << xdiff;
            std::cout << ", ydiff " << ydiff;
            std::cout << ", zdiff " << zdiff;

            //dist = sqrt(pow(xdiff, 2) + pow(ydiff, 2) + pow(zdiff, 2));
            dist = sqrt(pow(xdiff, 2) + pow(ydiff, 2));
            std::cout << ", Instances " << i << " " << j << ". Distance: " << dist << endl;
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
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[it->second].size() << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = rototranslations[it->second].block<3, 3>(0, 0);
        Eigen::Vector3f translation = rototranslations[it->second].block<3, 1>(0, 3);

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
    viewer.addPointCloud(scene, "scene_cloud");

    pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());

    int c = 0;
    for (it = bestMatches.rbegin(); it != bestMatches.rend(); it++) {
        pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*model, *rotated_model, rototranslations[it->second]);

        std::stringstream ss_cloud;
        ss_cloud << "instance" << c;

        pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
        viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

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

        }

    } else {
        std::cerr << "Couldn't open config file for reading.\n";
    }

    ObjectDetector *obj = new ObjectDetector(scene, model, configFile);
    float modelResolution = obj->CalculateResolution();

    if (modelResolution != 0.0f) {
        model_ss *= modelResolution;
        scene_ss *= modelResolution;
        rf_rad *= modelResolution;
        descr_rad *= modelResolution;
        cg_size *= modelResolution;
    }

    obj->RemoveOutliers(true, true);

    obj->ComputeNormals();
    obj->Downsample(model_ss, scene_ss);
    obj->ComputeDescriptors(descr_rad);
    obj->FindCorrespondences(cg_size, cg_thresh, rf_rad);
    obj->DetermineBestMatches(max_objects);
   
    obj->PrintInstances();
    obj->VisualizeResults();

    free(obj);
    return 0;
}