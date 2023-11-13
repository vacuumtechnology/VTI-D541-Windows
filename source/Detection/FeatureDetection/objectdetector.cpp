/*
    Detects Instances of a model pointcloud in a scene pointcloud, takes parameters from a config file
    Usage: ./ObjectDetector scene.pcd sceneconfig.txt
    Eli Wilson - VTI
*/


#include "objectdetector.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::mutex mtx;

//
// Constructor
//
ObjectDetector::ObjectDetector(pcl::PointCloud<PointType>::Ptr sceneCloud, std::string sceneConfig, std::string cylConfig) {
    // Default values
    this->min_distance = 6;
    scene_ss = 0;
    rf_rad = 0;
    descr_rad = 0;
    cg_size = 0;
    cg_thresh = 0;
    out_thresh = 0;
    num_threads = 10;
    norm_weight = .1;
    max_iter = 10000;
    dist_thresh = .5;
    rad_min = 0;
    rad_max = 12;
    switchScene = false;
    this->scene = sceneCloud;
    LoadParams(sceneConfig, cylConfig);
}

//
// Model Constructor
//
Model::Model(std::string pcdFile, std::string configFile) {
    //std::cout << "Creating model, pcd: " << pcdFile << " config: " << configFile << std::endl;
    cloud.reset(new pcl::PointCloud<PointType>);
    model_normals.reset(new pcl::PointCloud<NormalType>);
    model_keypoints.reset(new pcl::PointCloud<PointType>);
    model_descriptors.reset(new pcl::PointCloud<DescriptorType>);
    model_scene_corrs.reset(new pcl::Correspondences());
    model_rf.reset(new pcl::PointCloud<RFType>());
    pick_points.reset(new pcl::PointCloud<PointType>);

    // Load Cloud
    if (pcl::io::loadPCDFile(pcdFile, *cloud) < 0) {
        std::cout << "Error loading model cloud." << std::endl;
        return;
    }

    // Load config
    corr_thresh = 0;
    std::ifstream cFile(configFile);
    if (cFile.is_open())
    {
        std::string line;
        while (getline(cFile, line)) {
            line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
            if (line != "pick_points") {
                if (line[0] == '#' || line.empty())
                    continue;
                auto delimiterPos = line.find("=");
                auto name = line.substr(0, delimiterPos);
                auto value = line.substr(delimiterPos + 1);

                if (name == "model_ss") model_ss = atof(value.c_str());
                else if (name == "rf_rad") rf_rad = atof(value.c_str());
                else if (name == "descr_rad") descr_rad = atof(value.c_str());
                else if (name == "cg_size") cg_size = atof(value.c_str()) ;
                else if (name == "cg_thresh") cg_thresh = atof(value.c_str());
                else if (name == "max_objects") max_objects = atoi(value.c_str());
                else if (name == "out_thresh") out_thresh = atof(value.c_str());
                else if (name == "corr_thresh") corr_thresh = atoi(value.c_str());

            } else {
                while (getline(cFile, line)) {
                    float x, y, z;
                    cout << "pick point: " << line << endl;
                    sscanf(line.c_str(), "(%f,%f,%f)", &x, &y, &z);
                    PointType *pickPoint = new PointType();
                    pickPoint->x = x;
                    pickPoint->y = y;
                    pickPoint->z = z;

                    cout << "Adding pick point: " << pickPoint->x << " " << pickPoint->y << " " << pickPoint->z << endl;
                    pick_points->push_back(*pickPoint);
                }
            }
        }

        std::cout << "Model sampling size:    " << this->model_ss << std::endl;
        std::cout << "LRF support radius:     " << this->rf_rad << std::endl;
        std::cout << "SHOT descriptor radius: " << this->descr_rad << std::endl;
        std::cout << "Clustering bin size:    " << this->cg_size << std::endl << std::endl;

    }
    else {
        std::cerr << "Couldn't open config file for reading.\n";
    }
    cFile.close();
    //std::cout << "Model sampling size:    " << this->model_ss << std::endl;
    //std::cout << "LRF support radius:     " << this->rf_rad << std::endl;
    //std::cout << "SHOT descriptor radius: " << this->descr_rad << std::endl;
    //std::cout << "Clustering bin size:    " << this->cg_size << std::endl << std::endl;
}

void ObjectDetector::LoadModel(std::string modelFile, int occurences) {
    std::string folder = modelFile.substr(modelFile.find_last_of("/") + 1);
    std::string filename = modelFile + "/" + folder;
    ModelGroup* modelGroup = new ModelGroup;
    Model* model = new Model(filename + ".pcd", filename + ".config");
    modelGroup->max_objects = occurences;
    model->name = filename;
    model->Process();
    modelGroup->models.push_back(model);
    modelGroup->size = 1;

    int i = 2;
    struct stat buffer;
    while (true) {
        filename = modelFile + "/" + folder + std::to_string(i);
        if (stat((filename + ".pcd").c_str(), &buffer) == 0) {
            Model* model = new Model(filename + ".pcd", filename + ".config");
            model->name = filename;
            model->Process();
            modelGroup->models.push_back(model);
            modelGroup->size++;
        }
        else {
            break;
        }
        i++;
    }

    modelGroups.push_back(modelGroup);
}

void Model::Process() {
    ComputeNormals();

    Downsample();

    RemoveOutliers();

    ComputeDescriptors();
    std::cout << "Model Processed." << std::endl << std::endl;
}

//
// Calculates resolution of model which is later used to adjust parameters
//
float ObjectDetector::CalculateResolution(pcl::PointCloud<PointType>::Ptr sceneCloud) {
    resolution = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<PointType> tree;
    tree.setInputCloud(sceneCloud);

    std::cout << "Calculating Resolution" << std::endl;

    for (std::size_t i = 0; i < sceneCloud->size(); ++i)
    {
        if (!std::isfinite((*sceneCloud)[i].x))
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


    //    std::cout << "Model sampling size:    " << this->model_ss << std::endl;
    //    std::cout << "Scene sampling size:    " << this->scene_ss << std::endl;
    //    std::cout << "LRF support radius:     " << this->rf_rad << std::endl;
    //    std::cout << "SHOT descriptor radius: " << this->descr_rad << std::endl;
    //    std::cout << "Clustering bin size:    " << this->cg_size << std::endl << std::endl;

    return resolution;
}


//
// Removes Statistical Outliers to improve accuracy and decrease number of points to be processed
//
void Model::RemoveOutliers() {
    std::cout << "RemoveOutliers" << std::endl;


    sor.setInputCloud(model_keypoints);
    sor.setMeanK(50);
    sor.setStddevMulThresh(out_thresh);
    sor.filter(*model_keypoints);
    //std::cout << "after: " << model_keypoints->size() << endl;
}

//
// Loads config parameters for scene
//
void ObjectDetector::LoadParams(std::string sceneConfig, std::string cylConfig) {

    // Load config params
    ifstream cFile(sceneConfig);
    if (cFile.is_open())
    {
        std::string line;
        while (getline(cFile, line)) {
            line.erase(remove_if(line.begin(), line.end(), isspace), line.end());
            if (line[0] == '#' || line.empty()) continue;

            auto delimiterPos = line.find("=");
            auto name = line.substr(0, delimiterPos);
            auto value = line.substr(delimiterPos + 1);

            if (name == "scene_ss") scene_ss = atof(value.c_str());
            else if (name == "rf_rad") rf_rad = atof(value.c_str());
            else if (name == "descr_rad") descr_rad = atof(value.c_str());
            else if (name == "cg_size") cg_size = atof(value.c_str());
            else if (name == "cg_thresh") cg_thresh = atof(value.c_str());
            else if (name == "out_thresh") out_thresh = atof(value.c_str());
            else if (name == "num_threads") num_threads = atoi(value.c_str());
        }
    } else {
        cerr << "Couldn't open config file for reading.\n";
    }
    cFile.close();

    std::cout << "Scene sampling size:    " << this->scene_ss << std::endl;
    std::cout << "LRF support radius:     " << this->rf_rad << std::endl;
    std::cout << "SHOT descriptor radius: " << this->descr_rad << std::endl;
    std::cout << "Clustering bin size:    " << this->cg_size << std::endl << std::endl;

    // Load config params
    ifstream cFile2(cylConfig);
    if (cFile2.is_open())
    {
        std::string line;
        while (getline(cFile2, line)) {
            line.erase(remove_if(line.begin(), line.end(), isspace), line.end());
            if (line[0] == '#' || line.empty()) continue;

            auto delimiterPos = line.find("=");
            auto name = line.substr(0, delimiterPos);
            auto value = line.substr(delimiterPos + 1);

            if (name == "norm_weight") norm_weight = atof(value.c_str());
            else if (name == "max_iter") max_iter = atoi(value.c_str());
            else if (name == "dist_thresh") dist_thresh = atof(value.c_str());
            else if (name == "rad_min") rad_min = atoi(value.c_str());
            else if (name == "rad_max") rad_max = atoi(value.c_str());
        }
    } else {
        cerr << "Couldn't open config file for reading.\n";
    }
    cFile2.close();
}

void ObjectDetector::ResetAllModels() {
    sniffPoints.clear();
    for (int i = 0; i < modelGroups.size(); i++) {
        ResetModels(modelGroups[i]);
    }
}

void ObjectDetector::ResetModels(ModelGroup *modGroup) {

    for (int j = 0; j < modGroup->size; j++) {
        if (modGroup->models[j] == NULL) {
            continue;
        }
        Model* model = modGroup->models[j];

        model->model_scene_corrs.reset(new pcl::Correspondences());
        model->rototranslations.clear();
        model->clustered_corrs.clear();
    }
    for (it = modGroup->bestMatches.rbegin(); it != modGroup->bestMatches.rend(); it++) {
        delete(it->second);
    }
    modGroup->bestMatches.clear();
}

void ObjectDetector::LoadScene(pcl::PointCloud<PointType>::Ptr sceneCloud) {
    this->scene = sceneCloud;
    sniffPointCloud.reset(new pcl::PointCloud<PointType>);
    scene_normals.reset(new pcl::PointCloud<NormalType>);
    scene_keypoints.reset(new pcl::PointCloud<PointType>);
    scene_descriptors.reset(new pcl::PointCloud<DescriptorType>);
    scene_rf.reset(new pcl::PointCloud<RFType>());
    cyl_normals.reset(new pcl::PointCloud<NormalType>);

}

void ObjectDetector::ProcessSceneCylinder() {
    std::cout << "Process Scene Cylinder" << std::endl;

    uniform_sampling.setInputCloud(scene);
    uniform_sampling.setRadiusSearch(scene_ss);
    uniform_sampling.filter(*scene_keypoints);

    // Estimate point normals
    cyl_norm.setInputCloud(scene_keypoints);
    cyl_norm.setKSearch(50);
    cyl_norm.compute(*cyl_normals);


}

void ObjectDetector::ProcessScene() {
    std::cout << "Processing Scene" << std::endl;
    norm_est.setNumberOfThreads(num_threads);
    norm_est.setKSearch(15);
    norm_est.setInputCloud(scene);
    norm_est.compute(*scene_normals);

    uniform_sampling.setInputCloud(scene);
    uniform_sampling.setRadiusSearch(scene_ss);
    uniform_sampling.filter(*scene_keypoints);

    sor.setInputCloud(scene_keypoints);
    sor.setMeanK(10);
    sor.setStddevMulThresh(out_thresh);
    sor.filter(*scene_keypoints);
    //std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

    descr_est.reset(new pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType>);
    descr_est->setNumberOfThreads(num_threads);
    descr_est->setRadiusSearch(descr_rad);
    descr_est->setInputCloud(scene_keypoints);
    descr_est->setInputNormals(scene_normals);
    descr_est->setSearchSurface(scene);
    descr_est->compute(*scene_descriptors);

    rf_est.setFindHoles(false);
    rf_est.setRadiusSearch(rf_rad);
    rf_est.setInputCloud(scene_keypoints);
    rf_est.setInputNormals(scene_normals);
    rf_est.setSearchSurface(scene);
    rf_est.compute(*scene_rf);

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

void ObjectDetector::SearchThread(int i, Model* mod) {
    int start, finish, segmentSize;

    segmentSize = scene_descriptors->size() / num_threads;
    start = i * segmentSize;
    finish = start + segmentSize;

    for (int j = start; j < finish; j++) {
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

void ObjectDetector::FindCorrespondences(Model* mod) {

    std::cout << "FindCorrespondences" << std::endl;

    //
    //  Find Model-Scene Correspondences with KdTree
    //


    match_search.setInputCloud(mod->model_descriptors);

    auto start = std::chrono::steady_clock::now();
    std::vector<std::thread> threads;

    //  For each scene descriptor, find nearest neighbor into the model descriptor cloud and add it to the correspondences vector.
    for (int i = 0; i < num_threads; i++) {
        threads.emplace_back(std::thread(&ObjectDetector::SearchThread, this, i, mod));
    }

    for (int i = 0; i < threads.size(); i++) {
        threads[i].join();
    }

    auto end = std::chrono::steady_clock::now();

    std::cout << "Correspondences found: " << mod->model_scene_corrs->size() << " time taken: " << std::chrono::duration <double, std::milli>(end - start).count() << " ms" << std::endl;

    //
    //  Compute (Keypoints) Reference Frames for Hough
    //

    rf_est.setFindHoles(false);
    rf_est.setRadiusSearch(mod->rf_rad);

    rf_est.setInputCloud(mod->model_keypoints);
    rf_est.setInputNormals(mod->model_normals);
    rf_est.setSearchSurface(mod->cloud);
    rf_est.compute(*mod->model_rf);


    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize(mod->cg_size);
    clusterer.setHoughThreshold(mod->cg_thresh);
    clusterer.setUseInterpolation(true);
    clusterer.setUseDistanceWeight(false);

    clusterer.setInputCloud(mod->model_keypoints);
    clusterer.setInputRf(mod->model_rf);
    clusterer.setSceneCloud(scene_keypoints);
    clusterer.setSceneRf(scene_rf);
    clusterer.setModelSceneCorrespondences(mod->model_scene_corrs);

    //for (int i = 0; i < 10; i++) clusterer.train();
    clusterer.recognize(mod->rototranslations, mod->clustered_corrs);
    std::cout << "Correspondences Found" << std::endl;

}

// Sort matches by # of correspondences,
// Create transformed model cloud, store as ( # of correspondences, (index, rotated_model) )
void ObjectDetector::SortMatches(ModelGroup* modGroup) {
    Match* match;
    int correspondences;

    for (int i = 0; i < modGroup->size; i++) {
        Model* mod = modGroup->models[i];

        for (std::size_t x = 0; x < mod->rototranslations.size(); ++x) {
            correspondences = mod->clustered_corrs[x].size();

            if (correspondences >= mod->corr_thresh) {
                match = new Match;
                match->rotated_model.reset(new pcl::PointCloud<PointType>());
                match->transformedPickPoints.reset(new pcl::PointCloud<PointType>());
                match->model = mod;
                match->rototranslation = mod->rototranslations[x];
                match->correspondences = mod->clustered_corrs[x].size();
                Eigen::Vector3f translationVec = match->rototranslation.block<3, 1>(0, 3);
                pcl::transformPointCloud(*mod->cloud, *match->rotated_model, match->rototranslation);
                pcl::transformPointCloud(*mod->pick_points, *match->transformedPickPoints, match->rototranslation);
                modGroup->bestMatches.insert(std::make_pair(match->correspondences, match));

            }
        }
    }
}

bool ObjectDetector::CheckUnmoved(Eigen::Matrix3f rotation, Eigen::Vector3f translation) {
    for (int i = 0; i < 3; i++) {
        if (translation(i) != 0.0) return false;
        for (int j = 0; j < 3; j++) {
            if (i == j) {
                if (rotation(i, j) != 1.0) return false;
            } else {
                if (rotation(i, j) != 0.0) return false;
            }
        }
    }
    return true;
}

// Rejects bad matches
bool ObjectDetector::DetermineBestMatches(ModelGroup* modGroup) {
    std::cout << "DetermineBestMatches" << std::endl;
    
    int i = 0;
    int j = 0;
    float dist;
    Eigen::Vector4d diff;
    

    for (it = modGroup->bestMatches.rbegin(); it != modGroup->bestMatches.rend(); it++) {
        Eigen::Vector4d centroid1;
        pcl::compute3DCentroid(*it->second->rotated_model, centroid1);

        //std::cout << "Instance i" << i << ": " << centroid1.x() << ",  " << centroid1.y() << ",  " << centroid1.z() << std::endl;

        // Rejects invalid rotations
        Eigen::Matrix3f rotation = it->second->rototranslation.block<3, 3>(0, 0);
        Eigen::Vector3f translation = it->second->rototranslation.block<3, 1>(0, 3);
        //printf("\n");
        //printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
        //printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
        //printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
        //printf("\n");
        //printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
        if (/*rotation(0, 0) < .9 || rotation(1, 1) < .9 || rotation(2, 2) < .9 || */ CheckUnmoved(rotation, translation)) {
            std::cout << "\tInvalid Rotation Detected\n" << std::endl;
            // Erase duplicate instance and call recursively
            it = decltype(it)(modGroup->bestMatches.erase(std::next(it).base()));
            
            return DetermineBestMatches(modGroup);
        }


        // Rejects duplicate detections
        for (it2 = it; it2 != modGroup->bestMatches.rend(); it2++) {
            if (j >= modGroup->max_objects) break;

            Eigen::Vector4d centroid2;
            pcl::compute3DCentroid(*it2->second->rotated_model, centroid2);

            // Calculate distance between two translations
            diff = centroid2 - centroid1;

            dist = sqrt(pow(diff.x(), 2) + pow(diff.y(), 2) + pow(diff.z(), 2));
            //std::cout << "Instances " << i << " " << j << ". Distance: " << dist << std::endl;

            if (dist < min_distance && i != j) {
                std::cout << "\tDuplicate Detected\n" << std::endl;
                // Erase duplicate instance and call recursively
                it2 = decltype(it2)(modGroup->bestMatches.erase(std::next(it2).base()));
                
                return DetermineBestMatches(modGroup);
            }

            j++;
        }
        i++;
        j = i;
        if (i >= modGroup->max_objects) break;
    }

    if (modGroup->bestMatches.size() < modGroup->max_objects) return false;

    // Add transformed pick points to vector of all pick points
    i = 0;
    for (it = modGroup->bestMatches.rbegin(); it != modGroup->bestMatches.rend(); it++) {
        for (int j = 0; j < it->second->transformedPickPoints->points.size(); j++) {
            sniffPoints.push_back(it->second->transformedPickPoints->points[j]);
        }
        *sniffPointCloud += *it->second->transformedPickPoints;

        i++;
        if (i >= modGroup->max_objects) break;
    }
    return true;
}

//
//  Output results
//
void ObjectDetector::PrintInstances() {
    for (int j = 0; j < modelGroups.size(); j++) {
        std::cout << "\nModelGroup: " << j << endl;
        int c = 0;
        for (it = modelGroups[j]->bestMatches.rbegin(); it != modelGroups[j]->bestMatches.rend(); it++) {
            std::cout << "\n    Instance " << c << ":" << std::endl;
            std::cout << "        Correspondences belonging to this instance: " << it->second->correspondences << std::endl;
            Model* model = it->second->model;
            // Print the rotation matrix and translation vector
                Eigen::Matrix3f rotation = it->second->rototranslation.block<3, 3>(0, 0);
                Eigen::Vector3f translation = it->second->rototranslation.block<3, 1>(0, 3);

                printf("\n");
                printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
                printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
                printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
                printf("\n");
                printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
                c++;
            if (c >= modelGroups[j]->max_objects) break;
        }
    }
    std::cout << "Pick Points: " << endl;
    for (int i = 0; i < sniffPoints.size(); i++) {
        cout << sniffPoints[i].x << " " << sniffPoints[i].y << " " << sniffPoints[i].z << endl;
    }
}

std::vector<PointType> ObjectDetector::Detect(){

    std::cout << "groups: " << modelGroups.size() << std::endl;
    for(int i = 0; i < modelGroups.size(); i++){
        int c = 0;
        while (true) {
            std::cout << "models: " << modelGroups[i]->size << std::endl;
            std::cout << "expected: " << modelGroups[i]->max_objects << std::endl;
            for(int j = 0; j < modelGroups[i]->size; j++){
                if(modelGroups[i]->models[j] == NULL){
                    continue;
                }
                Model *model = modelGroups[i]->models[j];

                std::cout << "\nDetecting model: " << model->name << endl;

                FindCorrespondences(model);
            }

            SortMatches(modelGroups[i]);

            if (DetermineBestMatches(modelGroups[i])) break; // break if correct number of objects found
            if (c > 4) exit(0); // Give up if failed 5 times in a row
            std::cout << "Expected objects mismatch. Redetecting." << endl;
            ResetModels(modelGroups[i]);
            c++;
        }
        
    }

   
    PrintInstances();

    return sniffPoints;
  
}

PointType ObjectDetector::DetectCylinder() {
    pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ExtractIndices<PointType> extract;
    cloud_cylinder.reset(new pcl::PointCloud<PointType>());
    std::map<float, PointType> cyl_map;
    std::map<float, PointType> cyl_edge_map;

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(norm_weight);
    seg.setMaxIterations(max_iter);
    seg.setDistanceThreshold(dist_thresh);
    seg.setRadiusLimits(rad_min, rad_max);
    seg.setInputCloud(scene_keypoints);
    seg.setInputNormals(cyl_normals);

    int i = 0;
    while (true) {
        // Obtain the cylinder inliers and coefficients
        seg.segment(*inliers_cylinder, *coefficients_cylinder);
        std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

        // Write the cylinder inliers to disk
        extract.setInputCloud(scene_keypoints);
        extract.setIndices(inliers_cylinder);
        extract.setNegative(false);
        extract.filter(*cloud_cylinder);
        if (cloud_cylinder->points.empty()) {
            std::cerr << "Can't find the cylindrical component." << std::endl;
            if (i > 4) exit(0);
            i++;
        } else break;
    }
        

    // Sort cylinder points by position on x axis
    for (int i = 0; i < cloud_cylinder->points.size(); i++) {
        cyl_map.insert(std::make_pair(cloud_cylinder->points[i].x, cloud_cylinder->points[i]));
    }

    // Sort 50 least x points by z value
    std::map<float, PointType>::iterator it;
    std::map<float, PointType>::reverse_iterator it2;
    int c = 0;
    for (it2 = cyl_map.rbegin(); it2 != cyl_map.rend(); it2++) {
        cyl_edge_map.insert(std::make_pair(it2->second.z, it2->second));
        //    std::cout << "x: " << it->second.x << " y: " << it->second.y << " z: " << it->second.z << std::endl;
        if (c >= 49) break;
        c++;
    }


    // Subtract radius from z val of point to (hopefully) get center of cylinder endcap
    // Offset pick point 10 units away from end of manifold
    PointType p = cyl_edge_map.begin()->second;
    p.z += coefficients_cylinder->values[6];
    p.x += 10;
    //sniffPoints.push_back(p);
    sniffPointCloud->push_back(p);

    return p;
}

// Switch from scene preview to results view
void ObjectDetector::SwitchView() {
    switchScene = true;
}

//
//  Visualization
//
int ObjectDetector::VisualizeResults() {
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->getRenderWindow()->GlobalWarningDisplayOff();
    viewer->setSize(900, 1000);
    viewer->setBackgroundColor(.3, .3, .3);
    viewer->setCameraPosition(0, 0, -50, 0, -1, 0);

    while (true) {

        // Show just scene cloud
        viewer->addPointCloud(scene, "scene_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene_cloud");
        viewer->resetCamera();

        while (!viewer->wasStopped() && !switchScene)
        {
            viewer->spinOnce();
        }
        switchScene = false;

        // Show cylinder cloud
        pcl::visualization::PointCloudColorHandlerCustom<PointType> cylinder_model_color_handler(cloud_cylinder, 65, 72, 145);
        viewer->addPointCloud(cloud_cylinder, cylinder_model_color_handler, "cyl_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cyl_cloud");

        pcl::visualization::PointCloudColorHandlerCustom<PointType> pick_point_color_handler(sniffPointCloud, 78, 163, 49);
        viewer->addPointCloud(sniffPointCloud, pick_point_color_handler, "sniff");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sniff");

        while (!viewer->wasStopped() && !switchScene)
        {
            viewer->spinOnce();
        }
        switchScene = false;

        // Show all detected models
        pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());
        viewer->updatePointCloud(sniffPointCloud, pick_point_color_handler, "sniff");

        for (int i = 0; i < modelGroups.size(); i++) {
            int c = 0;
            std::cout << "matches: " << modelGroups[i]->bestMatches.size() << "\n\n" << std::endl;
            for (it = modelGroups[i]->bestMatches.rbegin(); it != modelGroups[i]->bestMatches.rend(); it++) {

                Model* mod = it->second->model;
                pcl::PointCloud<PointType>::Ptr rotated_model = it->second->rotated_model;


                std::stringstream ss_cloud;
                ss_cloud << "instance" << i << "." << c;

                std::stringstream ss_cloud2;
                ss_cloud2 << "pickpoints" << i << "." << c;

                pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 190, 190, 49);
                viewer->addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ss_cloud.str());

                c++;
                if (c >= modelGroups[i]->max_objects) break;
            }
        }

        while (!viewer->wasStopped() && !switchScene)
        {
            viewer->spinOnce();
        }
        switchScene = false;
        viewer->removeAllPointClouds();
    }
    
    return 1;
}

