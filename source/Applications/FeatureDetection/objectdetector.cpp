/*
    Detects Instances of a model pointcloud in a scene pointcloud, takes parameters from a config file
    Usage: ./ObjectDetector scene.pcd sceneconfig.txt
        - use "find model" button to select folder with model pointcloud and model config file
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
ObjectDetector::ObjectDetector(pcl::PointCloud<PointType>::Ptr sceneCloud) {
    this->scene = sceneCloud;
    this->min_distance = 6;
    scene_normals.reset(new pcl::PointCloud<NormalType>);
    scene_keypoints.reset(new pcl::PointCloud<PointType>);
    scene_descriptors.reset(new pcl::PointCloud<DescriptorType>);
    scene_rf.reset(new pcl::PointCloud<RFType>());
}

//
// Model Constructor
//
Model::Model(std::string pcdFile, std::string configFile, float resolution) {
    cloud.reset(new pcl::PointCloud<PointType>);
    model_normals.reset(new pcl::PointCloud<NormalType>);
    model_keypoints.reset(new pcl::PointCloud<PointType>);
    model_descriptors.reset(new pcl::PointCloud<DescriptorType>);
    model_scene_corrs.reset(new pcl::Correspondences());
    model_rf.reset(new pcl::PointCloud<RFType>());

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

                if (name == "model_ss") model_ss = atof(value.c_str()) * resolution;
                else if (name == "rf_rad") rf_rad = atof(value.c_str()) * resolution;
                else if (name == "descr_rad") descr_rad = atof(value.c_str()) * resolution;
                else if (name == "cg_size") cg_size = atof(value.c_str()) * resolution;
                else if (name == "cg_thresh") cg_thresh = atof(value.c_str());
                else if (name == "max_objects") max_objects = atoi(value.c_str());
                else if (name == "out_thresh") out_thresh = atof(value.c_str());
                else if (name == "corr_thresh") corr_thresh = atoi(value.c_str());
            }
            else {
                while (getline(cFile, line)) {
                    float x, y, z;
                    sscanf(line.c_str(), "(%f,%f,%f)", &x, &y, &z);
                    PointType pickPoint(x, y, z);
                    pick_points.push_back(pickPoint);
                }
            }
        }

    }
    else {
        std::cerr << "Couldn't open config file for reading.\n";
    }
}

void ObjectDetector::ScenePreview() {
    previewer.reset(new pcl::visualization::PCLVisualizer("previewer"));
    //cout << "here" << endl;
    //std::this_thread::sleep_for(std::chrono::seconds(10));
    previewer->setBackgroundColor(.2, .2, .2);
    previewer->addPointCloud(scene, "scene_cloud");
    previewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene_cloud");
    previewer->setCameraPosition(0, 0, -50, 0, -1, 0);

    previewer->resetCamera();

    while (!previewer->wasStopped() && !closeScene)
    {
        previewer->spinOnce();
    }
}

void ObjectDetector::CloseScenePreview() {
    closeScene = true;
}

void ObjectDetector::LoadModel(std::string modelFile) {
    std::string folder = modelFile.substr(modelFile.find_last_of("/") + 1);
    std::string filename = modelFile + "/" + folder;
    ModelGroup* modelGroup = new ModelGroup;
    Model* model = new Model(filename + ".pcd", filename + ".config", resolution);
    modelGroup->max_objects = model->max_objects;
    modelGroup->models.push_back(model);
    modelGroup->size = 1;

    int i = 2;
    struct stat buffer;
    while (true) {
        filename = modelFile + "/" + folder + std::to_string(i);
        if (stat((filename + ".pcd").c_str(), &buffer) == 0) {
            Model* model = new Model(filename + ".pcd", filename + ".config", resolution);
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


void ObjectDetector::ProcessScene() {

    norm_est.setKSearch(15);
    norm_est.setInputCloud(scene);
    norm_est.compute(*scene_normals);



    uniform_sampling.setInputCloud(scene);
    uniform_sampling.setRadiusSearch(scene_ss);
    uniform_sampling.filter(*scene_keypoints);
    std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

    /*sor.setInputCloud(scene_keypoints);
    sor.setMeanK(10);
    sor.setStddevMulThresh(out_thresh);
    sor.filter(*scene_keypoints);
    cout << "here" << endl;*/

    //descr_est.setNumberOfThreads(10);
    descr_est.setRadiusSearch(descr_rad);
    descr_est.setInputCloud(scene_keypoints);
    descr_est.setInputNormals(scene_normals);
    descr_est.setSearchSurface(scene);
    descr_est.compute(*scene_descriptors);


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

    clusterer.recognize(mod->rototranslations, mod->clustered_corrs);

    std::cout << "Correspondences Found" << std::endl;

}

void ObjectDetector::SortMatches(ModelGroup* modGroup) {
    // Sort matches by # of correspondences,
    // Create transformed model cloud, store as ( # of correspondences, (index, rotated_model) )
    for (int i = 0; i < modGroup->size; i++) {
        Model* mod = modGroup->models[i];
        for (std::size_t x = 0; x < mod->rototranslations.size(); ++x) {
            if (mod->clustered_corrs[x].size() >= mod->corr_thresh) {
                pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
                pcl::transformPointCloud(*mod->cloud, *rotated_model, mod->rototranslations[x]);
                modGroup->bestMatches.insert(std::make_pair(mod->clustered_corrs[x].size(), std::make_pair(x, rotated_model)));
            }
        }
    }
}

void ObjectDetector::DetermineBestMatches(ModelGroup* modGroup) {
    std::cout << "DetermineBestMatches" << std::endl;

    int i = 0;
    int j = 0;
    float dist;
    Eigen::Vector4d diff;


    // Detects duplicate detections
    for (it = modGroup->bestMatches.rbegin(); it != modGroup->bestMatches.rend(); it++) {

        Eigen::Vector4d centroid1;
        pcl::compute3DCentroid(*it->second.second, centroid1);

        std::cout << "Instance i" << i << ": " << centroid1.x() << ",  " << centroid1.y() << ",  " << centroid1.z() << std::endl;

        for (it2 = it; it2 != modGroup->bestMatches.rend(); it2++) {
            if (j >= modGroup->max_objects) break;

            Eigen::Vector4d centroid2;
            pcl::compute3DCentroid(*it2->second.second, centroid2);

            // Calculate distance between two translations
            diff = centroid2 - centroid1;

            dist = sqrt(pow(diff.x(), 2) + pow(diff.y(), 2) + pow(diff.z(), 2));
            std::cout << "Instances " << i << " " << j << ". Distance: " << dist << std::endl;

            if (dist < min_distance && i != j) {
                std::cout << "\nDuplicate Detected\n" << std::endl;
                // Erase duplicate instance and call recursively
                it2 = decltype(it2)(modGroup->bestMatches.erase(std::next(it2).base()));
                DetermineBestMatches(modGroup);
                return;
            }

            j++;
        }
        i++;
        j = i;
        if (i >= modGroup->max_objects) break;
    }
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
            std::cout << "        Correspondences belonging to this instance: " << it->first << std::endl;

            // Print the rotation matrix and translation vector
//                Eigen::Matrix3f rotation = models[i]->rototranslations[it->second.first].block<3, 3>(0, 0);
//                Eigen::Vector3f translation = models[i]->rototranslations[it->second.first].block<3, 1>(0, 3);

//                printf("\n");
//                printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
//                printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
//                printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
//                printf("\n");
//                printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
//                c++;
            if (c >= modelGroups[j]->max_objects) break;
        }
    }
}

void ObjectDetector::Detect(){

    std::chrono::steady_clock::time_point t0, tf;
    t0 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t1, t2;

    std::cout << "groups: " << modelGroups.size() << std::endl;
    for(int i = 0; i < modelGroups.size(); i++){
        std::cout << "models: " << modelGroups[i]->size << std::endl;
        for(int j = 0; j < modelGroups[i]->size; j++){
            if(modelGroups[i]->models[j] == NULL){
                continue;
            }
            Model *model = modelGroups[i]->models[j];

            model->ComputeNormals();
            cout << "here" << endl;

            model->Downsample();
            cout << "here" << endl;

            //t1 = std::chrono::steady_clock::now();
            //model->RemoveOutliers();
            //t2 = std::chrono::steady_clock::now();
            //auto RO = std::chrono::duration <double, std::milli> (t2-t1).count();

            model->ComputeDescriptors();
            cout << "here" << endl;

            FindCorrespondences(model);
            cout << "done " << j << endl;
        }

        SortMatches(modelGroups[i]);

        DetermineBestMatches(modelGroups[i]);
        auto DB = std::chrono::duration <double, std::milli> (t2-t1).count();

    }

   
    PrintInstances();

    tf = std::chrono::steady_clock::now();
    auto totalTime = std::chrono::duration <double, std::milli> (tf-t0).count();

//    cout << "\n\nTIMING\nRemoveOutliers: " << RO << "ms\nComputeNormals: " << CN << "ms\nDownsample: " << DS << "ms\nComputeDescriptors: " << CD << "ms\nFindCorrespondences: " << FC << "ms\nDetermineBest: " << DB << endl;
    cout << "Total Time: " << totalTime << endl;
  
}

//
//  Visualization
//
int ObjectDetector::VisualizeResults() {
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->setBackgroundColor(.3, .3, .3);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->setCameraPosition(0, 0, -50, 0, -1, 0);
    viewer->resetCamera();

    pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());
    std::multimap<size_t, std::pair <int, pcl::PointCloud<PointType>::Ptr>>::reverse_iterator it;

    std::ofstream myfile;
    myfile.open("txt/points.txt");
    for (int i = 0; i < modelGroups.size(); i++) {
        int c = 0;
        std::cout << "matches: " << modelGroups[i]->bestMatches.size();
        for (it = modelGroups[i]->bestMatches.rbegin(); it != modelGroups[i]->bestMatches.rend(); it++) {

            Eigen::Vector4d centroid1;
            pcl::compute3DCentroid(*it->second.second, centroid1);


            myfile << centroid1.x() << " " << centroid1.y() << " " << centroid1.z() << endl;


            std::stringstream ss_cloud;
            ss_cloud << "instance" << i << "." << c;

            pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(it->second.second, 219, 66, 35);
            viewer->addPointCloud(it->second.second, rotated_model_color_handler, ss_cloud.str());
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss_cloud.str());
            c++;
            if (c >= modelGroups[i]->max_objects) break;
        }
    }
    myfile.close();


    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return 1;
}

