#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "objectdetector.h"
#include "Capturer.h"
#include "PointsToRobot.h"
#include <Windows.h>
#include <vector>
#include <thread>
#include <fstream>

using namespace std;

void                        
showHelp(char* filename)
{
    cout << endl;
    cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd config_filename.txt" << endl << endl;
    cout << "Configuration:" << endl;
    cout << "     model_ss = val         Model uniform sampling radius" << endl;
    cout << "     scene_ss = val         Scene uniform sampling radius" << endl;
    cout << "     rf_rad = val           Reference frame radius" << endl;
    cout << "     descr_rad = val        Descriptor radius" << endl;
    cout << "     cg_size = val          Cluster size" << endl;
    cout << "     max_objects = val      Number of objects to detect" << endl;
    cout << "     cg_thresh = val        Clustering threshold" << endl << endl;
    cout << "     out_thresh = val       Outlier filtering threshold" << endl << endl;
}

void getSubdirs(vector<string>& output, const string& path)
{
    WIN32_FIND_DATA findfiledata;
    HANDLE hFind = INVALID_HANDLE_VALUE;

    char fullpath[MAX_PATH];
    GetFullPathName(path.c_str(), MAX_PATH, fullpath, 0);
    string fp(fullpath);

    hFind = FindFirstFile((LPCSTR)(fp + "\\*").c_str(), &findfiledata);
    if (hFind != INVALID_HANDLE_VALUE)
    {
        do
        {
            if ((findfiledata.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0
                && (findfiledata.cFileName[0] != '.'))
            {
                output.push_back(findfiledata.cFileName);
            }
        } while (FindNextFile(hFind, &findfiledata) != 0);
    }
}


int main (int argc, char *argv[])
{
	
    pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
    string modelPath = "../../pcd/";
    float model_ss = 0;
    float scene_ss = 0;
    float rf_rad = 0;
    float descr_rad = 0;
    float cg_size = 0;
    float cg_thresh = 0;
    float out_thresh = 0;
    int max_objects = 0;
    float num_threads = 10;
    bool view_result = true;
    
    if(argc < 3){
        cout << "Usage: ./FeatureDetection -c scene.pcd scene_config.txt" << endl;
    	return 0;
    }

    string flag = argv[1];
    string sceneFile = argv[2];
    string configFile = argv[3];

    if (flag == "-f") {
        //  Load clouds
        if (pcl::io::loadPCDFile(sceneFile, *scene) < 0) {
            cout << "Error loading scene cloud." << endl;
            showHelp(argv[0]);
            return (-1);
        }
    } else {
        Capturer capturer("../../txt/set.yml");
        scene = capturer.Capture();

    }
   


    // Load config
    ifstream cFile(configFile);
    if (cFile.is_open())
    {
        string line;
        while (getline(cFile, line)) {
            line.erase(remove_if(line.begin(), line.end(), isspace), line.end());
            if (line[0] == '#' || line.empty()) continue;

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
        cerr << "Couldn't open config file for reading.\n";
    }

    ObjectDetector *obj = new ObjectDetector(scene);
    obj->LoadParams(scene_ss, descr_rad, cg_size, cg_thresh, rf_rad, out_thresh, num_threads);
    thread sceneViewer(&ObjectDetector::VisualizeResults, obj);

    thread sceneProcessor(&ObjectDetector::ProcessScene, obj); // Process scene and Detect Cylinder in background while collecting user input

    vector<string> models;
    getSubdirs(models, modelPath);
    cout << "Models:" << endl;
    for (int i = 0; i < models.size(); i++) {
        cout << "\t" << models[i] << endl;
    }

    string s;
    while (true) {
        cout << "Add a model('d' when done): ";
        cin >> s;
        if (s == "d") break;

        try {
            obj->LoadModel(modelPath + s);
        }catch(exception e){
            cout << "invalid model" << endl;
        }

    }
    sceneProcessor.join();
    std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << obj->scene_keypoints->size() << std::endl;

    auto sniffPoints = obj->Detect();
    obj->SwitchScene();

    //vector<float> calibration;
    //ifstream calFile("../../txt/EpsonCalibration.txt");
    //if (calFile.is_open())
    //{
    //    string line;
    //    while (getline(calFile, line)) {
    //        calibration.push_back(atof(line.c_str()));
    //    }
    //}

    //PointsToRobot *pointsToRobot = new PointsToRobot(sniffPoints, calibration);
    //pointsToRobot->SendPoints();

    sceneViewer.join();

    return 0;
}

