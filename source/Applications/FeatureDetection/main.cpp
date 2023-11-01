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


int main (int argc, char *argv[]) {
    pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
    vector<PointType> sniffPoints;
    PointsToRobot* pointsToRobot;
    Capturer* capturer;
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
    bool useRobot;
    bool useCamera;
    
    if(argc < 3){
        cout << "Usage: ./FeatureDetection -c scene.pcd scene_config.txt" << endl;
    	return 0;
    }

    string flag = argv[1];
    string flag2 = argv[2];
    string sceneFile = argv[3];
    string configFile = argv[4];

    if (flag2 == "-r") {
        useRobot = true;
    } else {
        useRobot = false;
    }

    // Init robot
    if (useRobot) {
        vector<float> calibration;
        ifstream calFile("../../txt/cal.txt");
        if (calFile.is_open())
        {
            string line;
            while (getline(calFile, line)) {
                calibration.push_back(atof(line.c_str()));
            }
        }

        pointsToRobot = new PointsToRobot(calibration);
    }

    ObjectDetector* obj = new ObjectDetector(); // Object Detector init

    // Init camera and capture/load
    if (flag == "-f") {
        useCamera = false;
        //  Load scene cloud
        if (pcl::io::loadPCDFile(sceneFile, *scene) < 0) {
            cout << "Error loading scene cloud." << endl;
            showHelp(argv[0]);
            return (-1);
        }
    } else {
        useCamera = true;
        // Capture scene cloud
        capturer = new Capturer("../../txt/set.yml");
        scene = capturer->Capture();
    }

    // Load config params
    ifstream cFile(configFile);
    obj->CalculateResolution(scene);
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
    obj->LoadParams(scene_ss, descr_rad, cg_size, cg_thresh, rf_rad, out_thresh, num_threads);

    


    // Pick and Load Models
    vector<string> models;
    getSubdirs(models, modelPath);
    cout << "Models:" << endl;
    for (int i = 0; i < models.size(); i++) {
        cout << "\t" << models[i] << endl;
    }
    string s;
    int o;
    while (true) {
        cout << "Add a model('d' when done): ";
        cin >> s;
        if (s == "d") break;
        cout << "\tExpected # of occurences: ";
        try {
            cin >> o;
            obj->LoadModel(modelPath + s, o);
        }
        catch (exception e) {
            cout << "invalid model" << endl;
        }
    }

    thread sceneViewer(&ObjectDetector::VisualizeResults, obj);

    thread moveRobot;


    while (1) {
        obj->ProcessScene(scene);
        std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << obj->scene_keypoints->size() << std::endl;


        PointType p1 = obj->DetectCylinder();
        sniffPoints.push_back(p1);
        if (useRobot) {
            moveRobot = thread(&PointsToRobot::SendPoints, pointsToRobot, sniffPoints);
        }

        sniffPoints = obj->Detect();
        obj->SwitchScene();
        if(useRobot) moveRobot.join();

        if (useRobot) {
            pointsToRobot->SendPoints(sniffPoints);
        }
            
        Sleep(10000);
        

        obj->ResetAllModels();
        sniffPoints.clear();
        if (useCamera) {
            scene.reset(new pcl::PointCloud<PointType>());
            scene = capturer->Capture();
        }
        obj->SwitchScene();
    }
    

    sceneViewer.join();

    return 0;
}

