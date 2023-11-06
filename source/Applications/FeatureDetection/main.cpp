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
    vector< pair< string, int > > models;
    bool useRobot;
    bool useCamera;
    string sceneFile, sceneConfig, cylConfig;
    
    if(argc < 2){
        cout << "Usage: ./FeatureDetection ../../runconfigs/runconfig.config" << endl;
    	return 0;
    }

    string runConfig = argv[1];

    // Load config params
    ifstream cFile(runConfig);
    if (cFile.is_open())
    {
        string line;
        while (getline(cFile, line)) {
            line.erase(remove_if(line.begin(), line.end(), isspace), line.end());
            if (line[0] == '#' || line.empty()) continue;

            auto delimiterPos = line.find("=");
            auto name = line.substr(0, delimiterPos);
            auto value = line.substr(delimiterPos + 1);

            if (name == "camera") useCamera = (value == "true");
            else if (name == "robot") useRobot = (value == "true");
            else if (name == "cylConfig") cylConfig = value;
            else if (name == "sceneConfig") sceneConfig = value;
            else if (name == "sceneFile") sceneFile = value;
            else if (name == "models") {
                while (getline(cFile, line)) {
                    line.erase(remove_if(line.begin(), line.end(), isspace), line.end());
                    if (line[0] == '#' || line.empty()) continue;

                    auto delimiterPos = line.find("=");
                    auto name = line.substr(0, delimiterPos);
                    auto value = line.substr(delimiterPos + 1);

                    if (name == "endmodels") break;

                    models.push_back(make_pair(name, atoi(value.c_str())));
                }
            }
        }
    } else {
        cerr << "Couldn't open config file for reading.\n";
    }
    cout << useRobot << " " << useCamera << endl;

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
        pointsToRobot->WaitForHome();
    }


    // Init camera and capture/load
    if (!useCamera) {
        //  Load scene cloud
        if (pcl::io::loadPCDFile(sceneFile, *scene) < 0) {
            cout << "Error loading scene cloud." << endl;
            showHelp(argv[0]);
            return (-1);
        }
    } else {
        // Capture scene cloud
        capturer = new Capturer("../../txt/set.yml");
        scene = capturer->Capture();
    }

    ObjectDetector* obj = new ObjectDetector(scene, sceneConfig, cylConfig); // Object Detector init

    // Load Models
    for (int i = 0; i < models.size(); i++) {
        cout << "loading: " << modelPath + models[i].first << " " << models[i].second << endl;
        obj->LoadModel(modelPath + models[i].first, models[i].second);
    }

    thread sceneViewer(&ObjectDetector::VisualizeResults, obj);

    thread moveRobot;


    while (1) {
        obj->LoadScene(scene);

        obj->ProcessSceneCylinder();
        PointType p1 = obj->DetectCylinder();
        obj->SwitchView();
        sniffPoints.push_back(p1);
        if (useRobot) {
            moveRobot = thread(&PointsToRobot::SendPoints, pointsToRobot, sniffPoints);
        }

        obj->ProcessScene();
        std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << obj->scene_keypoints->size() << std::endl;

        sniffPoints = obj->Detect();
        obj->SwitchView();
        if(useRobot) moveRobot.join();

        if (useRobot) {
            pointsToRobot->SendPoints(sniffPoints);
        }
            
        Sleep(4000);
        

        obj->ResetAllModels();
        sniffPoints.clear();
        if (useCamera) {
            scene.reset(new pcl::PointCloud<PointType>());
            scene = capturer->Capture();
        }
        obj->SwitchView();
    }
    

    sceneViewer.join();

    return 0;
}

