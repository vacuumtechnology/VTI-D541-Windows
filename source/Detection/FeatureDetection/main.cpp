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
    cout << "Usage: ./FeatureDetection ../../runconfigs/runconfig.config" << endl << endl;
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
    string sceneFile, sceneConfig, cameraConfig;
    string cylConfig = "";
    string calType = "offset";
    string calFilename = "../../txt/cal.txt";
    vector<float> transform;
    bool findCylinder = false;
    
    if(argc < 2){
        showHelp(argv[0]);
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
            else if (name == "cameraConfig") cameraConfig = value;
            else if (name == "calType") calType = value;
            else if (name == "calFilename") calFilename = value;
            else if (name == "models") {
                while (getline(cFile, line)) {
                    line.erase(remove_if(line.begin(), line.end(), isspace), line.end());
                    if (line[0] == '#' || line.empty()) continue;

                    if (line == "endcap") {
                        findCylinder = true;
                        continue;
                    }

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
    cFile.close();
    cout << useRobot << " " << useCamera << endl;

    // Init robot
    if (useRobot) {
        if (calType == "offset") {
            cout << "calibration offset" << endl;

            vector<float> calibration;
            ifstream calFile(calFilename);
            if (calFile.is_open())
            {
                string line;
                while (getline(calFile, line)) {
                    calibration.push_back(atof(line.c_str()));
                }
            }
            pointsToRobot = new PointsToRobot(calibration, false);

        } else if (calType == "transform") {
            ifstream calFile(calFilename);
            if (calFile.is_open())
            {
                float val;
                cout << "calibration transform" << endl;
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 4; j++) {
                        if (calFile >> val) {
                            transform.push_back(val);
                            //transform(i, j) = val;
                            cout << val << endl;
                        } else {
                            cerr << "error reading robot calibration" << endl;
                            exit(0);
                        }
                    }
                }
            }
            pointsToRobot = new PointsToRobot(transform, true);
        }
        
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
        capturer = new Capturer(cameraConfig);
        scene = capturer->Capture();
    }

    // Object Detector init
    ObjectDetector* obj;
    if (findCylinder) {
        obj = new ObjectDetector(scene, sceneConfig, cylConfig);
    } else {
        obj = new ObjectDetector(scene, sceneConfig, "");
    }

    // Load Models
    for (int i = 0; i < models.size(); i++) {
        cout << "loading: " << modelPath + models[i].first << " " << models[i].second << endl;
        obj->LoadModel(modelPath + models[i].first, models[i].second);
    }

    thread sceneViewer(&ObjectDetector::VisualizeResults, obj); // Visualization thread
    thread moveRobot; // Robot Communication thread

    while (1) {
        obj->LoadScene(scene);

        if (findCylinder) {
            obj->ProcessSceneCylinder();
            PointType p1 = obj->DetectCylinder();
            obj->SwitchView();
            sniffPoints.push_back(p1);
            if (useRobot) {
                moveRobot = thread(&PointsToRobot::SendPoints, pointsToRobot, sniffPoints);
            }
        }

        obj->ProcessScene();
        std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << obj->scene_keypoints->size() << std::endl;

        sniffPoints = obj->Detect();
        obj->SwitchView();
        if(useRobot && findCylinder) moveRobot.join();

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
        obj->SwitchView();
    }
    

    sceneViewer.join();

    return 0;
}

