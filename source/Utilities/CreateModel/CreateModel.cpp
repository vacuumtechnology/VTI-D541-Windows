#include "CreateModel.h"
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <mutex>
#include <pcl/point_types.h>

std::mutex mtx;

// Select Point on cloud using shift + Right Click
void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* pclObj)
{
    CreateModel* Viewer = (CreateModel*)pclObj;

    float x, y, z;
    if (event.getPointIndex() == -1)
    {
        return;
    }

    std::vector<int> indices(1);
    std::vector<float> dist(1);
    pcl::search::KdTree<pcl::PointXYZRGB> search;
    search.setInputCloud(Viewer->cloud_out);

    event.getPoint(x, y, z);

    PointType *pickPoint = new PointType();
    pickPoint->x = x;
    pickPoint->y = y;
    pickPoint->z = z;
    Viewer->pickPoints.push_back(pickPoint);

    search.nearestKSearch(*pickPoint, 1, indices, dist);
    cout << (int)Viewer->cloud_out->at(indices[0]).r << " " << (int)Viewer->cloud_out->at(indices[0]).g << " " << (int)Viewer->cloud_out->at(indices[0]).b << endl;
    
    std::stringstream cubenameStream;
    cubenameStream << "cube" << Viewer->cubeCounter;
    Viewer->cubeCounter++;
    std::string cubename = cubenameStream.str();

    Viewer->viewer->addCube(x - 2, x + 2, y - 2, y + 2, z - 2, z + 2, 1, .5, 0, cubename);
    Viewer->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cubename);
    
    std::cout << "Point ( " << x << ", " << y << ", " << z << ")" << std::endl;
}

// visualizer keystroke handler for cropping step
void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* obj) {
    CreateModel* mod = (CreateModel*)obj;
    if (event.keyDown()) {
        switch (event.getKeyCode()) {
        case '1':
            mod->dimensions[0] -= mod->increment;
            break;
        case '2':
            mod->dimensions[0] += mod->increment;
            break;
        case '3':
            mod->dimensions[1] -= mod->increment;
            break;
        case '4':
            mod->dimensions[1] += mod->increment;
            break;
        case '5':
            mod->dimensions[2] -= mod->increment;
            break;
        case '6':
            mod->dimensions[2] += mod->increment;
            break;
        case '7':
            mod->dimensions[3] -= mod->increment;
            break;
        case '8':
            mod->dimensions[3] += mod->increment;
            break;
        case '9':
            mod->dimensions[4] -= mod->increment;
            break;
        case '0':
            mod->dimensions[4] += mod->increment;
            break;
        case '-':
            mod->dimensions[5] -= mod->increment;
            break;
        case '=':
            mod->dimensions[5] += mod->increment;
            break;
        case 'p':
            mod->increment = 1;
            break;
        case 'o':
            mod->increment = 10;
            break;
        }
        mod->cloudUpdated = true;
    }
    
}

// constructor
CreateModel::CreateModel(std::string filename) {

    // Setup the cloud pointer
    cloud.reset(new PointCloudType);
    cloudx.reset(new PointCloudType);
    cloudy.reset(new PointCloudType);
    cloud_out.reset(new PointCloudType);

    // Fill the cloud with some points
    pcl::io::loadPCDFile<PointType>(filename, *cloud);
    cropBoxFilter.setInputCloud(cloud);
    passX.setInputCloud(cloud);
    passX.setFilterFieldName("x");
    passY.setInputCloud(cloudx);
    passY.setFilterFieldName("y");
    passZ.setInputCloud(cloudy);
    passZ.setFilterFieldName("z");

    dimensions.resize(6);
    dimensions = { -200, 200, -200, 200, 500, 900 }; // initial cropbox dimensions
    filterCloud();

    increment = 10;
}

// destructor
CreateModel::~CreateModel() {
    for (int i = 0; i < pickPoints.size(); i++) {
        delete(pickPoints[i]);
    }
}


void CreateModel::Configure() {
    std::thread sceneViewer(&CreateModel::Visualizer, this);

    std::cout << "Select model region in viewer. Enter 'd' here when done: ";
    std::string s;
    while (getline(cin, s)) {
        if (s == "d") {
            switchScene = true;
            break;
        }
    }

    std::cout << "Select pick points in viewer. Enter 'd' here when done: ";
    while (getline(cin, s)) {
        if (s == "d") {
            break;
        }
    }

    std::cout << "========Model Parameters========\n" << endl;
    cout << "Downsampling factor(0 to 10): ";
    cin >> model_ss;
    cout << "Descriptor Radius: ";
    cin >> descr_rad;
    cout << "Clustering group size: ";
    cin >> cg_size;
    cout << "Clustering threshold: ";
    cin >> cg_thresh;
    cout << "Reference frame radius: ";
    cin >> rf_rad;
    cout << "Expected number of objects: ";
    cin >> max_objects;
    cout << "Outlier removal threshold: ";
    cin >> out_thresh;
    cout << "Correspondence threshold: ";
    cin >> corr_thresh;
    cout << "Model Name: ";
    cin >> modelName;
    saveFile = path  + modelName;
    
    CreateDirectory(saveFile.c_str(), NULL);
    saveFile += "/" + modelName;
    std::cout << saveFile << endl;
    writeCloud();
    writeConfig();

    switchScene = true;
    sceneViewer.join();
}

void CreateModel::Visualizer() {
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->setSize(900, 1000);
    viewer->setBackgroundColor(.3, .3, .3);
    viewer->setCameraPosition(0, 0, -40, 0, -1, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> scene_color_handler(cloud, 120, 120, 120); // Set excluded points to grey
    viewer->addPointCloud(cloud, scene_color_handler, "cloud"); // excluded points

    viewer->addPointCloud(cloud_out, "cloud_out"); // included points
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, .5, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_out");
    viewer->addCube(dimensions[0], dimensions[1], dimensions[2], dimensions[3], dimensions[4], dimensions[5], 1, .5, 0);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
    viewer->addText("'1' <- xmin -> '2'\n'3' <- xmax -> '4'\n'5' <- ymin -> '6'\n'7' <- ymax -> '8'\n'9' <- zmin -> '0'\n'-' <- zmax -> '='\nPrecision mode -> 'p'\nLow precision -> 'o'", 0, 20, "text1");
    viewer->registerKeyboardCallback(keyboardCallback, (void *)this);
    viewer->resetCamera();
    switchScene = false;
    cloudUpdated = false;
    while (!viewer->wasStopped() && !switchScene) {
        while (!cloudUpdated && !switchScene) {
            viewer->spinOnce();
        }
        filterCloud();
        updateCloud();
        cloudUpdated = false;
    }

    viewer->removePointCloud("cloud");
    viewer->removeShape("cube");
    viewer->updateText("Shift + Click to select a point", 0, 20, "text1");
    viewer->registerPointPickingCallback(pointPickingEventOccurred, (void*)this);
    viewer->resetCamera();
    switchScene = false;
    while (!viewer->wasStopped() && !switchScene) {
        viewer->spinOnce();
    }
}

void CreateModel::filterCloud() {
    cout << "before: " << cloud_out->size() << endl;
    for (int i = 0; i < dimensions.size(); i++) {
        std::cout << dimensions[i] << endl;
    }
    passX.setFilterLimits(dimensions[0], dimensions[1]);
    passY.setFilterLimits(dimensions[2], dimensions[3]);
    passZ.setFilterLimits(dimensions[4], dimensions[5]);
    passX.filter(*cloudx);
    passY.filter(*cloudy);
    passZ.filter(*cloud_out);
    cout << "after: " << cloud_out->size() << endl;

}

void CreateModel::updateCloud() {
    viewer->updatePointCloud(cloud_out, "cloud_out");
    viewer->removeShape("cube");
    viewer->addCube(dimensions[0], dimensions[1], dimensions[2], dimensions[3], dimensions[4], dimensions[5], 1, .5, 0);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");

}

void CreateModel::writeCloud() {
    std::string filename = saveFile + ".pcd";
    pcl::io::savePCDFileBinary(filename, *cloud_out);
    std::cout << "file saved" << endl;
}

void CreateModel::writeConfig() {
    std::string filename = saveFile + ".config";

    std::ofstream myfile;
    myfile.open(filename);
    myfile << "model_ss = " << model_ss << endl;
    myfile << "rf_rad = " << rf_rad << endl;
    myfile << "descr_rad = " << descr_rad << endl;
    myfile << "cg_size = " << cg_size << endl;
    myfile << "cg_thresh = " << cg_thresh << endl;
    myfile << "max_objects = " << max_objects << endl;

    myfile << "pick_points" << endl;
    for (int i = 0; i < pickPoints.size(); i++) {
        myfile << "(" << pickPoints[i]->x << "," << pickPoints[i]->y << "," << pickPoints[i]->z << ")" << endl;
    }

    myfile.close();
}