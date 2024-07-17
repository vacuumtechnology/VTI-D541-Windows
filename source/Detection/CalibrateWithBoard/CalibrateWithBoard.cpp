/*
Perform Hand-Eye calibration.
*/

#include <Zivid/Application.h>
#include <Zivid/Calibration/Detector.h>
#include <Zivid/Calibration/HandEye.h>
#include <Zivid/Calibration/Pose.h>
#include <Zivid/Exception.h>
#include <Zivid/Zivid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <math.h>
#include <fstream>

#define PI 3.14159265

// Select Point on cloud using shift + Left Click
void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event)
{

    float x, y, z;
    if (event.getPointIndex() == -1)
    {
        return;
    }

    event.getPoint(x, y, z);

    std::cout << "Point ( " << x << ", " << y << ", " << z << ")" << std::endl;
}

namespace
{
    enum class CommandType
    {
        AddPose,
        Calibrate,
        Unknown
    };

    Zivid::Matrix4x4 eigenToZivid(const Eigen::Affine3f& eigenTransform)
    {
        Eigen::Matrix4f eigenMatrix = eigenTransform.matrix();
        Zivid::Matrix4x4 zividMatrix;
        for (Eigen::Index row = 0; row < eigenMatrix.rows(); row++)
        {
            for (Eigen::Index column = 0; column < eigenMatrix.cols(); column++)
            {
                zividMatrix(row, column) = eigenMatrix(row, column);
            }
        }
        return zividMatrix;
    }

    Zivid::Matrix4x4 poseToTransform(std::vector<double> pose) {

        //Eigen::AngleAxisd rollangle(pose[3] * PI / 180, Eigen::Vector3d::UnitZ());
        //Eigen::AngleAxisd yawangle(pose[4] * PI / 180, Eigen::Vector3d::UnitY());
        //Eigen::AngleAxisd pitchangle(pose[5] * PI / 180, Eigen::Vector3d::UnitX());
        //Eigen::Quaternion<double> q = rollangle * yawangle * pitchangle;
        //Eigen::Matrix3d r = q.matrix();
        //
        Eigen::Matrix4f m;
        cout << "roll(z): " << pose[3] << " pitch(y): " << pose[4] << " yaw(x): " << pose[5] << endl;
        Eigen::Matrix3f r = (Eigen::AngleAxisf(pose[5], Eigen::Vector3f::UnitX())
                            * Eigen::AngleAxisf(pose[4], Eigen::Vector3f::UnitY())
                            * Eigen::AngleAxisf(pose[3], Eigen::Vector3f::UnitZ()))
                            .matrix();

        Eigen::Vector3f translationVector;
        translationVector(0) = pose[0];
        translationVector(1) = pose[1];
        translationVector(2) = pose[2];

        Eigen::Affine3f transformationMatrixFromQuaternion(r);
        transformationMatrixFromQuaternion.translation() = translationVector;
        ////Rotation matrix from UVW
        /*m << r(0, 0), r(0, 1), r(0, 2), pose[0],
             r(1, 0), r(1, 1), r(1, 2), pose[1],
             r(2, 0), r(2, 1), r(2, 2), pose[2],
             0,       0,       0,       1;*/

        return eigenToZivid(transformationMatrixFromQuaternion);

    }

    std::string getInput()
    {
        std::string command;
        std::getline(std::cin, command);
        return command;
    }

    CommandType enterCommand()
    {
        std::cout << "Enter command, p (to add robot pose) or c (to perform calibration):";
        const auto command = getInput();

        if(command == "P" || command == "p")
        {
            return CommandType::AddPose;
        }
        if(command == "C" || command == "c")
        {
            return CommandType::Calibrate;
        }
        return CommandType::Unknown;
    }

    Zivid::Calibration::Pose enterRobotPose(size_t index)
    {
        std::vector<double> pose; // xyzuvw
        std::vector<std::string> prompt = { "x: ", "y: ", "z: ", "u: ", "v: ", "w: " };
        pose.resize(6);

        for (int i = 0; i < pose.size(); i++) {
            std::cout << prompt[i];
            std::cin >> pose[i];
        }

        const auto robotPose = poseToTransform(pose);

        std::cout << "The following pose was entered: \n" << robotPose << std::endl;

        return robotPose;
    }

    Zivid::Frame assistedCapture(Zivid::Camera &camera)
    {
        const auto suggestSettingsParameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
            Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
            Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 800 } }
        };
        const auto settings = Zivid::CaptureAssistant::suggestSettings(camera, suggestSettingsParameters);
        return camera.capture(settings);
    }

    Zivid::Calibration::HandEyeOutput performCalibration(
        const std::vector<Zivid::Calibration::HandEyeInput> &handEyeInput)
    {
        while(true)
        {
            std::cout << "Enter type of calibration, eth (for eye-to-hand) or eih (for eye-in-hand): ";
            const auto calibrationType = getInput();
            if(calibrationType == "eth" || calibrationType == "ETH")
            {
                std::cout << "Performing eye-to-hand calibration" << std::endl;
                return Zivid::Calibration::calibrateEyeToHand(handEyeInput);
            }
            if(calibrationType == "eih" || calibrationType == "EIH")
            {
                std::cout << "Performing eye-in-hand calibration" << std::endl;
                return Zivid::Calibration::calibrateEyeInHand(handEyeInput);
            }
            std::cout << "Entered uknown method" << std::endl;
        }
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera{ zivid.connectCamera() };

        size_t currentPoseId{ 0 };
        bool calibrate{ false };
        std::vector<Zivid::Calibration::HandEyeInput> handEyeInput;
        Zivid::Settings s = Zivid::Settings("C:\\Users\\VTI\\Downloads\\calibration_board_detection_settings.yml");
        do
        {
            switch(enterCommand())
            {
                case CommandType::AddPose:
                {
                    try
                    {
                        const auto robotPose = enterRobotPose(currentPoseId);

                        const auto frame = camera.capture(s);

                        std::cout << "Detecting checkerboard in point cloud" << std::endl;
                        const auto detectionResult = Zivid::Calibration::detectFeaturePoints(frame.pointCloud());

                        if(detectionResult.valid())
                        {
                            std::cout << "Calibration board detected " << std::endl;
                            handEyeInput.emplace_back(Zivid::Calibration::HandEyeInput{ robotPose, detectionResult });
                            currentPoseId++;
                        }
                        else
                        {
                            std::cout
                                << "Failed to detect calibration board, ensure that the entire board is in the view of the camera"
                                << std::endl;
                        }
                    }
                    catch(const std::exception &e)
                    {
                        std::cout << "Error: " << Zivid::toString(e) << std::endl;
                        continue;
                    }
                    break;
                }
                case CommandType::Calibrate:
                {
                    calibrate = true;
                    break;
                }
                case CommandType::Unknown:
                {
                    std::cout << "Error: Unknown command" << std::endl;
                    break;
                }
            }
        } while(!calibrate);

        const auto calibrationResult{ performCalibration(handEyeInput) };

        calibrationResult.transform().save("../../txt/zividtransform.yaml");

        std::ofstream calStream;
        calStream.open("../../txt/transform.cal");
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                calStream << calibrationResult.transform().at(i, j) << " ";
            }
            calStream << std::endl;
        }
        calStream.close();

        if(calibrationResult.valid()){
            std::cout << "Hand-Eye calibration OK\n"
                      << "Result:\n"
                      << calibrationResult << std::endl;
        } else {
            std::cout << "Hand-Eye calibration FAILED" << std::endl;
            return EXIT_FAILURE;
        }


        std::string dataFile = "../../pcd/Capture.zdf";
        std::cout << "Reading ZDF frame from file: " << dataFile << std::endl;
        const auto frame = Zivid::Frame(dataFile);
        auto pointCloud = frame.pointCloud();
        pointCloud.transform(calibrationResult.transform());

        const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA>();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

        cloud->width = data.width();
        cloud->height = data.height();
        cloud->is_dense = false;
        cloud->points.resize(data.size());
        for (size_t i = 0; i < data.size(); ++i)
        {
            cloud->points[i].x = data(i).point.x;
            cloud->points[i].y = data(i).point.y;
            cloud->points[i].z = data(i).point.z;
            cloud->points[i].r = data(i).color.r;
            cloud->points[i].g = data(i).color.g;
            cloud->points[i].b = data(i).color.b;
        }

        auto viewer = pcl::visualization::PCLVisualizer("Viewer");
        viewer.setSize(900, 1000);
        viewer.setBackgroundColor(.3, .3, .3);
        viewer.setCameraPosition(0, 0, -40, 0, -1, 0);
        viewer.addPointCloud<pcl::PointXYZRGB>(cloud);
        viewer.registerPointPickingCallback(pointPickingEventOccurred);
        viewer.resetCamera();

        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "\nError: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}