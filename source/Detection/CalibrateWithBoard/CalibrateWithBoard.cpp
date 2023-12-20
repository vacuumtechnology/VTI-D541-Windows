/*
Perform Hand-Eye calibration.
*/

#include <Zivid/Application.h>
#include <Zivid/Calibration/Detector.h>
#include <Zivid/Calibration/HandEye.h>
#include <Zivid/Calibration/Pose.h>
#include <Zivid/Exception.h>
#include <Zivid/Zivid.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <math.h>
#include <fstream>

#define PI 3.14159265

namespace
{
    enum class CommandType
    {
        AddPose,
        Calibrate,
        Unknown
    };

    Zivid::Matrix4x4 eigenToZivid(const Eigen::Matrix4f& eigenTransform)
    {
        Eigen::Matrix4f eigenMatrix = eigenTransform;
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
        Eigen::Matrix3f r = (Eigen::AngleAxisf(pose[5], Eigen::Vector3f::UnitX())
                            * Eigen::AngleAxisf(pose[4], Eigen::Vector3f::UnitY())
                            * Eigen::AngleAxisf(pose[3], Eigen::Vector3f::UnitZ()))
                            .matrix();;

        ////Rotation matrix from UVW


        m << r(0, 0), r(0, 1), r(0, 2), pose[0],
             r(1, 0), r(1, 1), r(1, 2), pose[1],
             r(2, 0), r(2, 1), r(2, 2), pose[2],
             0,       0,       0,       1;

        return eigenToZivid(m);


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
        do
        {
            switch(enterCommand())
            {
                case CommandType::AddPose:
                {
                    try
                    {
                        const auto robotPose = enterRobotPose(currentPoseId);

                        const auto frame = assistedCapture(camera);

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