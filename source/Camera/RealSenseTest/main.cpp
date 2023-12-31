
// Intel Realsense Headers
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>

using namespace std;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> point_cloud;
typedef point_cloud::Ptr cloud_pointer;
typedef point_cloud::Ptr prevCloud;

// Prototypes
void Visualize(cloud_pointer cloud);
bool userInput(void);

// Global Variables
string cloudFile; // .pcd file name
string prevCloudFile; // .pcd file name (Old cloud)
int i = 1; // Index for incremental file name

//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values. 
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels

    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index = (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//=================================================== 
cloud_pointer PCL_Conversion(const rs2::points& points, const rs2::video_frame& color) {

    // Object Declaration (Point Cloud)
    cloud_pointer cloud(new point_cloud);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = get<0>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = get<2>(RGB_Color); // Reference tuple<0>

    }

    return cloud; // PCL RGB Point Cloud generated
}

int main() try
{

    //======================
    // Variable Declaration
    //======================
    bool captureLoop = true; // Loop control for generating point clouds

    //====================
    // Object Declaration
    //====================
    pcl::PointCloud<PointType>::Ptr newCloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> openCloud;

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //======================
    // Stream configuration with parameters resolved internally. See enable_stream() overloads for extended usage
    //======================
    cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_INFRARED);
    cfg.enable_stream(RS2_STREAM_DEPTH);

    rs2::pipeline_profile selection = pipe.start(cfg);

    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        pipe.wait_for_frames();
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
    }

    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
        Sleep(1000);
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
    }

    // Begin Stream with default configs
    if (!userInput()) return 0;

    // Wait for frames from the camera to settle
    for (int i = 0; i < 30; i++) {
        auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
    }

    // Loop and take frame captures
    for (int i = 0; i < 10; i++) {

        // Capture a single frame and obtain depth + RGB values from it    
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto RGB = frames.get_color_frame();

        // Map Color texture to each point
        pc.map_to(RGB);

        // Generate Point Cloud
        auto points = pc.calculate(depth);

        // Convert generated Point Cloud to PCL Formatting
        cloud_pointer cloud = PCL_Conversion(points, RGB);

        // Resize/move point cloud to appear similar to zivid
        for (int i = 0; i < cloud->points.size(); i++) {
            cloud->points[i].x *= 900;
            cloud->points[i].y *= 900;
            cloud->points[i].z *= 900;
            cloud->points[i].z += 700;
        }

        pcl::PassThrough<PointType> Cloud_Filter; // Create the filtering object
        Cloud_Filter.setInputCloud(cloud);           // Input generated cloud to filter
        Cloud_Filter.setFilterFieldName("z");        // Set field name to Z-coordinate
        Cloud_Filter.setFilterLimits(500, 820);      // Set accepted interval values
        Cloud_Filter.filter(*cloud);              // Filtered Cloud Outputted

        *newCloud += *cloud;

    }//End-while

    std::cout << "Downsample" << std::endl;

    /*pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud(newCloud);
    uniform_sampling.setRadiusSearch(.6);
    uniform_sampling.filter(*newCloud);
    std::cout << "Cloud size after downsample: " << newCloud->size() << " points" << endl;*/



    pcl::RandomSample<PointType> random_sampling;
    random_sampling.setInputCloud(newCloud);
    random_sampling.setSample(150000);
    random_sampling.filter(*newCloud);
    std::cout << "Cloud size after downsample: " << newCloud->size() << " points" << endl;

    //pcl::StatisticalOutlierRemoval<PointType> sor;
    //sor.setInputCloud(newCloud);
    //sor.setMeanK(15);
    //sor.setStddevMulThresh(.7);
    //sor.filter(*newCloud);

    std::cout << "Cloud size: " << newCloud->size() << " points" << endl;

    cloudFile = "../../pcd/Captured_Frame.pcd";

    //==============================
    // Write PC to .pcd File Format
    //==============================
    // Take Cloud Data and write to .PCD File Format
    cout << "Generating PCD Point Cloud File... " << endl;
    pcl::io::savePCDFileASCII(cloudFile, *newCloud); // Input cloud to be saved to .pcd
    cout << cloudFile << " successfully generated. " << endl;

    rs2::colorizer color_map;
    for (auto&& frame : pipe.wait_for_frames())
    {
        // We can only save video frames as pngs, so we skip the rest
        if (auto vf = frame.as<rs2::video_frame>())
        {
            auto stream = frame.get_profile().stream_type();
            // Use the colorizer to get an rgb image for the depth stream
            if (vf.is<rs2::depth_frame>()) vf = color_map.process(frame);

            // Write images to disk
            std::string png_file = "../../png/rs_frame.png";
            stbi_write_png(png_file.c_str(), vf.get_width(), vf.get_height(), vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
            std::cout << "Saved " << png_file << std::endl;
        }
    }

    //Load generated PCD file for viewing
    Visualize(newCloud);


    cout << "Exiting Program... " << endl;
    return EXIT_SUCCESS;
} catch (const rs2::error& e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


void Visualize(cloud_pointer cloudView)
{
    

    //==========================
    // Pointcloud Visualization
    //==========================
    // Create viewer object titled "Captured Frame"
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Captured Frame"));

    // Set background of viewer to black
    viewer->setBackgroundColor(.3, .3, .3);
    // Add generated point cloud and identify with string "Cloud"
    viewer->addPointCloud<PointType>(cloudView, "Cloud");
    // Default size for rendered points
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
    viewer->setCameraPosition(0, 0, -100, 0, -1, 0);
    viewer->resetCamera();

    cout << endl;
    cout << "Press [Q] in viewer to continue. " << endl;

    viewer->spin(); // Allow user to rotate point cloud and view it

}

//========================================
// userInput
// - Prompts user for a char to 
// test for decision making.
// [y|Y] - Capture frame and save as .pcd
// [n|N] - Exit program
//========================================
bool userInput(void) {

    bool setLoopFlag;
    bool inputCheck = false;
    char takeFrame; // Utilize to trigger frame capture from key press ('t')
    do {

        // Prompt User to execute frame capture algorithm
        cout << endl;
        cout << "Generate a Point Cloud? [y/n] ";
        cin >> takeFrame;
        cout << endl;
        // Condition [Y] - Capture frame, store in PCL object and display
        if (takeFrame == 'y' || takeFrame == 'Y') {
            setLoopFlag = true;
            inputCheck = true;
            takeFrame = 0;
        }
        // Condition [N] - Exit Loop and close program
        else if (takeFrame == 'n' || takeFrame == 'N') {
            setLoopFlag = false;
            inputCheck = true;
            takeFrame = 0;
        }
        // Invalid Input, prompt user again.
        else {
            inputCheck = false;
            cout << "Invalid Input." << endl;
            takeFrame = 0;
        }
    } while (inputCheck == false);

    return setLoopFlag;
}