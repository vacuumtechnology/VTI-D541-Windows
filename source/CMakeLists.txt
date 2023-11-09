cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(FeatureDetection)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Make project require C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake
                      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/3rd-party)

set(ZIVID_VERSION 2.9.0)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(PROGRAMS
    Camera/Basic/CaptureAssistant
    Applications/Basic/Visualization/CaptureVis3D
    Applications/Basic/Visualization/ReadPCLVis3D
    Applications/FeatureDetection
    Applications/DetectCylinder
    Applications/DetectSphere
    Applications/Calibrate
    Applications/Base
    Applications/CreateModel
    Applications/Crop
    Applications/FormTest
    Applications/RealSenseTest
    )

# Simple non robust way to find the librealsense library
if(WIN32)
    if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
        set(RS_LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x64")      
        set(RS_DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x64")          
    else()
        set(RS_LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x86")        
        set(RS_DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x86")            
    endif()
    set(PROJECT_BINARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/\$\(Configuration\)")
    set(RS_INCLUDE_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\include")   
endif()

find_library(REALSENSE2_FOUND realsense2 HINTS ${RS_LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()

find_package(
    Zivid ${ZIVID_VERSION}
    COMPONENTS Core
    REQUIRED)
find_package(Threads REQUIRED)

if(NOT DEFINED EIGEN3_INCLUDE_DIR)
    message(
        FATAL_ERROR
            "Please point EIGEN3_INCLUDE_DIR to the root directory of your Eigen3 installation (the folder containing Eigen/Core, Eigen/Dense etc.), or disable the Eigen samples (-DUSE_EIGEN3=OFF)."
    )
else()
    if(NOT IS_DIRECTORY "${EIGEN3_INCLUDE_DIR}/Eigen")
        message(
            FATAL_ERROR
                "EIGEN3_INCLUDE_DIR was set to ${EIGEN3_INCLUDE_DIR}, which is not a valid Eigen 3 directory. Please correct this, or disable the Eigen samples (-DUSE_EIGEN3=OFF)."
        )
    endif()
endif()

find_package(PCL 1.2)
if(NOT PCL_FOUND)
    message(
        FATAL_ERROR
            "Point Cloud Library (PCL) not found. Please point PCL_DIR to the directory of your PCL installation (containing the file PCLConfig.cmake), or disable the PCL samples with -DUSE_PCL=OFF."
    )
endif()

message(STATUS "All programs: ${PROGRAMS}")

if(WIN32)
    set(ZIVID_SAMPLE_DATA_DIR "$ENV{PROGRAMDATA}/Zivid")
elseif(UNIX)
    set(ZIVID_SAMPLE_DATA_DIR "/usr/share/Zivid/data")
else()
    message(FATAL_ERROR "Unsupported operating system")
endif()

foreach(PROGRAM ${PROGRAMS})
    get_filename_component(PROGRAM_NAME ${PROGRAM} NAME)

    FILE(GLOB SRCFILES ${PROGRAM}/*.c*)
    set(project_SOURCES ${SRCFILES})
    add_executable(${PROGRAM_NAME} ${project_SOURCES})

    target_link_libraries(${PROGRAM_NAME} Zivid::Core)

    target_link_libraries(${PROGRAM_NAME} Zivid::Visualization)

    target_include_directories(${PROGRAM_NAME} SYSTEM PRIVATE ${PROGRAM_NAME})

    target_link_libraries(${PROGRAM_NAME} ${PCL_LIBRARIES})
    target_include_directories(${PROGRAM_NAME} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})

    target_link_libraries(${PROGRAM_NAME} Threads::Threads)

    include_directories(${PROGRAM_NAME} ${RS_INCLUDE_DIR})
    target_link_libraries(${PROGRAM_NAME} ${REALSENSE2_FOUND})
    # target_link_libraries(rs-align ${realsense2_LIBRARY})

    # Post Build script to copy realsense2.dll
    # if(WIN32)
    # message(STATUS "Adding Post build script to copy realsense2.dll to project's binary folder")
    # message(STATUS "Will try to copy from ${RS_DLL_DIR} to ${PROJECT_BINARY_OUTPUT_PATH}")
    # add_custom_command(TARGET ${PROGRAM_NAME} POST_BUILD        # Adds a post-build event to rs-getting-started
    #     COMMAND ${CMAKE_COMMAND} -E copy_if_different              # which executes "cmake - E copy_if_different..."
    #         "${DLL_DIR}/realsense2.dll"                            # <--this is in-file
    #         ${PROJECT_BINARY_OUTPUT_PATH})                 # <--this is out-file path
    # endif()

endforeach()

