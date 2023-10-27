cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(FeatureDetection)

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
    )

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

    target_include_directories(${PROGRAM_NAME} SYSTEM
                                PRIVATE ${PROGRAM_NAME})

    target_link_libraries(${PROGRAM_NAME} ${PCL_LIBRARIES})
    target_include_directories(${PROGRAM_NAME} SYSTEM
                                PRIVATE ${PCL_INCLUDE_DIRS})

    target_link_libraries(${SAMPLE_NAME} Threads::Threads)


endforeach()

