cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(PointCloudProcessing)

set(ZIVID_VERSION 2.9.0)

set(PROGRAMS
    Utilities/PCDCrop
    Utilities/ViewPCD
    Utilities/VoxelDownsample
    Utilities/CapturePCDZivid
    Utilities/VisualCrop
    Utilities/OutlierRemove
    ObjectDetection/RSDCorrespondence
    ObjectDetection/SHOTCorrespondence
    )

find_package(
    Zivid ${ZIVID_VERSION}
    COMPONENTS Core
    REQUIRED)
find_package(Threads REQUIRED)


find_package(PCL 1.2)
if(NOT PCL_FOUND)
    message(
        FATAL_ERROR
            "Point Cloud Library (PCL) not found. Please point PCL_DIR to the directory of your PCL installation (containing the file PCLConfig.cmake), or disable the PCL PROGRAMs with -DUSE_PCL=OFF."
    )
endif()

message(STATUS "All PROGRAMs: ${PROGRAMS}")

if(WIN32)
    set(ZIVID_PROGRAM_DATA_DIR "$ENV{PROGRAMDATA}/Zivid")
elseif(UNIX)
    set(ZIVID_PROGRAM_DATA_DIR "/usr/share/Zivid/data")
else()
    message(FATAL_ERROR "Unsupported operating system")
endif()
file(TO_CMAKE_PATH "${ZIVID_PROGRAM_DATA_DIR}" ZIVID_PROGRAM_DATA_DIR)

set(PROGRAM_NAMES_LIST_FILE "${CMAKE_BINARY_DIR}/PROGRAMNames.txt")
file(REMOVE ${PROGRAM_NAMES_LIST_FILE})
foreach(PROGRAM ${PROGRAMS})
    get_filename_component(PROGRAM_NAME ${PROGRAM} NAME)

    file(APPEND "${PROGRAM_NAMES_LIST_FILE}" ${PROGRAM}\n)

    add_executable(${PROGRAM_NAME} ${PROGRAM}/${PROGRAM_NAME}.cpp)

    target_link_libraries(${PROGRAM_NAME} Zivid::Core)

    target_compile_definitions(
        ${PROGRAM_NAME} PRIVATE ZIVID_PROGRAM_DATA_DIR="${ZIVID_PROGRAM_DATA_DIR}")

    target_link_libraries(${PROGRAM_NAME} Zivid::Visualization)

    target_link_libraries(${PROGRAM_NAME} ${PCL_LIBRARIES})
    target_include_directories(${PROGRAM_NAME} SYSTEM
                               PRIVATE ${PCL_INCLUDE_DIRS})
    
    target_link_libraries(${PROGRAM_NAME} Threads::Threads)

endforeach()
