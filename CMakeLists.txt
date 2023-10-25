cmake_minimum_required(VERSION 3.5)

project(MarkerDetectGui LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV CONFIG REQUIRED)
find_package(aruco CONFIG REQUIRED)
find_package(Qt5 COMPONENTS
    Widgets Core Gui
    Xml SerialPort
    REQUIRED)

add_executable(MarkerDetectGui
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    datapack.h
    camthread.cpp
    camthread.h
    serialthread.cpp
    serialthread.h
    calibrator.cpp
    calibrator.h
)

target_link_libraries(MarkerDetectGui PRIVATE
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
    Qt5::Xml
    Qt5::SerialPort
)
target_link_libraries(MarkerDetectGui PRIVATE
    opencv_calib3d
    opencv_core
    opencv_features2d
    #opencv_highgui
    opencv_imgcodecs
    opencv_imgproc
    opencv_objdetect
    opencv_stitching
    opencv_video
    opencv_videoio
    opencv_aruco
    opencv_videostab
    opencv_xfeatures2d
    opencv_ximgproc
    aruco
    pthread
)
