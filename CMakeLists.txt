cmake_minimum_required(VERSION 2.8)

project(tjsp_attack_2021)

set(CMAKE_CXX_STANDARD 14)  # for chrono
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Ofast -march=native -pthread")

set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-std=c++14)

#====================== OpenCV Config ============================
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

#========================= Threads ===============================
find_package(Threads REQUIRED)

#======================= Tensorflow ==============================
include_directories(./tensorflow/include)
include_directories(./tensorflow/include/src)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/tensorflow)

#=========================== Boost ===============================
find_package(Boost REQUIRED COMPONENTS thread)
if (!Boost_FOUND)
    message("-- Boost NOT FOUND, Install with 'sudo apt install libboost-dev'")
endif ()

#========================== Serial ===============================
#set(serial_DIR /usr/local/share/serial/cmake/)
find_package(serial)
include_directories(${serial_INCLUDE_DIRS})

#======================== Dahua SDK ==============================
include_directories(/opt/DahuaTech/MVviewer/include)

#======================== MindVision SDK ==============================
include_directories(/opt/mindvision/include)

#========================= Include ===============================
include_directories(source include)


add_executable(attack_dahua main.cpp)
target_link_libraries(attack_dahua ${OpenCV_LIBS} ${serial_LIBRARIES} ${Boost_LIBRARIES} usb-1.0 dlib)
target_link_libraries(attack_dahua ${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/libtensorflow_cc.so ${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/libtensorflow_framework.so.2)
target_link_libraries(attack_dahua /opt/DahuaTech/MVviewer/lib/libMVSDK.so) 
target_compile_definitions(attack_dahua PRIVATE DAHUA)


add_executable(attack_mind main.cpp)
target_link_libraries(attack_mind ${OpenCV_LIBS} ${serial_LIBRARIES} ${Boost_LIBRARIES} usb-1.0 dlib)
target_link_libraries(attack_mind ${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/libtensorflow_cc.so ${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/libtensorflow_framework.so.2)
target_link_libraries(attack_mind /opt/mindvision/lib/x64/libMVSDK.so)
target_compile_definitions(attack_mind PRIVATE MINDVISION)

