cmake_minimum_required(VERSION 2.8)

project(tjsp_attack_2020)

set(CMAKE_CXX_STANDARD 14)  # for chrono
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Ofast -march=native -pthread")

set(CAMERA "DAHUA") # default
set(CAMERA "MINDVISION")
set(CAMERA "DAHENG")

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
    message("-- Found Boost NOT FOUND")
endif ()

#========================== Serial ===============================
#set(serial_DIR /usr/local/share/serial/cmake/)
find_package(serial)
include_directories(${serial_INCLUDE_DIRS})

#======================== Dahua SDK ==============================
include_directories(/opt/DahuaTech/MVviewer/include)

#======================== MindVision SDK ==============================
include_directories(/opt/MindVision/include)

#========================= Include ===============================
include_directories(source include)

# set(LIB_THREAD_DIR /lib/x86_64-linux-gnu)
# add_library(libpthread SHARED IMPORTED)
# set_target_properties(libopencv_imgproc PROPERTIES IMPORTED_LOCATION ${LIB_THREAD_DIR}/libpthread.so.0)


add_executable(attack main.cpp)

target_link_libraries(attack ${OpenCV_LIBS})
target_link_libraries(attack ${serial_LIBRARIES})
target_link_libraries(attack ${Boost_LIBRARIES} usb-1.0)
target_link_libraries(attack ${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/libtensorflow_cc.so ${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/libtensorflow_framework.so.2)

## DaHua SDK 和 MindVision SDK 库文件重名, 只能取一个
if (${CAMERA} EQUAL "MINDVISION")
    message("Linking MindVision SDK...")
    target_link_libraries(attack /opt/MindVision/lib/x64/libMVSDK.so)
    add_definitions(-DMINDVISION)

# elseif (${CAMERA} EQUAL "DAHENG")
    message("Linking DaHeng SDK...")
#    target_link_libraries(attack /opt/MindVision/lib/x64/libMVSDK.so)
    add_definitions(-DDAHENG)

else ()
    message("Linking DaHua SDK...")
    target_link_libraries(attack /opt/DahuaTech/MVviewer/lib/libMVSDK.so) 

endif ()

#target_link_libraries(attack libpthread)
#file(GLOB HikVision "/opt/MVS/Samples_LinuxSDK/lib/64/*.so")
#target_link_libraries(attack ${HikVision})

# FIND_PACKAGE(OpenMP REQUIRED)
# if (OPENMP_FOUND)
#     message("OPENMP FOUND")
#     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#     set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# endif ()

