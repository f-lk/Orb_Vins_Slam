cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "-std=c++0x" )
project( orb_vins_slam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(/usr/include/eigen3/Eigen)

find_package(OpenCV 3 REQUIRED)

find_package(Pangolin REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
        ${Pangolin_INCLUDE_DIRS}
)

include_directories(/usr/include/eigen3)

set( DBoW3_INCLUDE_DIRS "/usr/local/include" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.a" )

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(orb_vins_lib SHARED
        ${PROJECT_SOURCE_DIR}/src/system.cpp
        ${PROJECT_SOURCE_DIR}/src/tracking.cpp
        ${PROJECT_SOURCE_DIR}/src/ORBextractor.cpp
        ${PROJECT_SOURCE_DIR}/src/keyframe.cpp
        ${PROJECT_SOURCE_DIR}/src/converter.cpp
        ${PROJECT_SOURCE_DIR}/src/initializer.cpp
        ${PROJECT_SOURCE_DIR}/src/ORBmatcher.cpp
        ${PROJECT_SOURCE_DIR}/src/mappoint.cpp
        ${PROJECT_SOURCE_DIR}/src/map.cpp
        ${PROJECT_SOURCE_DIR}/src/viewer.cpp
        ${PROJECT_SOURCE_DIR}/src/mapdrawer.cpp
        ${PROJECT_SOURCE_DIR}/src/framedrawer.cpp
        ${PROJECT_SOURCE_DIR}/src/frame.cpp
        ${PROJECT_SOURCE_DIR}/src/keyframedatabase.cpp
        ${PROJECT_SOURCE_DIR}/src/optimizer.cpp
        ${PROJECT_SOURCE_DIR}/src/pnpsolver.cpp
        )

add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/run/run_EuRoc.cpp)

target_link_libraries(${PROJECT_NAME}
        ${EIGEN3_lib}
        ${OpenCV_LIBRARIES}
        orb_vins_lib
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        )
