cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0095 NEW)

project(LidarDataManager LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -march=native -ftree-vectorize")

enable_testing()

#global variables for installation

include(GNUInstallDirs)

set(CMAKE_INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR})
set(CMAKE_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR})
set(CMAKE_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME})
set(CMAKE_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

#dependencies

find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(StereoVision REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED IMPORTED_TARGET proj)

add_library(PROJ::proj ALIAS PkgConfig::PROJ)

#configure executable
set(PROC_BLOCKS_FILES
    processingBlocks/aliasheaderattributes.h
    processingBlocks/aliasheaderattributes.cpp
    processingBlocks/identityprocessor.h
    processingBlocks/identityprocessor.cpp
    processingBlocks/crsconversion.h
    processingBlocks/crsconversion.cpp
    processingBlocks/pointsattributesfilters.h
    processingBlocks/pointsattributesfilters.cpp
    processingBlocks/regionofinterestselector.h
    processingBlocks/regionofinterestselector.cpp
    processingBlocks/attributebasedselector.h
    processingBlocks/attributebasedselector.cpp
    processingBlocks/attributesetbasedselector.h
    processingBlocks/attributesetbasedselector.cpp
    processingBlocks/pointsnumberlimit.h
    processingBlocks/pointsnumberlimit.cpp)

set(DATA_MANAGER_SRC lidarDataManager.cpp
    ${PROC_BLOCKS_FILES})

add_executable(lidarDataManager
    ${DATA_MANAGER_SRC}
)

set(DENSITY_CACHE_SRC densityCacheEstimator.cpp)

add_executable(densityCacheEstimator
    ${DENSITY_CACHE_SRC}
)

target_link_libraries(lidarDataManager StereoVision::stevi PROJ::proj)

target_link_libraries(densityCacheEstimator StereoVision::stevi)


add_subdirectory(tests)
add_subdirectory(benchmarks)
