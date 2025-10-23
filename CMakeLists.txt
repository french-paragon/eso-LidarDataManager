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

include(FetchContent)

set(CMAKE_INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR})
set(CMAKE_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR})
set(CMAKE_INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME})
set(CMAKE_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

#dependencies

find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

FetchContent_Declare(
  MultidimArrays
  GIT_REPOSITORY https://github.com/french-paragon/MultidimArrays.git
  GIT_TAG 6b816b3b0ab8f1a484cfa695daa9f64f11ea2348
  FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(MultidimArrays)

FetchContent_Declare(
  StereoVision
  GIT_REPOSITORY https://github.com/french-paragon/LibStevi.git
  GIT_TAG 8175f3a85d28b12033ca0717891e58cf310f6539
  FIND_PACKAGE_ARGS
)
option(buildGui OFF) #ensuire gui is not built for the fetched stereo vision library
FetchContent_MakeAvailable(StereoVision)

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
