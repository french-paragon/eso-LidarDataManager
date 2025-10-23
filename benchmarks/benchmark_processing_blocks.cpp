/*
 * This file is part of the LidarDataManager tool.
 * Copyright (c) 2025 Laurent Valentin Jospin <laurent.jospin@epfl.ch>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <benchmark/benchmark.h>

#include <StereoVision/io/pointcloud_io.h>
#include <StereoVision/io/pcd_pointcloud_io.h>
#include <StereoVision/io/las_pointcloud_io.h>

#include "../processingBlocks/identityprocessor.h"
#include "../processingBlocks/attributebasedselector.h"
#include "../processingBlocks/crsconversion.h"

#include <random>

using GenericCloud = StereoVision::IO::GenericPointCloud<float, float>;
using GenericCloudHeaderInterface = StereoVision::IO::GenericPointCloudHeaderInterface<float, float>;
using GenericCloudInterface = StereoVision::IO::GenericPointCloudPointAccessInterface<float, float>;

GenericCloud getRandomPointCloud(int nPoints) {

    using Point = GenericCloud::Point;

    std::random_device rd;
    std::default_random_engine re;
    re.seed(rd());

    constexpr float ptRange = 1000;
    constexpr float colorRange = 1;

    std::uniform_real_distribution<float> points_dist(-ptRange, ptRange);
    std::uniform_real_distribution<float> color_dist(0, colorRange);

    GenericCloud ret;

    for (int i = 0; i < nPoints; i++) {
        Point point;

        point.xyz.x = points_dist(re);
        point.xyz.y = points_dist(re);
        point.xyz.z = points_dist(re);

        point.rgba.r = color_dist(re);
        point.rgba.g = color_dist(re);
        point.rgba.b = color_dist(re);
        point.rgba.a = color_dist(re);

        ret.addPoint(point);
    }

    return ret;

}

static void PcdAsciiWritingBenchmark(benchmark::State& state) {

    // setup
    constexpr int nPoints = 1024;

    GenericCloud ptCloud = getRandomPointCloud(nPoints);

    StereoVision::IO::FullPointCloudAccessInterface pointCloudStack;

    pointCloudStack.headerAccess = std::make_unique<GenericCloudHeaderInterface>(ptCloud);
    pointCloudStack.pointAccess = std::make_unique<GenericCloudInterface>(ptCloud);

    GenericCloudInterface* base = static_cast<GenericCloudInterface*>(pointCloudStack.pointAccess.get());

    std::string outFile = "test_pcd_ascii.pcd";

    //time loop
    for (auto _ : state) {

        bool hasMore = true;
        base->reset();

        StereoVision::IO::PcdDataStorageType dataStorageType = StereoVision::IO::PcdDataStorageType::ascii;

        bool ok = StereoVision::IO::writePointCloudPcd(std::filesystem::path(outFile), pointCloudStack, dataStorageType);

        benchmark::DoNotOptimize(ok);

    }
}

static void PcdBinaryWritingBenchmark(benchmark::State& state) {

    // setup
    constexpr int nPoints = 1024;

    GenericCloud ptCloud = getRandomPointCloud(nPoints);

    StereoVision::IO::FullPointCloudAccessInterface pointCloudStack;

    pointCloudStack.headerAccess = std::make_unique<GenericCloudHeaderInterface>(ptCloud);
    pointCloudStack.pointAccess = std::make_unique<GenericCloudInterface>(ptCloud);

    GenericCloudInterface* base = static_cast<GenericCloudInterface*>(pointCloudStack.pointAccess.get());

    std::string outFile = "test_pcd_bin.pcd";

    //time loop
    for (auto _ : state) {

        bool hasMore = true;
        base->reset();

        StereoVision::IO::PcdDataStorageType dataStorageType = StereoVision::IO::PcdDataStorageType::binary;

        bool ok = StereoVision::IO::writePointCloudPcd(std::filesystem::path(outFile), pointCloudStack, dataStorageType);

        benchmark::DoNotOptimize(ok);

    }
}

static void LasWritingBenchmark(benchmark::State& state) {

    // setup
    constexpr int nPoints = 1024;

    GenericCloud ptCloud = getRandomPointCloud(nPoints);

    StereoVision::IO::FullPointCloudAccessInterface pointCloudStack;

    pointCloudStack.headerAccess = std::make_unique<GenericCloudHeaderInterface>(ptCloud);
    pointCloudStack.pointAccess = std::make_unique<GenericCloudInterface>(ptCloud);

    GenericCloudInterface* base = static_cast<GenericCloudInterface*>(pointCloudStack.pointAccess.get());

    std::string outFile = "test_las.las";

    //time loop
    for (auto _ : state) {

        bool hasMore = true;
        base->reset();

        bool ok = StereoVision::IO::writePointCloudLas(std::filesystem::path(outFile), pointCloudStack);

        benchmark::DoNotOptimize(ok);

    }
}

static void PcdAsciiReadingBenchmark(benchmark::State& state) {
    // setup

    std::string inFile = "test_pcd_ascii.pcd";

    bool fileReadOk = true;

    std::array<float,3> meanPoint = {0,0,0};
    std::array<float,4> meanColor = {0,0,0,0};

    //time loop
    for (auto _ : state) {

        StatusOptional<StereoVision::IO::FullPointCloudAccessInterface> pointCloudStack =
                StereoVision::IO::openPointCloud(inFile);

        if (!pointCloudStack.has_value()) {
            state.SkipWithError("Failed to open file");
            break;
        }

        bool hasMore = true;

        do {

            auto point = pointCloudStack->pointAccess->castedPointGeometry<float>();

            meanPoint[0] += point.x;
            meanPoint[1] += point.y;
            meanPoint[2] += point.z;

            auto color = pointCloudStack->pointAccess->castedPointColor<float>();

            if (color.has_value()) {
                meanColor[0] = color->r;
                meanColor[1] = color->g;
                meanColor[2] = color->b;
                meanColor[3] = color->a;
            }

            hasMore = pointCloudStack->pointAccess->gotoNext();

        } while (hasMore);

    }

    benchmark::DoNotOptimize(meanPoint);
    benchmark::DoNotOptimize(meanColor);
}

static void PcdBinaryReadingBenchmark(benchmark::State& state) {
    // setup

    std::string inFile = "test_pcd_bin.pcd";

    bool fileReadOk = true;

    std::array<float,3> meanPoint = {0,0,0};
    std::array<float,4> meanColor = {0,0,0,0};

    //time loop
    for (auto _ : state) {

        StatusOptional<StereoVision::IO::FullPointCloudAccessInterface> pointCloudStack =
                StereoVision::IO::openPointCloud(inFile);

        if (!pointCloudStack.has_value()) {
            state.SkipWithError("Failed to open file");
            break;
        }

        bool hasMore = true;

        do {

            auto point = pointCloudStack->pointAccess->castedPointGeometry<float>();

            meanPoint[0] += point.x;
            meanPoint[1] += point.y;
            meanPoint[2] += point.z;

            auto color = pointCloudStack->pointAccess->castedPointColor<float>();

            if (color.has_value()) {
                meanColor[0] = color->r;
                meanColor[1] = color->g;
                meanColor[2] = color->b;
                meanColor[3] = color->a;
            }

            hasMore = pointCloudStack->pointAccess->gotoNext();

        } while (hasMore);

    }

    benchmark::DoNotOptimize(meanPoint);
    benchmark::DoNotOptimize(meanColor);
}

static void LasReadingBenchmark(benchmark::State& state) {
    // setup

    std::string inFile = "test_las.las";

    bool fileReadOk = true;

    std::array<float,3> meanPoint = {0,0,0};
    std::array<float,4> meanColor = {0,0,0,0};

    //time loop
    for (auto _ : state) {

        StatusOptional<StereoVision::IO::FullPointCloudAccessInterface> pointCloudStack =
                StereoVision::IO::openPointCloud(inFile);

        if (!pointCloudStack.has_value()) {
            state.SkipWithError("Failed to open file");
            break;
        }

        bool hasMore = true;

        do {

            auto point = pointCloudStack->pointAccess->castedPointGeometry<float>();

            meanPoint[0] += point.x;
            meanPoint[1] += point.y;
            meanPoint[2] += point.z;

            auto color = pointCloudStack->pointAccess->castedPointColor<float>();

            if (color.has_value()) {
                meanColor[0] = color->r;
                meanColor[1] = color->g;
                meanColor[2] = color->b;
                meanColor[3] = color->a;
            }

            hasMore = pointCloudStack->pointAccess->gotoNext();

        } while (hasMore);

    }

    benchmark::DoNotOptimize(meanPoint);
    benchmark::DoNotOptimize(meanColor);
}

static void PcdAsciiFullReadWriteBenchmark(benchmark::State& state) {


    std::string inFile = "test_pcd_ascii.pcd";

    bool fileReadOk = true;

    std::array<float,3> meanPoint = {0,0,0};
    std::array<float,4> meanColor = {0,0,0,0};

    std::string outFile = "test_pcd_ascii_copy.pcd";

    //time loop
    for (auto _ : state) {

        StatusOptional<StereoVision::IO::FullPointCloudAccessInterface> pointCloudStack =
                StereoVision::IO::openPointCloud(inFile);

        if (!pointCloudStack.has_value()) {
            state.SkipWithError("Failed to open file");
            break;
        }

        //ensure to disable the optimization where the data is just copied directly when writing to the same type of file.
        pointCloudStack.value().pointAccess = std::make_unique<IdentityProcessor>(std::move(pointCloudStack.value().pointAccess));

        StereoVision::IO::PcdDataStorageType dataStorageType = StereoVision::IO::PcdDataStorageType::ascii;

        bool ok = StereoVision::IO::writePointCloudPcd(std::filesystem::path(outFile), pointCloudStack.value(), dataStorageType);

        benchmark::DoNotOptimize(ok);

    }
}

static void PcdBinaryFullReadWriteBenchmark(benchmark::State& state) {


    std::string inFile = "test_pcd_bin.pcd";

    bool fileReadOk = true;

    std::array<float,3> meanPoint = {0,0,0};
    std::array<float,4> meanColor = {0,0,0,0};

    std::string outFile = "test_pcd_bin_copy.pcd";

    //time loop
    for (auto _ : state) {

        StatusOptional<StereoVision::IO::FullPointCloudAccessInterface> pointCloudStack =
                StereoVision::IO::openPointCloud(inFile);

        if (!pointCloudStack.has_value()) {
            state.SkipWithError("Failed to open file");
            break;
        }

        //ensure to disable the optimization where the data is just copied directly when writing to the same type of file.
        pointCloudStack.value().pointAccess = std::make_unique<IdentityProcessor>(std::move(pointCloudStack.value().pointAccess));

        StereoVision::IO::PcdDataStorageType dataStorageType = StereoVision::IO::PcdDataStorageType::binary;

        bool ok = StereoVision::IO::writePointCloudPcd(std::filesystem::path(outFile), pointCloudStack.value(), dataStorageType);

        benchmark::DoNotOptimize(ok);

    }
}

static void LasFullReadWriteBenchmark(benchmark::State& state) {


    std::string inFile = "test_las.las";

    bool fileReadOk = true;

    std::array<float,3> meanPoint = {0,0,0};
    std::array<float,4> meanColor = {0,0,0,0};

    std::string outFile = "test_las_copy.las";

    //time loop
    for (auto _ : state) {

        StatusOptional<StereoVision::IO::FullPointCloudAccessInterface> pointCloudStack =
                StereoVision::IO::openPointCloud(inFile);

        if (!pointCloudStack.has_value()) {
            state.SkipWithError("Failed to open file");
            break;
        }

        //ensure to disable the optimization where the data is just copied directly when writing to the same type of file.
        pointCloudStack.value().pointAccess = std::make_unique<IdentityProcessor>(std::move(pointCloudStack.value().pointAccess));

        StereoVision::IO::PcdDataStorageType dataStorageType = StereoVision::IO::PcdDataStorageType::binary;

        bool ok = StereoVision::IO::writePointCloudPcd(std::filesystem::path(outFile), pointCloudStack.value(), dataStorageType);

        benchmark::DoNotOptimize(ok);

    }
}

static void AttributeSelectorBenchmark(benchmark::State& state) {
    // setup
    constexpr int nPoints = 1024;

    GenericCloud ptCloud = getRandomPointCloud(nPoints);

    std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> baseInterface =
            std::make_unique<GenericCloudInterface>(ptCloud);

    GenericCloudInterface* base = static_cast<GenericCloudInterface*>(baseInterface.get());

    std::array<int,2> options = {42, 69};

    std::string name = "number";
    AttributeBasedSelector::Comparator comparator = AttributeBasedSelector::Equal;
    StereoVision::IO::PointCloudGenericAttribute val = options[0];

    ptCloud.addAttribute(name);

    for (int i = 0; i < nPoints; i++) {
        ptCloud[i].attributes[name] = options[i%2];
    }

    std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> selector =
            AttributeBasedSelector::setupAttributeBasedSelector(baseInterface,
                                                                name,
                                                                comparator,
                                                                val);

    int nRead = 0;

    //time loop
    for (auto _ : state) {

        bool hasMore = true;
        base->reset();

        do {

            auto point = selector->castedPointGeometry<float>();
            auto color = selector->castedPointColor<float>();

            benchmark::DoNotOptimize(point);
            benchmark::DoNotOptimize(color);

            nRead++;

            hasMore = selector->gotoNext();

        } while (hasMore);
    }

    int rFinal = nRead;

    benchmark::DoNotOptimize(rFinal);
}

static void ConversionEcef2Geo(benchmark::State& state) {
    // setup
    constexpr int nPoints = 1024;

    constexpr double earthRadius = 6.3781e6;

    GenericCloud ptCloud = getRandomPointCloud(nPoints);

    for (int i = 0; i < nPoints; i++) {

        double norm = ptCloud[i].xyz.x*ptCloud[i].xyz.x;
        norm += ptCloud[i].xyz.y*ptCloud[i].xyz.y;
        norm += ptCloud[i].xyz.z*ptCloud[i].xyz.z;

        norm = std::sqrt(norm);

        double scale = earthRadius/norm;

        ptCloud[i].xyz.x *= scale;
        ptCloud[i].xyz.y *= scale;
        ptCloud[i].xyz.z *= scale;
    }

    std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> baseInterface =
            std::make_unique<GenericCloudInterface>(ptCloud);

    GenericCloudInterface* base = static_cast<GenericCloudInterface*>(baseInterface.get());

    std::string inCrs = "EPSG:4978";
    std::string outCrs = "EPSG:4979";

    std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> selector =
            CrsConversion::setupCrsConversion(baseInterface,
                                              inCrs,
                                              outCrs);

    int nRead = 0;

    //time loop
    for (auto _ : state) {

        bool hasMore = true;

        do {

            auto point = selector->castedPointGeometry<float>();
            auto color = selector->castedPointColor<float>();

            benchmark::DoNotOptimize(point);
            benchmark::DoNotOptimize(color);

            nRead++;

            hasMore = selector->gotoNext();

        } while (hasMore);

        base->reset(); // reset the generic cloud first, to have more data ready.
        selector->gotoNext(); //then reset the buffered point cloud.
    }

    int rFinal = nRead;

    benchmark::DoNotOptimize(rFinal);
}

BENCHMARK(PcdAsciiWritingBenchmark);
BENCHMARK(PcdBinaryWritingBenchmark);
BENCHMARK(LasWritingBenchmark);
BENCHMARK(PcdAsciiReadingBenchmark);
BENCHMARK(PcdBinaryReadingBenchmark);
BENCHMARK(LasReadingBenchmark);
BENCHMARK(PcdAsciiFullReadWriteBenchmark);
BENCHMARK(PcdBinaryFullReadWriteBenchmark);
BENCHMARK(LasFullReadWriteBenchmark);
BENCHMARK(AttributeSelectorBenchmark);
BENCHMARK(ConversionEcef2Geo);

BENCHMARK_MAIN();
