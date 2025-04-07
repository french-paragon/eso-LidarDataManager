#include <benchmark/benchmark.h>

#include <StereoVision/io/pointcloud_io.h>
#include <StereoVision/io/pcd_pointcloud_io.h>

#include "../processingBlocks/attributebasedselector.h"

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

static void PcdAsciiReadingBenchmark(benchmark::State& state) {
    // setup

    std::string inFile = "test_pcd_ascii.pcd";

    bool fileReadOk = true;

    std::array<float,3> meanPoint = {0,0,0};
    std::array<float,4> meanColor = {0,0,0,0};

    //time loop
    for (auto _ : state) {

        std::optional<StereoVision::IO::FullPointCloudAccessInterface> pointCloudStack =
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

        std::optional<StereoVision::IO::FullPointCloudAccessInterface> pointCloudStack =
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

BENCHMARK(PcdAsciiWritingBenchmark);
BENCHMARK(PcdBinaryWritingBenchmark);
BENCHMARK(PcdAsciiReadingBenchmark);
BENCHMARK(PcdBinaryReadingBenchmark);
BENCHMARK(AttributeSelectorBenchmark);

BENCHMARK_MAIN();
