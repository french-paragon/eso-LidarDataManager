#include <iostream>
#include <string>
#include <filesystem>

#include <tclap/CmdLine.h>

#include <StereoVision/io/pointcloud_io.h>

#include <limits>
#include <cmath>

#include <MultidimArrays/MultidimArrays.h>

constexpr int gridScale = 10;

struct GridInfos {
    double scale;
    double x0;
    double y0;
    int width;
    int height;
};

std::optional<GridInfos> computeGridInfos(std::string const& inFile) {

    //Open file
    auto pointCloudStackOpt = StereoVision::IO::openPointCloud(inFile);

    if (!pointCloudStackOpt.has_value()) {
        std::cerr << "Could not open file: " << inFile << "! Aborting!" << std::endl;
        return std::nullopt;
    }

    StereoVision::IO::FullPointCloudAccessInterface& pointCloudStack = pointCloudStackOpt.value();

    if (pointCloudStack.headerAccess == nullptr and pointCloudStack.pointAccess == nullptr) {
        std::cerr << "Error reading file: " << inFile << ", null accesss interfaces! Aborting!" << std::endl;
        return std::nullopt;
    }

    bool ok = true;

    auto currentPoints = pointCloudStack.pointAccess->castedPointGeometry<double>();

    double minX = currentPoints.x;
    double maxX = currentPoints.x;

    double minY = currentPoints.y;
    double maxY = currentPoints.y;

    int nPoints = 0;

    do {

        currentPoints = pointCloudStack.pointAccess->castedPointGeometry<double>();

        minX = std::min(minX,currentPoints.x);
        maxX = std::max(maxX, currentPoints.x);

        minY = std::min(minY,currentPoints.y);
        maxY = std::max(maxY, currentPoints.y);

        nPoints++;

        ok = pointCloudStack.pointAccess->gotoNext();

    } while (ok);

    if (nPoints <= 1) {
        std::cerr << "Error with file: " << inFile << ", not enough points! Aborting!" << std::endl;
        return std::nullopt;
    }

    GridInfos ret;
    ret.x0 = minX;
    ret.y0 = minY;

    double w = maxX - minX;
    double h = maxY - minY;

    ret.scale = std::sqrt(double(nPoints)/(gridScale*w*h));

    if (!std::isfinite(ret.scale)) {
        std::cerr << "Error with file: " << inFile << ", points cover surface invalid! Aborting!" << std::endl;
        return std::nullopt;
    }

    ret.height = std::ceil(ret.scale*h);
    ret.width = std::ceil(ret.scale*w);

    return ret;
}

inline float densityKernel(float dsqr) {
    return 1/std::max<float>(1,dsqr);
}

int processData(std::string const& inFile, GridInfos const& gridInfos) {

    //Open file
    auto pointCloudStackOpt = StereoVision::IO::openPointCloud(inFile);

    if (!pointCloudStackOpt.has_value()) {
        std::cerr << "Could not open file: " << inFile << "! Aborting!" << std::endl;
        return 1;
    }

    StereoVision::IO::FullPointCloudAccessInterface& pointCloudStack = pointCloudStackOpt.value();

    if (pointCloudStack.headerAccess == nullptr and pointCloudStack.pointAccess == nullptr) {
        std::cerr << "Error reading file: " << inFile << ", null accesss interfaces! Aborting!" << std::endl;
        return 1;
    }

    Multidim::Array<float,2> density(gridInfos.width, gridInfos.height);

    #pragma omp parallel for
    for (int i = 0; i < density.shape()[0]; i++) {
        for (int j = 0; j < density.shape()[1]; j++) {
            density.atUnchecked(i,j) = 0;
        }
    }

    bool ok = true;

    do {

        auto currentPoints = pointCloudStack.pointAccess->castedPointGeometry<double>();

        double x = gridInfos.scale*(currentPoints.x-gridInfos.x0);
        double y = gridInfos.scale*(currentPoints.y-gridInfos.y0);

        int x0 = std::max<int>(0,std::floor(x));
        int x1 = std::min(density.shape()[0]-1,x0+1);

        int y0 = std::max<int>(0,std::floor(y));
        int y1 = std::min(density.shape()[0]-1,y0+1);

        double densityLimit =
                (x1-x)*(y1-y)*density.atUnchecked(x0,y0) +
                (x-x0)*(y1-y)*density.atUnchecked(x1,y0) +
                (x1-x)*(y-y0)*density.atUnchecked(x0,y1) +
                (x-x0)*(y-y0)*density.atUnchecked(x1,y1);

        densityLimit *= gridInfos.scale*gridInfos.scale;

        std::cout << densityLimit << std::endl;

        #pragma omp parallel for
        for (int i = 0; i < density.shape()[0]; i++) {
            for (int j = 0; j < density.shape()[1]; j++) {
                float dx = i-x;
                float dy = j-y;
                float dsqr = dx*dx + dy*dy;
                density.atUnchecked(i,j) += densityKernel(dsqr);
            }
        }

        ok = pointCloudStack.pointAccess->gotoNext();

    } while (ok);

    std::cout.flush();
    return 0;

}

int main(int argc, char** argv) {

    const char* message = "Processed lidar data on the fly";
    constexpr char delimiter = '=';
    const char* version = "0.1";

    std::string inFile;

    try {

        TCLAP::CmdLine cmd(message, delimiter, version);

        TCLAP::UnlabeledValueArg<std::string> inputFileArg("inFile", "Input file",true,"","path to a point cloud or point cloud-like file");

        cmd.add(inputFileArg);

        cmd.parse(argc, argv);

        inFile = inputFileArg.getValue();

    } catch (TCLAP::ArgException &e) {

        std::cerr << "Command line error: " << e.error() << " for argument " << e.argId() << std::endl;
        return 1;

    }

    std::optional<GridInfos> gridInfosOpt = computeGridInfos(inFile);

    if (!gridInfosOpt.has_value()) {
        return 1;
    }

    return processData(inFile, gridInfosOpt.value());
}
