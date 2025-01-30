#include <iostream>
#include <string>
#include <filesystem>

#include <tclap/CmdLine.h>

#include <StereoVision/io/pointcloud_io.h>
#include <StereoVision/io/las_pointcloud_io.h>
#include <StereoVision/io/pcd_pointcloud_io.h>

#include "processingBlocks/aliasheaderattributes.h"
#include "processingBlocks/regionofinterestselector.h"
#include "processingBlocks/attributebasedselector.h"
#include "processingBlocks/pointsattributesfilters.h"
#include "processingBlocks/crsconversion.h"

int main(int argc, char** argv) {

    const char* message = "Processed lidar data on the fly";
    constexpr char delimiter = '=';
    const char* version = "0.1";

    std::string inFile;

    std::string inCrs = "";
    std::string outCrs = "";

    std::string roi = "";

    double density = std::numeric_limits<double>::infinity();
    int number = -1;

    int returnCap = -1;
    int lineIdx = -1;

    std::string outFormat = "";

    bool removeColor = false;
    bool removeAllAttributes = false;
    std::vector<std::string> attributes2filter;

    try {

        TCLAP::CmdLine cmd(message, delimiter, version);

        TCLAP::UnlabeledValueArg<std::string> inputFileArg("inFile", "Input file",true,"","path to a point cloud or point cloud-like file");

        TCLAP::ValueArg<std::string> inCrsArg("", "incrs", "Override the crs of the input data", false, "", "any string that can be parsed by PROJ, e.g. WTK string or \"EPSG:####\" codes");
        TCLAP::ValueArg<std::string> outCrsArg("", "outcrs", "The crs to use for the output data. If not specified, then no CRS transform is done.", false, "", "any string that can be parsed by PROJ, e.g. WTK string or \"EPSG:####\" codes");

        const char* roiDescr = "A description of a region of interest formatted as \"x0,y0,z0,dx,dy,dz,rx,ry,rz\",\n"
                "with x0,y0,z0 the origin of the Rectangular cuboid,"
                "dx,dy,dz the offsets for span the unrotated cuboid and"
                "rx,ry,rz the axis angle of the rotation around x0,y0,z0 to apply";
        TCLAP::ValueArg<std::string> roiArg("", "roi", "Definition of a region of interest", false, "", roiDescr);

        TCLAP::ValueArg<double> densityArg("d", "density", "The maximal density of the point cloud, as points per m^2", false, std::numeric_limits<double>::infinity(), "A double");

        TCLAP::ValueArg<int> numberArg("n", "number", "The maximal number of points in the output point cloud. The tool will try to spead the output points as uniformly as possible.",
                                       false, -1, "An int, if below 0 then no limits are imposed");

        TCLAP::ValueArg<int> returnCapArg("r", "returns", "The maximal return index to use.",
                                       false, -1, "An int, if below 0 then no limits are imposed");

        TCLAP::ValueArg<int> lineArg("l", "line", "The index of the line to export.",
                                       false, -1, "An int, if below 0 then no limits are imposed");

        std::vector<std::string> allowedOutFormats;
                allowedOutFormats.push_back("pcd-ascii");
                allowedOutFormats.push_back("pcd-bin");
                allowedOutFormats.push_back("lasv14");
                allowedOutFormats.push_back("lasv13");
                allowedOutFormats.push_back("lasv12");
                TCLAP::ValuesConstraint<std::string> allowedOutFormatsConstraint( allowedOutFormats );
        TCLAP::ValueArg<std::string> formatArg("f", "format", "Output format", false, "pcd-ascii", &allowedOutFormatsConstraint);

        TCLAP::SwitchArg removeColorArg("", "remove_color", "remove the color data, if present");
        TCLAP::SwitchArg removeAllAttributesArg("", "remove_all_attributes", "remove all data that is not geometry");
        TCLAP::MultiArg<std::string> removeAttributeArg("", "remove_attribute", "filter out an attribute in the data", false, "string, namming an attribute");

        cmd.add(inputFileArg);

        cmd.add(inCrsArg);
        cmd.add(outCrsArg);
        cmd.add(roiArg);
        cmd.add(densityArg);
        cmd.add(numberArg);
        cmd.add(returnCapArg);
        cmd.add(lineArg);
        cmd.add(formatArg);

        cmd.add(removeColorArg);
        cmd.add(removeAllAttributesArg);
        cmd.add(removeAttributeArg);

        cmd.parse(argc, argv);

        inFile = inputFileArg.getValue();

        if (inCrsArg.isSet() and outCrsArg.isSet()) {
            std::string inCrs = inCrsArg.getValue();
            std::string outCrs = outCrsArg.getValue();
        }

        if (roiArg.isSet()) {
            roi = roiArg.getValue();
        }

        density = densityArg.getValue();
        number = numberArg.getValue();
        returnCap = returnCapArg.getValue();
        lineIdx = lineArg.getValue();

        outFormat = formatArg.getValue();

        removeColor = removeColorArg.isSet();
        removeAllAttributes = removeAllAttributesArg.isSet();

        attributes2filter = removeAttributeArg.getValue();

    } catch (TCLAP::ArgException &e) {

        std::cerr << "Command line error: " << e.error() << " for argument " << e.argId() << std::endl;
        return 1;

    }

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

    //process stack

    //do the filtering first (so that the points are removed)

    //region of interest
    std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> roiSelector =
        RegionOfInterestSelector::setupRoiSelection(pointCloudStack.pointAccess, roi);

    if (roiSelector != nullptr) {
        pointCloudStack.pointAccess = std::move(roiSelector);
    }

    if (density > 0 and density < std::numeric_limits<double>::infinity()) {
        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> densitySelector =
                AttributeBasedSelector::setupAttributeBasedSelector(pointCloudStack.pointAccess,
                                                                    "densityFilterAttr",
                                                                    AttributeBasedSelector::SmallerOrEqual,
                                                                    density);

        if (densitySelector != nullptr) {
            pointCloudStack.pointAccess = std::move(densitySelector);
        }
    }

    if (number > 0) {
        std::cerr << "Number based filtering not implemented yet, ignoring argument" << std::endl;
    }

    if (returnCap > 0) {

        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> returnNumberSelector =
                AttributeBasedSelector::setupAttributeBasedSelector(pointCloudStack.pointAccess,
                                                                    "returnNumber",
                                                                    AttributeBasedSelector::SmallerOrEqual,
                                                                    returnCap);

        if (returnNumberSelector != nullptr) {
            pointCloudStack.pointAccess = std::move(returnNumberSelector);
        }
    }

    if (lineIdx >= 0) {

        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> lineSelector =
                AttributeBasedSelector::setupAttributeBasedSelector(pointCloudStack.pointAccess,
                                                                    "lineNumber",
                                                                    AttributeBasedSelector::Equal,
                                                                    lineIdx);

        if (lineSelector != nullptr) {
            pointCloudStack.pointAccess = std::move(lineSelector);
        }
    }

    //then processing (only on the leftover points).

    std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> attributesFilter =
            PointsAttributesFilters::setupPointAttributeFiltering(pointCloudStack.pointAccess,
                                                                  removeColor,
                                                                  attributes2filter,
                                                                  removeAllAttributes);

    if (attributesFilter != nullptr) {
        pointCloudStack.pointAccess = std::move(attributesFilter);
    }

    //crs conversion
    if (!outCrs.empty()) {
        std::string inCrsVal;

        std::optional<StereoVision::IO::PointCloudGenericAttribute> inCrsAttr =
                pointCloudStack.headerAccess->getAttributeByName("crs");

        if (inCrsAttr.has_value()) {
            inCrsVal = StereoVision::IO::castedPointCloudAttribute<std::string>(inCrsAttr.value());
        } else {
            inCrsVal = inCrs;
        }

        if (inCrsVal.empty()) {
            std::cerr << "Could not get input crs info, crs conversion error!";
            return 1;
        }

        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> crsConvertor =
                CrsConversion::setupCrsConversion(pointCloudStack.pointAccess,
                                                  inCrs,
                                                  outCrs);

        if (crsConvertor == nullptr) {
            std::cerr << "Error building crs converter, crs conversion error!";
            return 1;
        }

        AliasHeaderAttributes::AliasMap headerAlias;
        headerAlias["crs"] = outCrs;

        pointCloudStack.headerAccess = std::make_unique<AliasHeaderAttributes>(std::move(pointCloudStack.headerAccess), headerAlias);
        pointCloudStack.pointAccess = std::move(crsConvertor);

    }

    //write file

    if (outFormat == "lasv13" or outFormat == "lasv12") {
        std::cerr << "Older LAS version unsupported yet" << std::endl;
        return 1;
    } else if (outFormat == "lasv14") {
        bool ok = StereoVision::IO::writePointCloudLas(std::cout, pointCloudStack);

        if (!ok) {
            std::cerr << "Error writing point cloud data to stdout!" << std::endl;
            return 1;
        }
    } else if (outFormat == "pcd-ascii" or outFormat == "pcd-bin") {

        StereoVision::IO::PcdDataStorageType dataStorageType = StereoVision::IO::PcdDataStorageType::ascii;

        if (outFormat == "pcd-bin") {
            dataStorageType = StereoVision::IO::PcdDataStorageType::binary;
        }

        bool ok = StereoVision::IO::writePointCloudPcd(std::cout, pointCloudStack, dataStorageType);

        if (!ok) {
            std::cerr << "Error writing point cloud data to stdout!" << std::endl;
            return 1;
        }
    }


    return 0;

}
