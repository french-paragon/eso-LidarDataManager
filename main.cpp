#include <iostream>
#include <string>
#include <filesystem>

#include <tclap/CmdLine.h>

int main(int argc, char** argv) {

    const char* message = "Processed lidar data on the fly";
    constexpr char delimiter = '=';
    const char* version = "0.1";

    std::string inFile;

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

    } catch (TCLAP::ArgException &e) {

        std::cerr << "Command line error: " << e.error() << " for argument " << e.argId() << std::endl;

    }


}
