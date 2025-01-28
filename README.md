# LidarDataManager

Lidar Data manager is a tool for quick conversion and processing of Lidar Data

## Setting up the environment

Before compiling, you need to make sure a few things have been installed on your system:

- CMake (version 3.5 is the minimum required)
- Eigen3
- LibStereoVision, [which is available on github](https://github.com/french-paragon/LibStevi)
- PkgConfig
- Proj
- TClap

## Compiling

Once the required dependencies are installed, the application can be built using cmake. We recommand you perform the build steps outside of the source directory.

```
cd ..
mkdir build
cd build
cmake ../LidarDataManager
make
```

## Features

- Format conversion between different point cloud formats (PCD ASCII/Binary, LAS v1.2-1.4)
- Point cloud filtering and decimation
- Attribute management
- Region of Interest (ROI) selection
- Coordinate Reference System (CRS) transformation
- Line and return filtering

## Usage

```bash
lidarDataManager [options] <input_file>
```

## Options

### Data Filtering

- `--remove_attribute=<attribute>`: Remove specific attributes from the data (can be used multiple times)
- `--remove_all_attributes`: Remove all non-geometric data
- `--remove_color`: Remove color information if present

### Output Control

- `-f, --format=<format>`: Specify output format
  - Supported formats: pcd-ascii, pcd-bin, lasv14, lasv13, lasv12

### Point Selection

- `-l, --line=<index>`: Export specific line index (use -1 for no limit)
- `-r, --returns=<index>`: Set maximum return index (use -1 for no limit)
- `-n, --number=<count>`: Limit maximum number of output points (use -1 for no limit)
- `-d, --density=<value>`: Set maximum point density (points per m²)

### Spatial Control

- `--roi="x0,y0,z0,dx,dy,dz,rx,ry,rz"`: Define Region of Interest
  - `x0,y0,z0`: Origin of rectangular cuboid
  - `dx,dy,dz`: Cuboid dimensions
  - `rx,ry,rz`: Rotation angles around origin

### Coordinate Reference System

- `--outcrs=<crs>`: Set output CRS (PROJ format or EPSG code)
- `--incrs=<crs>`: Override input data CRS (PROJ format or EPSG code)

### General Options

- `-h, --help`: Display usage information
- `--version`: Display version information
- `--`: Ignore remaining labeled arguments

## Input

The tool requires a path to a point cloud or point cloud-like file as input.

## Example

```bash
lidarDataManager input.las -f pcd-ascii --density=10 --roi="0,0,0,100,100,50,0,0,0" --outcrs="EPSG:4326"
```

This command will:

1. Read the input LAS file
2. Convert it to PCD ASCII format
3. Limit point density to 10 points/m²
4. Apply ROI filtering for a 100x100x50m cuboid
5. Transform coordinates to EPSG:4326 (WGS84)
