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

## Usage

For usage information, call the tool with the -h flag:

```
./lidarDataManager -h
```

## Containerized application

We do provide a containerized version of the application. To build the corresponding docker container, use the Dockerfile provided. In the root source directory you can run:

```
docker build -t lidardatamanager --no-cache .
```

Then, to execute the tool, run:

```
docker run lidardatamanager [Options]
```

(You might need to configure a volume to transfert the input files to the container, output is via stdout so just redirect the output of the container)
