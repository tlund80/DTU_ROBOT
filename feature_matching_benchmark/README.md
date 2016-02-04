# Feature Matching Benchmark for 3D Shape Features

This application contains code for benchmarking 3D feature descriptors. If you use our code for academic purposes, please cite our publication:

Buch, A. G. & Krüger, N. (2015). Which 3D Feature Should I Use? *International Journal of Computer Vision*. Submitted.

## General
The following has been tested on Ubuntu 14.04.

## Installation
In the following installation guide, we assume that all code will reside in the same root directory:
```sh
mkdir -p ~/workspace
```

First install **system dependencies**:
```sh
sudo apt-get install cmake g++ libboost-dev libeigen3-dev libopencv-dev libvtk5-dev 
```

Now we need a recent installation of the **Point Cloud Library** (PCL). The code is tested on PCL 1.7.2, which includes a couple of bugfixes that we contributed to PCL. We make no guarantees for the result should you try other versions. We therefore recommend installing from source the following way:
```sh
cd ~/workspace
git clone https://github.com/PointCloudLibrary/pcl.git --branch pcl-1.7.2 pcl
cd pcl ; mkdir build ; cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_tools=OFF ..
make -jN all
```
replacing N with the desired number of cores you want to use for speeding up compilation.

The benchmark code relies on our own library, **CoViS**, which sits on top of OpenCV and PCL. After PCL has been compiled, you must also compile CoViS:
```sh
cd ~/workspace
git clone git@gitlab.com:caro-sdu/covis.git covis
cd covis ; mkdir build ; cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -jN covis
```

Finally, compile the benchmark code from our **CoViS application** repository:
```sh
cd ~/workspace
git clone git@gitlab.com:caro-sdu/covis-app.git covis-app
cd covis-app/feature_matching_benchmark ; mkdir build ; cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -jN all
```
