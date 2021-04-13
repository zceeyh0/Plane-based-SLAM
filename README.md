# Plane-based-SLAM
Author: Yansong Huang.

This is a SLAM (Simultaneous Localisation And Mapping) framework combining plane features and point features for image processing. <br>
The language for this program is C++, and it has been coded and tested in Ubuntu 20.04.<br>
This program is a third-year project from Electronic and Electrical Department of University College London. It can be only used for academic purposes but not commercial purposes.<br>
For any questions regarding this project, please contact the author: zceeyh0@ucl.ac.uk<br>

To run this program, please follow the steps below (in Linux system):<br>

1. Make sure you have installed all related libraries and dependencies<br>
OpenCV: https://opencv.org/releases/<br>
PCL: https://pointclouds.org/downloads/ or git clone https://github.com/PointCloudLibrary/pcl/releases<br>
G2O: git clone https://github.com/RainerKuemmerle/g2o.git<br>
DBoW3: This is already contained in the project, to install it again , git clone https://github.com/rmsalinas/DBow3.git<br>

2. Download this program by git clone this repository, there should be seven folders and six single files<br>
bin: executable binary files to run the program<br>
build: essential files to build the project using CMake<br>
cmake_modules: modules to link the program to G2O and DBoW3 library<br>
data: RGB and depth images of online datasets from NYU-Depth V2 and TUM<br>
DBoW3: DBoW3 library for semantic loop closure detection<br>
include: header files for the project<br>
src: source code for the project<br>

3. Change directory to the build folder and build the project:<br>
cd build<br>
cmake ..<br>
make<br>
If no error pops up, the compilation is done successfully.<br>
If you have learned about CMake compilation, you can modify the CMakeLists files in the current directory and the src folder<br>

4. Before running the compiled program, here are some tips:<br>
main.cpp contains the general steps to implement this SLAM algorithm. Two parameters min and max are set as 180 to 280, which are the indices of input scans.<br>
slamFunc.cpp includes details of each function in SLAM. Change the directory in "readImage" function to use different datasets.<br>
If it's your first time running this program and you only need a general view, I suggest to comment the loopClosure function in main.cpp since this costs a lot of time and resources. Leaf size of Voxel Grid filter in slamFunc.cpp also can be increased to speed up the program, but the clarity of the point cloud will be reduced.<br>

5. To run the program, type cd .. and then bin/main<br>
The program will firstly ask you to choose the type of fine alignment method, from my experiments, 1 is more stable than 2 and 3 all the time.<br>
A window showing the global point cloud will pops up. Click R on your keyboard to reset the coordinates and visualise the updating point cloud.<br>
When the program finishes, a .pcd file will be saved in the current directory. Type pcl_viewer result.pcd to view the resulting point cloud model.<br>

# Enjoy!

