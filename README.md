# Plane-based-SLAM
Author: Yansong Huang
This is a SLAM (Simultaneous Localisation And Mapping) framework combining plane features and point features for image processing. 
The language for this program is C++, and it is tested in Ubuntu 20.04.
This program is a third-year project from Electronic and Electrical Department of University College London. It can be only used for academic purposes but not commercial purposes.
For any questions regarding this project, please contact the author: zceeyh0@ucl.ac.uk

To run this program, please follow the steps below (in Linux system):

1. Make sure you have installed all related libraries and dependencies
OpenCV: https://opencv.org/releases/
PCL: http://www.pointclouds.org/downloads/windows.html or git clone https://github.com/PointCloudLibrary/pcl/releases
G2O: git clone https://github.com/RainerKuemmerle/g2o.git
DBoW3: This is already contained in the project, to install it again , git clone https://github.com/rmsalinas/DBow3.git

2. Download this program by git clone this repository, there should be seven folders and six single files
bin: executable binary files to run the program
build: essential files to build the project using CMake
cmake_modules: modules to link the program to G2O and DBoW3 library
data: RGB and depth images of online datasets from NYU-Depth V2 and TUM
DBoW3: DBoW3 library for semantic loop closure detection
include: header files for the project
src: source code for the project

3. Change directory to the build folder and build the project:
cd build
cmake ..
make
If no error pops up, the compilation is done successfully.
If you have learned about CMake compilation, you can modify the CMakeLists files in the current directory and the src folder

4. Before running the compiled program, here are some tips:
main.cpp contains the general steps to implement this SLAM algorithm. Two parameters min and max are set as 180 to 280, which are the indices of input scans.
slamFunc.cpp includes details of each function in SLAM. Change the directory in "readImage" function to use different datasets.
If it's your first time running this program and you only need a general view, I suggest to comment the loopClosure function in main.cpp since this costs a lot of time and resources. Leaf size of Voxel Grid filter in slamFunc.cpp also can be increased to speed up the program, but the clarity of the point cloud will be reduced.

5. To run the program, type cd .. and then bin/main
The program will firstly ask you to choose the type of fine alignment method, from my experiments, 1 is more stable than 2 and 3 all the time.
A window showing the global point cloud will pops up. Click R on your keyboard to reset the coordinates and visualise the updating point cloud.
When the program finishes, a .pcd file will be saved in the current directory. Type pcl_viewer result.pcd to view the resulting point cloud model.

# Enjoy!
