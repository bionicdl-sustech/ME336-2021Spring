# GeoGrasp
Geometry-based method for computing grasping points on 3D point clouds. Find more details at: https://www.researchgate.net/publication/331358070_Fast_Geometry-based_Computation_of_Grasping_Points_on_Three-dimensional_Point_Clouds

The original github address is https://github.com/yayaneath/GeoGrasp. We modified it to PCL 1.8, and support python call method.

# Requirements
The package has been tested on Ubuntu 18.04. GeoGrasp is wrapped in a python package with the following dependecies:

- pybind11
- PCL 1.8
- vtk 7.1

# usage
Set your python version in CMakelists.txt, and make a build folder for compiling.

> $ mkdir build   
> $ cd build   
> $ cmake ..   
> $ make   
> $ cp ./*.so ../


# Notes

- We have some problems when using pybind11 with apt installed pcl, as it's default vtk version is 6.3. 
Please install the the vtk 7.1 and pcl1.8 from source.

- There are some confliction between PCL, pybind11, and python package open3d. Do not use them in same time.

# Citation
[1] Zapata-Impata, B. S., Mateo, C. M., Gil, P., & Pomares, J. (2017). Using Geometry to Detect Grasping Points on 3D Unknown Point Cloud. In Proceedings of the 14th International Conference on Informatics in Control, Automation and Robotics (ICINCO) 2017 (Vol. 2, pp. 154–161). Best Paper Award. SCITEPRESS - Science and Technology Publications. https://doi.org/10.5220/0006470701540161

[2] Zapata-Impata, B. S., Gil, P., Pomares, J., & Torres, F. (2019). Fast geometry-based computation of grasping points on three-dimensional point clouds. International Journal of Advanced Robotic Systems, 16(1), 172988141983184. https://doi.org/10.1177/1729881419831846
