# Overview
This driver is implement with libfranka and pybind11, and provides python3 API.

# How to use
Before compiling the files, make sure you have install [pybind11](https://github.com/pybind/pybind11), [libfranka](https://frankaemika.github.io/docs/installation_linux.html), [EIGEN3](http://eigen.tuxfamily.org/index.php?title=Main_Page), and libpoco-dev.

Then locate the RobotDirver, open the terminal, transfer to the python environment you used,
and type commands as below:
```
mkdir build
cd build
cmake ..
make -j4
cp RobotDriver.cpython-35m-x86_64-linux-gnu.so ../../RobotDriver.cpython-35m-x86_64-linux-gnu.so
```
Note: as the python version is 3.5, the library name is '**cpython-35m**', 
for other version, the name will be changed.
