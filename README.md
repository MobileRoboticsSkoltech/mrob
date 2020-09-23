<p align="center">
  <img src="http://sites.skoltech.ru/app/data/uploads/sites/50/2018/02/mr_animate1.gif" width="450">
</p>

# MROB: Mobile Robotics library
The Skoltech Mobile Robotics library (mrob) is our common framework for implementing our robotics research and projects. It includes a core set of functionalities such as geometric transformations (SE3), factor graphs for general state estimation, optimization, 3D point cloud registration and more to come.

The general structure for the algorithms implemented:
* [common](https://github.com/MobileRoboticsSkoltech/mrob/tree/master/src/common): common matrix definitions and typedefs.
* [geometry](https://github.com/MobileRoboticsSkoltech/mrob/tree/master/src/geometry): Geometric transformations, mostly Rotations and Rigid Body Transformations in 3D.
* [Fgraph](https://github.com/MobileRoboticsSkoltech/mrob/tree/master/src/FGraph): Factor Graphs for state estimation
* [PCReg](https://github.com/MobileRoboticsSkoltech/mrob/tree/master/src/PCRegistration): Point Cloud Registration.
* [mrobPy](https://github.com/MobileRoboticsSkoltech/mrob/tree/master/mrobpy) Python bindings (using pybind11) for the above methods.

## Python Examples
The library is mainly designed to run in python, that is, algorithms are written in cpp and bind to python for general purpose use.
We provide some examples in [python_examples](https://github.com/MobileRoboticsSkoltech/mrob/tree/master/python_examples) for more details.


## Dependencies
The present library is meant to be a self-contained library. However, there are few dependencies:
* C++'14
* CMake
* [Eigen](https://gitlab.com/libeigen/eigen) (included as a submodule)
* [pybind11](https://github.com/pybind/pybind11) (included as a submodule)
  - python3-distutils
  - python3-dev

This is the list of required packages to install:
`sudo apt install build-essential cmake python3-distutils python3-dev`


## Repository 
Standard github cloning, adding the recursive term for submodules.

`git clone --recursive git@github.com:MobileRoboticsSkoltech/mrob.git`

If there was ever a submodule update (not frequently) the command to use:

`git submodule update --recursive`

## Installation
```
cd mrob
mkdir build
cd build
cmake ..
make -j
```


## License
3-Clause BSD License


