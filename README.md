# MROB: Mobile Robotics library
The Skoltech Mobile Robotics library (mrob) is our common framework for implementing our robotics research and projects. It includes a core set of functionalities including perception, path planning and optimization. The present library is meant to be a self-contained library.
* [common](https://github.com/MobileRoboticsSkoltech/mrob/tree/master/src/common): common matrix definitions and typedefs.
* [geometry](https://github.com/MobileRoboticsSkoltech/mrob/tree/master/src/geometry): Geometric transformations, mostly Rotations and Rigid Body Transformations in 3D.
* [Fgraph](https://github.com/MobileRoboticsSkoltech/mrob/tree/master/src/FGraph): Factor Graph (WIP)
* [PCReg](https://github.com/MobileRoboticsSkoltech/mrob/tree/master/src/PCRegistration): Point Cloud Registration (WIP)
* [mrobPy](https://github.com/MobileRoboticsSkoltech/mrob/tree/master/mrobpy) Python bindings (using pybind11) for some of the above methods.

## Dependencies
* C++'14
* CMake
* [Eigen](https://gitlab.com/libeigen/eigen) (included as a submodule)
* [pybind11](https://github.com/pybind/pybind11) (included as a submodule)
  - python3-distutils
  - python3-dev

`sudo apt install build-essential cmake python3-distutils python3-dev`




## Repository 
Standard github cloining, adding the recursive term for submodules.
`git clone --recursive git@github.com:MobileRoboticsSkoltech/mrob.git`


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


