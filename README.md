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
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) (requires installation)
* [pybind11](https://github.com/pybind/pybind11)
  - python3-distutils
  - python3-dev

`sudo apt install build-essential cmake python3-distutils python3-dev libeigen3-dev`




## Repository configuration SSH

Create a SSH key and configure your account appropriately.
Clone the project from CDISE bitbucket:

`git clone --recursive ssh://git@cdise-bitbucket.skoltech.ru:7999/mr/mrob.git`


`git remote remove origin`

Create a new repository at your private space.

`git remote add origin ssh://yourUserName@cdise-bitbucket.skoltech.ru:7999/mr/yourNewProject.git`

`git push -u origin master`


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


