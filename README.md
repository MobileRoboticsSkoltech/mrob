# MROB: Mobile Robotics library
The Skoltech Mobile Robotics library (mrob) is our common framework for implementing our robotics research and projects. It includes a core set of functionalities including perception, path planning and optimization. The present library is meant to be a self-contained library.
* [common](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/mrob/browse/src/common): common matrix definitions and typedefs.
* [SE3](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/mrob/browse/src/SE3): Rigid Body Transformations library.
* [Fgraph](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/mrob/browse/src/FGraph): Factor Graph (WIP)
* [PCReg](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/mrob/browse/src/PCRegistration): Point Cloud Registration (WIP)
* [mrobPy](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/mrob/browse/mrobpy) Python bindings (using pybind11) for some of the above methods.

## Dependencies
* C++'14
* CMake
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) (requires installation)
* [pybind11](https://github.com/pybind/pybind11)
  - python3-distutils
  - python3-dev

`sudo apt install build-essential cmake python3-distutils python3-dev`

## Coding conventions
Coding conventions are necessary to maintain homogeneity and readability across all the project. Here are some conventions that we _should_ follow:

* BSD/Allman conventions: -like, ie. brace on the next line from a control statement, indented on the same level. In switch-case statements the cases are on the same indent level as the switch statement.
* Indents use 4 spaces instead of tabs. Tabs are not used.
* Class and struct names camel-case and beginning with an uppercase letter. For example `BaseClass`, `Arun`.
* Variables are in lower camel-case. For example `odometryObs`. Member variables have an underscore appended `localNodes_`.
* Functions are in lower case, and can be separated by underscores, e.g. `solve_iterative()` or in lower camel-case, what is important is to be as descriptive as possible, not contracting names for the sake of understandability.
* File names: Should be all lowercase, and can be separated by underscores, `example_align_methods.cpp`
* Constants and enumerations are in uppercase. For example `M_PI`.
* Class definitions proceed in the following order:

  - public constructors and the destructor
  - public virtual functions
  - public non-virtual member functions
  - public static functions
  - public member variables
  - public static variables
  - repeat all of the above in order for protected definitions, and finally private
* Header files are commented using one-line comments beginning with / &ast &ast to mark them, comments are important.


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


### Eclipse
for Eclipse users:
* Make: on project, properties, C++build, select the build command `make -C ${projDirPath}./build`
* Find Eigen symbols: project, properties, C++ general, paths and symbols: add Eigen source, for example at /usr/local/include/eigen3


