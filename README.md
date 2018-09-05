# skmr
The Skoltech Mobile Robotics library (skmr) is our common framework for implementing our robotics research and projects. It includes a core set of functionalities including perception, path planning and optimization. The present library is meant to be a self-contained library.
* [common](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/skmr/browse/src/common): common matrix definitions and typedefs.
* [SE3](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/skmr/browse/src/SE3): Rigid Body Transformations library.
* [Fgraph](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/skmr/browse/src/FGraph): Factor Graph (WIP)
* [PCReg](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/skmr/browse/src/PCRegistration): Point Cloud Registration (WIP)
* [TemplateNew](): Template for new projects.

## Dependencies
Eigen, C++'11

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


## Repository configuration


Clone the project from CDISE bitbucket:

`git -c http.sslVerify=false clone 	https://git@cdise-bitbucket.skoltech.ru/scm/mr/skmr.git`

We need to disable the certificate because server certificate verifications fails.

`git remote remove origin`

Create a new repository, either at the group or private.

`git remote add origin ssh://yourUserName@cdise-bitbucket.skoltech.ru:7999/mr/yourNewProject.git`

`git push -u origin master`


To push and do other operations:
* Obtain the .pem certificate directly from the web-page https://cdise-bitbucket.skoltech.ru
* Configure .git/config, by adding the following lines: 
```
[http]
	sslCAInfo=/home/yourUserName/sk-bitbucket.pem
```

When a project is finished and tested, it should be merged back to the original repository. Use the merge request feature.

## Installation
```
cd skmr
mkdir build
cd build
cmake ..
make -j
```


### Eclipse
for Eclipse users:
* Make: on project, properties, C++build, select the build command `make -C ${projDirPath}./build`
* Find Eigen symbols: project, properties, C++ general, paths and symbols: add Eigen source, for example at /usr/local/include/eigen3


