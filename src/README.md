# MROB Cpp source
Cpp code is included here, that is, most of the algorithms and structures.


## Coding conventions
Coding conventions are necessary to maintain homogeneity and readability across all the project. Here are some conventions that we _should_ follow:

* BSD/Allman conventions: -like, ie. brace on the next line from a control statement, indented on the same level. In switch-case statements the cases are on the same indent level as the switch statement.
* Indents use 4 spaces instead of tabs. Tabs are not used.
* Class and struct names camel-case and beginning with an uppercase letter. For example `BaseClass`, `Arun`.
* Variables are in lower case, either camel-case `odometryObs` or separated by underscores `odometry_obs`. Member variables have an underscore appended `localNodes_`. For matrices, in order to maintain a coherent mathematical description, variables may be upper case.
* Functions are in lower case, and can be separated by underscores, e.g. `solve_iterative()` or in lower camel-case, what is important is to be as descriptive as possible, not contracting names for the sake of understandability.
* File names: Should be all lowercase, and can be separated by underscores, `example_align_methods.cpp`
* Constants and enumerations are in uppercase. For example `M_PI`.
* Class definitions proceed in the following order:

  - public enums
  - public constructors and the destructor
  - public virtual functions
  - public non-virtual member functions
  - public static functions
  - public member variables
  - public static variables
  - repeat all of the above in order for protected definitions, and finally private
* Header files are commented using one-line comments beginning with / &ast &ast to mark them, comments are important.


### Eclipse
for Eclipse users:
* Make: on project, properties, C++build, select the build command `make -C ${projDirPath}./build`
* Find Eigen symbols: project, properties, C++ general, paths and symbols: add Eigen source, for example at /usr/local/include/eigen3
* Update template: windows - preferences - C/C++ - code style - templates
