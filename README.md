# skmr
The Skoltech Mobile Robotics library (skmr) is our common framework for implementing our robotics research and projects. It includes a core set of functionalities including perception, path planning and optimization. The present library is meant to be a self-contained library.

## Dependencies
Eigen, C++'11

## Coding conventions
Coding conventions are necessary to maintain homogeneity and readability across all the project. Here are some conventions that we _should_ follow:

* BSD/Allman conventions: -like, ie. brace on the next line from a control statement, indented on the same level. In switch-case statements the cases are on the same indent level as the switch statement.
* Indents use 4 spaces instead of tabs. Tabs are not used.
* Class and struct names camel-case and beginning with an uppercase letter.
* Variables are in lower camel-case. Member variables have an underscore appended. For example `odometryObs`, `localNodes_`.
* Constants and enumerations are in uppercase. For example M_PI.
* Class definitions proceed in the following order:

  - public constructors and the destructor
  - public virtual functions
  - public non-virtual member functions
  - public static functions
  - public member variables
  - public static variables
  - repeat all of the above in order for protected definitions, and finally private
* Header files are commented using one-line comments beginning with / &ast &ast to mark them, comments are important.

