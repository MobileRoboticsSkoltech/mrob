# SE3
The Special Euclidean Group SE(n) is the group representing rotations and translations, that is,
rigid body transformations, any any dimesion n. For this particular library we are interested in 3D and 2D.
Associated to the groups of RBT, there is the Lie algebra se3 representing the same transformation in the tangent space around the identity.
Particularly for n=3, xi =\[w , v\] in Re^6, where w in Re^3 represents the rotation and v the translation.
So this library provides the mapping from SE(n), SO(n) to a vector of real values and 
The present library is meant to be part of the mrob library, by implementing methods to solve the representation of Rigid Body Transfromations.

## Dependencies
C++'14, Eigen



## Coding conventions
Please check the common conventions for [mrob](https://cdise-bitbucket.skoltech.ru/projects/MR/repos/mrob/browse).

