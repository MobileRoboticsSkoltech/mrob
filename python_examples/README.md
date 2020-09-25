# Python examples using MROB

First, compile the project as explained in the README. We recommend to use the path ~/soft. Then add the following line to .bashrc or .bash_aliases:

`echo "export PYTHONPATH=${PYTHONPATH}:${HOME}/soft/mrob/lib" >> ~/.bash_aliases`

Make sure that the path to lib is correct


You can run any of the examples, for instance running the Manhattan 3500 benchmark for 2d Pose slam:
`python3 FGrap_M3500`



## Other uses
You can write your own code using mrob just following the examples.
```
python3
import mrob
T = mrob.geometry.SE3()
```

There are some namespaces dividing the library in utilities, such as *geometry* for transformations.

You can also use a less verbose way of calling rmob library by specifying loading functions. We encourage the namespace-like style

```
python3
from mrob.geometry import SE3
T = SE3()
```
