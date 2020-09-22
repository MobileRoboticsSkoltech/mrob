## Python short instructions:

First, compile the project as explained in the README. We recommend to use the path ~/soft. Then add the following line to .bashrc or .bash_aliases:

`echo "export PYTHONPATH=${PYTHONPATH}:${HOME}/soft/mrob/lib" >> ~/.bash_aliases`

Make sure that the path to lib is correct

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
