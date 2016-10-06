C++ Matlab Interface (written by Michael Neunert, neunertm@gmail.com)
====================

This is a basic C++ Wrapper for Matlab. You can use it to send and read data/variables from Matlab and execute Matlab commands. It is less efficient than directly working with mxArrays but it provides a very intuitive and easy to use interface.

1. Installation

1a. Schweizer-Messer
If you do not build it within the Schweizer-Messer written by Paul Furgale, you will have to remove SM specific types, manually compile it and then link against libeng and libmx, which should either be in your Matlab folder or common library directories.

1b. Packages
Under linux you have to install csh. Under Ubuntu/Mint you can simply type
> sudo apt-get install csh


2. Usage

2a. Initialization
First you will have to create an object of the type sm::matlab::Engine, e.g.:
> sm::matlab::Engine myMatlabEngine;
Once the object is created, Matlab will launch in the background. Hence, this command might take some time. Even though run in the background, you have full access to almost all Matlab commands including opening windows e.g. for plotting.

2b. Execute a command or launch an m-file
Just hit executeCommand("command"), e.g.
> myMatlabEngine.executeCommand("plot([1 2 3]");
The return value is the Matlab output of the command

2c. Send data to Matlab
You can simply send data to Matlab using put(variableName, value) (for standard types) or putEigen() (for Eigen Matrices and Vectors), e.g.
> myMatlabEngine.put("a", 4);
> Eigen::Vector2d B;
> B << 2.0, 4.2;
> myMatlabEngine.putEigen("B", B);
Now, you can process the data in Matlab

2d. Get data from Matlab
Essentially, it is the same as using the put() function: Just use get(variableName, value) or getEigen(variableName, value). Please note that Matlab normally stores all data types as doubles, so use doubles or Eigen double matrices/vectors as return containers, e.g.:
> double a;
> myMatlabEngine.get("a", a);
> Eigen::Matrix4d C;
> myMatlabEngine.getEigen("C", C);

2e. Interactive Console
You can also open up an interactive console which almost behaves like the regular Matlab command line. Just enter
> myMatlabEngine.openCommandLine();
To exit, type "quit" or "exit". This intentionally blocks the Matlab commands "quit" and "exit". In case you want to close Matlab early, use executeCommand(). However, calling the Engine class constructor is recommended as early exit might make the object unusable.
Under Windows, you can also use setVisibility() to show/hide a Matlab console. Unfortunately, Matlab does not provide this functionality under Linux/MacOSX but openCommandLine() should be a good alternative.

2f. More
For more options, take a look at the Engine.hpp header which shows all available functions.


3. Example

For an example see demo.cpp


ENJOY!!

