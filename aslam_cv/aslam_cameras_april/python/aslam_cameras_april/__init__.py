# Import the numpy to Eigen type conversion.
import numpy_eigen
import os
import aslam_cv

isCompiled = False
pathToSo = os.path.dirname(os.path.realpath(__file__))
if os.path.isfile(os.path.join(pathToSo,"libaslam_cameras_april_python.so")):    
    # Import the the C++ exports from your package library.
    from libaslam_cameras_april_python import *
    # Import other files in the directory
    # from mypyfile import *
    isCompiled = True
else:
    print "Warning: the package aslam_cameras_april_python is not compiled."
    PACKAGE_IS_NOT_COMPILED = True;
