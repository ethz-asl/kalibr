# Import the numpy to Eigen type conversion.
import numpy_eigen
import os

isCompiled = False
pathToSo = os.path.dirname(os.path.realpath(__file__))
if os.path.isfile(os.path.join(pathToSo,"libaslam_cv_python.so")):    
    # Import the the C++ exports from your package library.
    from libaslam_cv_python import *
    # Import other files in the directory
    # from mypyfile import *
    isCompiled = True
else:
    print "Warning: the package aslam_cv_python is not compiled. Type 'rosmake aslam_cv_python' if you need this."
    PACKAGE_IS_NOT_COMPILED = True;
