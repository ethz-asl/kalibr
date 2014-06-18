# Import the numpy to Eigen type conversion.
import roslib; roslib.load_manifest('numpy_eigen'); import numpy_eigen
import os

isCompiled = False
pathToSo = os.path.dirname(os.path.realpath(__file__))
if os.path.isfile(os.path.join(pathToSo,"libincremental_calibration_python.so")):    
    roslib.load_manifest('aslam_backend_python'); import aslam_backend
    # Import the the C++ exports from your package library.
    from libincremental_calibration_python import *
    # Import other files in the directory
    # from mypyfile import *
    isCompiled = True
else:
    print "Warning: the package incremental_calibration_python is not compiled. Type 'rosmake incremental_calibration_python' if you need this."
    PACKAGE_IS_NOT_COMPILED = True;

