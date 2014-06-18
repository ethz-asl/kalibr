from SplineTests import *
from BSplineTests import *
from BSplinePoseTests import *

if __name__ == '__main__':
    import rostest
    rostest.rosrun('splines', 'test_vs_fitpack', TestSplinesVsFitpack)
    rostest.rosrun('splines', 'bspline_pose', TestBSplinePose)
    #rostest.rosrun('splines', 'bspline', TestBSplines)


