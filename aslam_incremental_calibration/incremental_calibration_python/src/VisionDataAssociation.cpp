// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/calibration/vision/VisionDataAssociation.hpp>

void exportVisionDataAssociation()
{
    using namespace boost::python;
    using namespace aslam::calibration;
    typedef VisionDataAssociation::camera_t camera_t;

    class_<VisionDataAssociation, 
           boost::shared_ptr<VisionDataAssociation> 
           >("VisionDataAssociation", 
             init<const sm::kinematics::Transformation &,
             boost::shared_ptr<camera_t>,
             const sm::kinematics::Transformation &,
             boost::shared_ptr<camera_t> ,
             double,
             double,
             double,
             int
             >("VisionDataAssociation(const sm::kinematics::Transformation & T_v_cl, boost::shared_ptr<camera_t> leftCamera, const sm::kinematics::Transformation & T_v_cr,boost::shared_ptr<camera_t> rightCamera, double descriptorDistanceThreshold, double disparityTrackingThreshold, double disparityKeyframeThreshold, int numTracksThreshold)"))
           .def(init<const sm::PropertyTree &>())
           ;
    
}
