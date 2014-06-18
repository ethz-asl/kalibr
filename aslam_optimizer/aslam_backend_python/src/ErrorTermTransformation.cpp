#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/ErrorTermTransformation.hpp>


void exportErrorTermTransformation()
{
    using namespace boost::python;
    using namespace aslam::backend;

    class_<ErrorTermTransformation,
           boost::shared_ptr<ErrorTermTransformation>,
           bases<ErrorTerm> >("ErrorTermTransformation",
                              init<aslam::backend::TransformationExpression, sm::kinematics::Transformation, double, double>("ErrorTermTransformation(aslam::backend::TransformationExpression T, sm::kinematics::Transformation prior, double weightRotation, double weightTranslation)"))
                              ;
}
