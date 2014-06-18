#ifndef ASLAM_PYTHON_EXPORT_OMNI_UNDISTORTER_HPP
#define ASLAM_PYTHON_EXPORT_OMNI_UNDISTORTER_HPP

#include <aslam/OmniUndistorter.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

namespace aslam {
namespace cameras {

typedef Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, Eigen::Dynamic> image_t;

template<typename MASK_T>
image_t undistortImageNumpyCvmat(const OmniUndistorter<MASK_T> *instance, const image_t & image_eigen)
{
  cv::Mat image_cv, image_cv_undist;
  eigen2cv(image_eigen, image_cv);

  instance->undistortImage(image_cv, image_cv_undist);

  image_t image_eigen_undist(image_cv_undist.rows, image_cv_undist.cols);
  cv2eigen(image_cv_undist, image_eigen_undist);

  return image_eigen_undist;
}

template<typename MASK_T>
image_t undistortImageToPinholeNumpyCvmat(const OmniUndistorter<MASK_T> *instance, const image_t & image_eigen)
{
  cv::Mat image_cv, image_cv_undist;
  eigen2cv(image_eigen, image_cv);

  instance->undistortImageToPinhole(image_cv, image_cv_undist);

  image_t image_eigen_undist(image_cv_undist.rows, image_cv_undist.cols);
  cv2eigen(image_cv_undist, image_eigen_undist);

  return image_eigen_undist;
}

template<typename MASK_T>
void exportOmniUndistorter(const std::string & name) {
  using namespace boost::python;
  typedef MASK_T mask_t;
  typedef typename OmniUndistorter<mask_t>::distorted_geometry_t distorted_geometry_t;

  class_<OmniUndistorter<mask_t>, boost::shared_ptr<OmniUndistorter<mask_t> > >(
      name.c_str(), init<>()).def(
      init<const sm::PropertyTree &, const sm::PropertyTree &>(
          (name + "(undistorterConfig, cameraConfig)").c_str())).def(
      init<const sm::PropertyTree &, int, double, double>(
          (name + "(cameraConfig, interpolation, alpha, scale)").c_str())).def(
      init<boost::shared_ptr<distorted_geometry_t>, int, double, double>(
          (name + "(distortedGeometry, interpolation, alpha, scale)").c_str()))
      .def("init", &OmniUndistorter<mask_t>::init)
      .def("constructUndistortedFrame", &OmniUndistorter<mask_t>::constructUndistortedFrame)
      .def("undistortImage", &undistortImageNumpyCvmat<mask_t>)
      .def("undistortImage", &OmniUndistorter<mask_t>::undistortImage)
      .def("getIdealGeometry", &OmniUndistorter<mask_t>::idealGeometry)
      .def("getIdealPinholeGeometry", &OmniUndistorter<mask_t>::idealPinholeGeometry)
      .def("undistortImageToPinhole", &OmniUndistorter<mask_t>::undistortImageToPinhole)
      .def("undistortImageToPinhole", &undistortImageToPinholeNumpyCvmat<mask_t>);

}

}
}

#endif /* ASLAM_PYTHON_EXPORT_OMNI_UNDISTORTER_HPP */
