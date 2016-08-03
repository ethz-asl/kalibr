#include <utility>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/python/Id.hpp>
#include <sm/python/boost_serialization_pickle.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/targets.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <aslam/LinkCvSerialization.hpp>

typedef Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, Eigen::Dynamic> image_t;

boost::python::tuple pointToGridCoordinates(
    const aslam::cameras::GridCalibrationTargetBase * target, size_t i) {
  std::pair<size_t, size_t> gc = target->pointToGridCoordinates(i);
  return boost::python::make_tuple(gc.first, gc.second);
}

image_t getImage(const aslam::cameras::GridCalibrationTargetObservation * gcto) {
  const cv::Mat & from = gcto->image();
  image_t to(from.rows, from.cols);
  cv2eigen(from, to);

  return to;
}

void setImage(aslam::cameras::GridCalibrationTargetObservation * frame,
              const image_t & from) {
  cv::Mat to;
  eigen2cv(from, to);
  frame->setImage(to);
}


class PythonImageList {
public:
  typedef std::vector<cv::Mat> ImageListType;
  typedef boost::shared_ptr<ImageListType> ImageListTypePtr;

  PythonImageList() :_images(new ImageListType) {};
  ~PythonImageList() {};

  void addImage(const image_t &image)
  {
    cv::Mat image_cv;
    eigen2cv(image, image_cv);
    _images->push_back(image_cv);
  }

  ImageListTypePtr getImages()
  {
    return _images;
  }

private:
  //image lsit
  ImageListTypePtr _images;
};

void addImageToList(PythonImageList *list, const image_t & from)
{
  list->addImage(from);
}

bool initCameraGeometryFromObservation(aslam::cameras::GridDetector * gd,
                                       const image_t & image) {
  cv::Mat image_cv;
  eigen2cv(image, image_cv);
  return gd->initCameraGeometryFromObservation(image_cv);
}

bool initCameraGeometryFromObservations(aslam::cameras::GridDetector * gd,
                                        PythonImageList &image_list) {
  return gd->initCameraGeometryFromObservations( image_list.getImages() );
}

boost::python::tuple findTarget1(aslam::cameras::GridDetector * gd,
                                 const aslam::Time & stamp,
                                 const image_t & image) {
  aslam::cameras::GridCalibrationTargetObservation obs(gd->target());
  cv::Mat to;
  eigen2cv(image, to);
  bool success = gd->findTarget(to, stamp, obs);

  return boost::python::make_tuple(success, obs);

}

boost::python::tuple findTarget2(
    aslam::cameras::GridDetector * gd,
    const Eigen::Matrix<boost::uint8_t, Eigen::Dynamic, Eigen::Dynamic> & image) {
  return findTarget1(gd, aslam::Time(0, 0), image);
}

boost::python::tuple findTargetNoTransformation1(aslam::cameras::GridDetector * gd,
                                 const aslam::Time & stamp,
                                 const image_t & image) {
  aslam::cameras::GridCalibrationTargetObservation obs(gd->target());
  cv::Mat to;
  eigen2cv(image, to);
  bool success = gd->findTargetNoTransformation(to, stamp, obs);

  return boost::python::make_tuple(success, obs);
}

boost::python::tuple findTargetNoTransformation2(aslam::cameras::GridDetector * gd,
                                                 const image_t & image) {
  return findTargetNoTransformation1(gd, aslam::Time(0, 0), image);
}

/// \brief get a point from the target expressed in the target frame
/// \return true if the grid point was seen in this image.
//bool imagePoint(size_t i, Eigen::Vector2d & outPoint) const;
boost::python::tuple imagePoint(
    aslam::cameras::GridCalibrationTargetObservation * frame, size_t i) {
  Eigen::Vector2d p = Eigen::Vector2d::Zero();
  bool success = frame->imagePoint(i, p);
  return boost::python::make_tuple(success, p);
}

/// \brief get a point from the target expressed in the target frame
/// \return true if the grid point was seen in this image.
//bool imageGridPoint(size_t r, size_t c, Eigen::Vector2d & outPoint) const;
boost::python::tuple imageGridPoint(
    aslam::cameras::GridCalibrationTargetObservation * frame, size_t r,
    size_t c) {
  Eigen::Vector2d p = Eigen::Vector2d::Zero();
  bool success = frame->imageGridPoint(r, c, p);
  return boost::python::make_tuple(success, p);
}

/// \brief get all corners in target coordinates (order matches getCornersImageFrame)
Eigen::MatrixXd getCornersTargetFrame(
    aslam::cameras::GridCalibrationTargetObservation * frame) {
  // Get the corners in the target frame
  std::vector<cv::Point3f> targetCorners;
  unsigned int numCorners = frame->getCornersTargetFrame(targetCorners);

  // Convert all target corners to eigen
  Eigen::MatrixXd targetCornersEigen = Eigen::MatrixXd::Zero(numCorners, 3);

  for (unsigned int i = 0; i < numCorners; i++) {
    targetCornersEigen(i, 0) = targetCorners[i].x;
    targetCornersEigen(i, 1) = targetCorners[i].y;
    targetCornersEigen(i, 2) = targetCorners[i].z;
  }

  return targetCornersEigen;
}

/// \brief get all corners in image frame coordinates (order matches getObservedTargetFrame)
Eigen::MatrixXd getCornersImageFrame(
    aslam::cameras::GridCalibrationTargetObservation * frame) {
  // Get the corners in the image frame
  std::vector<cv::Point2f> imageCorners;
  unsigned int numCorners = frame->getCornersImageFrame(imageCorners);

  // Convert all image corners to eigen
  Eigen::MatrixXd imageCornersEigen = Eigen::MatrixXd::Zero(numCorners, 2);

  for (unsigned int i = 0; i < numCorners; i++) {
    imageCornersEigen(i, 0) = imageCorners[i].x;
    imageCornersEigen(i, 1) = imageCorners[i].y;
  }

  return imageCornersEigen;
}

/// \brief get all corners in image frame coordinates (order matches getObservedTargetFrame)
Eigen::MatrixXd getCornerReprojection(aslam::cameras::GridCalibrationTargetObservation * frame, const boost::shared_ptr<aslam::cameras::CameraGeometryBase> cameraGeometry) {
  // Get the corners in the image frame
  std::vector<cv::Point2f> cornersReproj;
  unsigned int numCorners = frame->getCornerReprojection(cameraGeometry, cornersReproj);

  // Convert all image corners to eigen
  Eigen::MatrixXd cornersReprojEigen = Eigen::MatrixXd::Zero(numCorners, 2);

  for (unsigned int i = 0; i < numCorners; i++) {
    cornersReprojEigen(i, 0) = cornersReproj[i].x;
    cornersReprojEigen(i, 1) = cornersReproj[i].y;
  }

  return cornersReprojEigen;
}

/// \brief get the point index of all (observed) corners (order corresponds to the output of getCornersImageFrame and getCornersTargetFrame)
Eigen::VectorXi getCornersIdx(
    aslam::cameras::GridCalibrationTargetObservation * frame) {
  // Get the corners in the image frame
  std::vector<unsigned int> cornersIdx;
  unsigned int numCorners = frame->getCornersIdx(cornersIdx);

  // Convert all image corners to eigen
  Eigen::VectorXi imageCornersEigen = Eigen::VectorXi::Zero(numCorners, 1);

  for (unsigned int i = 0; i < numCorners; i++)
    imageCornersEigen(i) = cornersIdx[i];

  return imageCornersEigen;
}


void exportGridCalibration() {
  using namespace boost::python;
  using namespace aslam::cameras;

  aslam::linkCvSerialization();

  class_<PythonImageList>("NumpyImageList",  init<>())
      .def("addImage", &addImageToList);

  class_<GridDetector::GridDetectorOptions>("GridDetectorOptions", init<>())
    .def_readwrite("plotCornerReprojection", &GridDetector::GridDetectorOptions::plotCornerReprojection)
    .def_readwrite("imageStepping", &GridDetector::GridDetectorOptions::imageStepping)
    .def_readwrite("filterCornerOutliers", &GridDetector::GridDetectorOptions::filterCornerOutliers)
    .def_readwrite("filterCornerSigmaThreshold", &GridDetector::GridDetectorOptions::filterCornerSigmaThreshold)
    .def_readwrite("filterCornerMinReprojError", &GridDetector::GridDetectorOptions::filterCornerMinReprojError)
    .def_pickle(sm::python::pickle_suite<GridDetector::GridDetectorOptions>());

  class_<GridDetector, boost::shared_ptr<GridDetector>, boost::noncopyable>(
      "GridDetector",
      init<boost::shared_ptr<CameraGeometryBase>, GridCalibrationTargetBase::Ptr, GridDetector::GridDetectorOptions>("GridDetector::GridDetector( boost::shared_ptr<CameraGeometryBase> geometry, GridCalibrationTargetBase::Ptr target, GridDetector::GridDetectorOptions options)"))
      .def("initCameraGeometry", &GridDetector::initCameraGeometry)
      .def("initCameraGeometryFromObservation", &initCameraGeometryFromObservation)
      .def("initCameraGeometryFromObservations", &initCameraGeometryFromObservations)
      .def("geometry", &GridDetector::geometry)
      .def("target",&GridDetector::target)
      .def("findTarget", &findTarget1)
      .def("findTarget", &findTarget2)
      .def("findTargetNoTransformation", &findTargetNoTransformation1)
      .def("findTargetNoTransformation", &findTargetNoTransformation2)
      .def(init<boost::shared_ptr<CameraGeometryBase>, GridCalibrationTargetBase::Ptr>("GridDetector::GridDetector( boost::shared_ptr<CameraGeometryBase> geometry, GridCalibrationTargetBase::Ptr target)"))
      .def(init<>("Do not use the default constructor. It is only necessary for the pickle interface"))
      .def_pickle(sm::python::pickle_suite<GridDetector>());

  class_<GridCalibrationTargetBase,
      boost::shared_ptr<GridCalibrationTargetBase>, boost::noncopyable>(
      "GridCalibrationTargetBase",
      init<size_t, size_t>("Do not use the default constructor of base class"))
      .def("size", &GridCalibrationTargetBase::size)
      .def("rows", &GridCalibrationTargetBase::rows)
      .def("cols", &GridCalibrationTargetBase::cols)
      .def("point", &GridCalibrationTargetBase::point)
      .def("points", &GridCalibrationTargetBase::points)
      .def("pointToGridCoordinates", &pointToGridCoordinates)
      .def("gridCoordinatesToPoint", &GridCalibrationTargetBase::gridCoordinatesToPoint)
      .def("gridPoint", &GridCalibrationTargetBase::gridPoint)
      .def(init<>("Do not use the default constructor. It is only necessary for the pickle interface"))
      .def_pickle(sm::python::pickle_suite<GridCalibrationTargetBase>())
      ;

  class_<GridCalibrationTargetCheckerboard::CheckerboardOptions>("CheckerboardOptions", init<>())
    .def_readwrite("useAdaptiveThreshold", &GridCalibrationTargetCheckerboard::CheckerboardOptions::useAdaptiveThreshold)
    .def_readwrite("normalizeImage", &GridCalibrationTargetCheckerboard::CheckerboardOptions::normalizeImage)
    .def_readwrite("performFastCheck", &GridCalibrationTargetCheckerboard::CheckerboardOptions::performFastCheck)
    .def_readwrite("windowWidth", &GridCalibrationTargetCheckerboard::CheckerboardOptions::windowWidth)
    .def_readwrite("filterQuads", &GridCalibrationTargetCheckerboard::CheckerboardOptions::filterQuads)
    .def_readwrite("doSubpixelRefinement", &GridCalibrationTargetCheckerboard::CheckerboardOptions::doSubpixelRefinement)
    .def_readwrite("showExtractionVideo", &GridCalibrationTargetCheckerboard::CheckerboardOptions::showExtractionVideo)
    .def_pickle(sm::python::pickle_suite<GridCalibrationTargetCheckerboard::CheckerboardOptions>());


  class_<GridCalibrationTargetCheckerboard, bases<GridCalibrationTargetBase>,
      boost::shared_ptr<GridCalibrationTargetCheckerboard>, boost::noncopyable>(
      "GridCalibrationTargetCheckerboard", init<size_t, size_t, double, double, GridCalibrationTargetCheckerboard::CheckerboardOptions>(
          "GridCalibrationTargetCheckerboard(size_t rows, size_t cols, double rowSpacingMeters, double colSpacingMeters, CheckerboardOptions options)"))
      .def(init<size_t, size_t, double, double>("GridCalibrationTargetCheckerboard(size_t rows, size_t cols, double rowSpacingMeters, double colSpacingMeters)"))
      .def(init<>("Do not use the default constructor. It is only necessary for the pickle interface"))
      .def_pickle(sm::python::pickle_suite<GridCalibrationTargetCheckerboard>());



  class_<GridCalibrationTargetCirclegrid::CirclegridOptions>("CirclegridOptions", init<>())
    .def_readwrite("useAsymmetricCirclegrid", &GridCalibrationTargetCirclegrid::CirclegridOptions::useAsymmetricCirclegrid)
    .def_readwrite("showExtractionVideo", &GridCalibrationTargetCirclegrid::CirclegridOptions::showExtractionVideo)
    .def_pickle(sm::python::pickle_suite<GridCalibrationTargetCirclegrid::CirclegridOptions>());


  class_<GridCalibrationTargetCirclegrid, bases<GridCalibrationTargetBase>,
      boost::shared_ptr<GridCalibrationTargetCirclegrid>, boost::noncopyable>(
      "GridCalibrationTargetCirclegrid", init<size_t, size_t, double, GridCalibrationTargetCirclegrid::CirclegridOptions>(
          "GridCalibrationTargetCirclegrid(size_t rows, size_t cols, double spacingMeters, CirclegridOptions options)"))
      .def(init<size_t, size_t, double>("GridCalibrationTargetCirclegrid(size_t rows, size_t cols, double spacingMeters)"))
      .def(init<>("Do not use the default constructor. It is only necessary for the pickle interface"))
      .def_pickle(sm::python::pickle_suite<GridCalibrationTargetCirclegrid>());


  GridCalibrationTargetBase::Ptr (GridCalibrationTargetObservation::*target)() = &GridCalibrationTargetObservation::target;

  class_<GridCalibrationTargetObservation,
          boost::shared_ptr<GridCalibrationTargetObservation> >(
          "GridCalibrationTargetObservation",
      init<GridCalibrationTargetBase::Ptr>("GridCalibrationTargetObservation(GridCalibrationTarget::Ptr target)"))
  //// optionally store the image.
  //GridCalibrationTargetObservation(GridCalibrationTarget::Ptr target, cv::Mat image);
  //.def("setPoints", &GridCalibrationTargetObservation::setPoints)

    //old fct: use getCornersTargetFrame / getCornersImageFrame instead (makes the distinction between april and normal tags)
    //  .def("points", &GridCalibrationTargetObservation::points)
    .def("getCornersTargetFrame", &getCornersTargetFrame)
    .def("getCornersImageFrame", &getCornersImageFrame)
    .def("getCornersIdx", &getCornersIdx)
    .def("getCornerReprojection", &getCornerReprojection)
    .def("getImage", &getImage)
    .def("setImage", &setImage)
    .def("clearImage", &GridCalibrationTargetObservation::clearImage)
    .def("hasSuccessfulObservation",  &GridCalibrationTargetObservation::hasSuccessfulObservation)
    .def("imagePoint", &imagePoint)
    .def("imageGridPoint", &imageGridPoint)
    .def("imRows", &GridCalibrationTargetObservation::imRows)
    .def("imCols", &GridCalibrationTargetObservation::imCols)
    .def("updateImagePoint", &GridCalibrationTargetObservation::updateImagePoint)
    .def("removeImagePoint", &GridCalibrationTargetObservation::removeImagePoint)
    .def("target", target)
    .def("T_t_c", &GridCalibrationTargetObservation::T_t_c, return_value_policy<copy_const_reference>())
    .def("set_T_t_c", &GridCalibrationTargetObservation::set_T_t_c)
    .def("time", &GridCalibrationTargetObservation::time)
    .def("setTime", &GridCalibrationTargetObservation::setTime)
    // The default constructor is bad, but it is necessary for the pickle interface
    .def(init<>("Do not use the default constructor. It is only necessary for the pickle interface"))
    .def_pickle(sm::python::pickle_suite<GridCalibrationTargetObservation>())
    ;
}
