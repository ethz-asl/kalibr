#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sm/logging.hpp>
#include <aslam/cameras/GridDetector.hpp>

namespace aslam {
namespace cameras {

//serialization constructor (don't use!)
GridDetector::GridDetector() {
}

GridDetector::GridDetector(boost::shared_ptr<CameraGeometryBase> geometry,
                           GridCalibrationTargetBase::Ptr target,
                           const GridDetector::GridDetectorOptions &options)
    : _geometry(geometry),
      _target(target),
      _options(options) {
  SM_ASSERT_TRUE(Exception, _geometry.get() != NULL,
                 "Unable to initialize with null camera geometry");
  SM_ASSERT_TRUE(Exception, _target.get() != NULL,
                 "Unable to initialize with null calibration target");

  initializeDetector();
}

void GridDetector::initializeDetector()
{
  if (_options.plotCornerReprojection) {
    cv::namedWindow("Corner reprojection");
    cvStartWindowThread();
  }
}

GridDetector::~GridDetector() {

}

void GridDetector::initCameraGeometry(boost::shared_ptr<CameraGeometryBase> geometry) {
  SM_ASSERT_TRUE(Exception, geometry.get() != NULL, "Unable to initialize with null camera geometry");
  _geometry = geometry;
}

bool GridDetector::initCameraGeometryFromObservation(const cv::Mat &image) {
  boost::shared_ptr<std::vector<cv::Mat> > images_ptr = boost::make_shared<std::vector<cv::Mat>>();
  images_ptr->push_back(image);

  return initCameraGeometryFromObservations(images_ptr);
}

bool GridDetector::initCameraGeometryFromObservations(boost::shared_ptr<std::vector<cv::Mat> > images_ptr) {

  std::vector<cv::Mat>& images = *images_ptr;

  SM_DEFINE_EXCEPTION(Exception, std::runtime_error);
  SM_ASSERT_TRUE(Exception, images.size() != 0, "Need min. one image");

  std::vector<GridCalibrationTargetObservation> observations;

  for(unsigned int i=0; i<images.size(); i++)
  {
    GridCalibrationTargetObservation obs(_target);

    //detect calibration target
    bool success = findTargetNoTransformation(images[i], obs);

    //delete image copy (save memory)
    obs.clearImage();

    //append
    if(success)
      observations.push_back(obs);
  }

  //initialize the intrinsics
  if(observations.size() > 0)
    return _geometry->initializeIntrinsics(observations);

  return false;
}

bool GridDetector::findTarget(const cv::Mat & image,
                              GridCalibrationTargetObservation & outObservation) const {
  return findTarget(image, aslam::Time(0, 0), outObservation);
}

bool GridDetector::findTargetNoTransformation(const cv::Mat & image, const aslam::Time & stamp,
    GridCalibrationTargetObservation & outObservation) const {
  bool success = false;

  // Extract the calibration target corner points
  Eigen::MatrixXd cornerPoints;
  std::vector<bool> validCorners;
  success = _target->computeObservation(image, cornerPoints, validCorners);

  // Set the image, target, and timestamp regardless of success.
  outObservation.setTarget(_target);
  outObservation.setImage(image);
  outObservation.setTime(stamp);

  // Set the observed corners in the observation
  for (int i = 0; i < cornerPoints.rows(); i++) {
    if (validCorners[i])
      outObservation.updateImagePoint(i, cornerPoints.row(i).transpose());
  }

  return success;
}

bool GridDetector::findTarget(const cv::Mat & image, const aslam::Time & stamp,
    GridCalibrationTargetObservation & outObservation) const{
  sm::kinematics::Transformation trafo;

  // find calibration target corners
  bool success = findTargetNoTransformation(image, stamp, outObservation);

  // calculate trafo cam-target
  if (success) {
    // also estimate the transformation:
    success = _geometry->estimateTransformation(outObservation, trafo);

    if (success)
      outObservation.set_T_t_c(trafo);
    else
      SM_DEBUG_STREAM("estimateTransformation() failed");
  }

  //remove corners with a reprojection error above a threshold
  //(remove detection outliers)
  if(_options.filterCornerOutliers && success)
  {
    //calculate reprojection errors
    std::vector<cv::Point2f> corners_reproj;
    std::vector<cv::Point2f> corners_detected;
    outObservation.getCornerReprojection(_geometry, corners_reproj);
    unsigned int numCorners = outObservation.getCornersImageFrame(corners_detected);

    //calculate error norm
    Eigen::MatrixXd reprojection_errors_norm = Eigen::MatrixXd::Zero(numCorners,1);

    for(unsigned int i=0; i<numCorners; i++ )
    {
      cv::Point2f reprojection_err = corners_detected[i] - corners_reproj[i];

      reprojection_errors_norm(i,0) = sqrt(reprojection_err.x*reprojection_err.x +
                                           reprojection_err.y*reprojection_err.y);
    }

    //calculate statistics
    double mean = reprojection_errors_norm.mean();
    double std = 0.0;
    for(unsigned int i=0; i<numCorners; i++)
    {
      double temp = reprojection_errors_norm(i,0)-mean;
      std += temp*temp;
    }
    std /= (double)numCorners;
    std = sqrt(std);

    //disable outlier corners
    std::vector<unsigned int> cornerIdx;
    outObservation.getCornersIdx(cornerIdx);

    unsigned int removeCount = 0;
    for(unsigned int i=0; i<corners_detected.size(); i++ )
    {
      if( reprojection_errors_norm(i,0) > mean + _options.filterCornerSigmaThreshold * std &&
          reprojection_errors_norm(i,0) > _options.filterCornerMinReprojError)
      {
        outObservation.removeImagePoint( cornerIdx[i] );
        removeCount++;
        SM_DEBUG_STREAM("removed target point with reprojection error of " << reprojection_errors_norm(i,0) << " (mean: " << mean << ", std: " << std << ")\n";);
      }
    }

    if(removeCount>0)
      SM_DEBUG_STREAM("removed " << removeCount << " of " << numCorners << " calibration target corner outliers\n";);
  }


  // show plot of reprojected corners
  if (_options.plotCornerReprojection) {
    cv::Mat imageCopy1 = image.clone();
    cv::cvtColor(imageCopy1, imageCopy1, CV_GRAY2RGB);

    if (success) {
      //calculate reprojection
      std::vector<cv::Point2f> reprojs;
      outObservation.getCornerReprojection(_geometry, reprojs);

      for (unsigned int i = 0; i < reprojs.size(); i++)
        cv::circle(imageCopy1, reprojs[i], 3, CV_RGB(255,0,0), 1);


    } else {
      cv::putText(imageCopy1, "Detection failed! (frame not used)",
                  cv::Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 0.8,
                  CV_RGB(255,0,0), 3, 8, false);
    }

    cv::imshow("Corner reprojection", imageCopy1);  // OpenCV call
    if (_options.imageStepping) {
      cv::waitKey(0);
    } else {
      cv::waitKey(1);
    }
  }

  return success;
}

/// \brief Find the target but don't estimate the transformation.
bool GridDetector::findTargetNoTransformation(const cv::Mat & image,
                                              GridCalibrationTargetObservation & outObservation) const {
  return findTargetNoTransformation(image, aslam::Time(0, 0), outObservation);
}

}  // namespace cameras
}  // namespace aslam

//export explicit instantions for all included archives
#include <sm/boost/serialization.hpp>
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::GridDetector);
BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::GridDetector::GridDetectorOptions);
