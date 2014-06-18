#include <aslam/calculateOverlappingFov.hpp>

namespace aslam {

void calculateOverlappingFOV(const cameras::CameraGeometryBase & camera1,
                             const cameras::CameraGeometryBase & camera2,
                             const sm::kinematics::Transformation& T_cam1_cam2,
                             cameras::ImageMask& outMask1,
                             cameras::ImageMask& outMask2,
                             double& outOverlappingRatio1,
                             double& outOverlappingRatio2,
                             const Eigen::VectorXi& sampleDistances,
                             double scale) {
  // get dimensions
  int width1 = (int) camera1.width() * scale;
  int height1 = (int) camera1.height() * scale;
  int width2 = (int) camera2.width() * scale;
  int height2 = (int) camera2.height() * scale;

  // clear incoming masks
  cv::Mat outMask1CV = outMask1.getMask();
  cv::Mat outMask2CV = outMask2.getMask();
  outMask1CV = cv::Mat::zeros(height1, width1, CV_8UC1);
  outMask2CV = cv::Mat::zeros(height2, width2, CV_8UC1);
  outMask1.setScale(scale);
  outMask2.setScale(scale);

  Eigen::VectorXd point;

  // build outMask1
  outOverlappingRatio1 = 0;
  for (int x = 0; x < width1; x++) {
    for (int y = 0; y < height1; y++) {
      // if visible
      // TODO: BB: what about rounding mistakes?
      Eigen::VectorXd kp(2);
      kp << (double) x / scale, (double) y / scale;
      if (camera1.vsKeypointToHomogeneous(kp, point)) {
        double norm = point.norm();
        int i = 0;
        // check points on the beam until one is found or no other points to check
        while (i < sampleDistances.size()
            && outMask1CV.at<unsigned char>(y, x) == 0) {
          // Send point to distance sampleDistances(i) on the beam
          Eigen::Vector4d tempPoint = point;
          tempPoint(3) = norm / (norm + sampleDistances(i));

          // Project point to other camera frame
          tempPoint = T_cam1_cam2.inverse() * tempPoint;

          Eigen::VectorXd keypointLocation;
          if (camera2.vsHomogeneousToKeypoint(tempPoint, keypointLocation)) {
            outMask1CV.at<unsigned char>(y, x) = 255;
            outOverlappingRatio1++;
          }  // if
          i++;
        }  // while
      }  // if
    }  // for
  }  // for
  outOverlappingRatio1 = outOverlappingRatio1 / (width1 * height1);

  // build outMask2
  outOverlappingRatio2 = 0;
  for (int x = 0; x < width2; x++) {
    for (int y = 0; y < height2; y++) {
      Eigen::VectorXd kp(2);
      kp << (double) x / scale, (double) y / scale;

      // if visible
      // TODO: BB: what about rounding mistakes?
      if (camera2.vsKeypointToHomogeneous(kp, point)) {

        double norm = point.norm();
        int i = 0;
        // check points on the beam until one is found or no other points to check
        while (i < sampleDistances.size()
            && outMask2CV.at<unsigned char>(y, x) == 0) {
          // Send point to distance sampleDistances(i) on the beam
          Eigen::Vector4d tempPoint = point;
          tempPoint(3) = norm / (norm + sampleDistances(i));

          // Project point to other camera frame
          tempPoint = T_cam1_cam2 * tempPoint;

          Eigen::VectorXd keypointLocation;
          if (camera1.vsHomogeneousToKeypoint(tempPoint, keypointLocation)) {
            outMask2CV.at<unsigned char>(y, x) = 255;
            outOverlappingRatio2++;
          }  // if
          i++;
        }  // while
      }  // if
    }  // for
  }  // for
  outOverlappingRatio2 = outOverlappingRatio2 / (width2 * height2);
}

}  // namespace aslam
