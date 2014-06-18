//-*-c++-*-

#include <iostream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include "apriltags/Homography33.h"

Homography33::Homography33(const std::pair<float,float> &opticalCenter) : cxy(opticalCenter), fA(), H(), valid(false) {
  fA.setZero();
  H.setZero();
}

Eigen::Matrix3d& Homography33::getH() {
  compute();
  return H;
}

#ifdef STABLE_H
void Homography33::setCorrespondences(const std::vector< std::pair<float,float> > &sPts,
                                      const std::vector< std::pair<float,float> > &dPts) {
  valid = false;
  srcPts = sPts;
  dstPts = dPts;
}
#else
void Homography33::addCorrespondence(float worldx, float worldy, float imagex, float imagey) {
  valid = false;
  imagex -= cxy.first;
  imagey -= cxy.second;

  /* Here are the rows of matrix A.  We will compute A'*A
   * A[3*i+0][3] = -worldxyh[i][0]*imagexy[i][2];
   * A[3*i+0][4] = -worldxyh[i][1]*imagexy[i][2];
   * A[3*i+0][5] = -worldxyh[i][2]*imagexy[i][2];
   * A[3*i+0][6] =  worldxyh[i][0]*imagexy[i][1];
   * A[3*i+0][7] =  worldxyh[i][1]*imagexy[i][1];
   * A[3*i+0][8] =  worldxyh[i][2]*imagexy[i][1];
   *       
   * A[3*i+1][0] =  worldxyh[i][0]*imagexy[i][2];
   * A[3*i+1][1] =  worldxyh[i][1]*imagexy[i][2];
   * A[3*i+1][2] =  worldxyh[i][2]*imagexy[i][2];
   * A[3*i+1][6] = -worldxyh[i][0]*imagexy[i][0];
   * A[3*i+1][7] = -worldxyh[i][1]*imagexy[i][0];
   * A[3*i+1][8] = -worldxyh[i][2]*imagexy[i][0];
   *
   * A[3*i+2][0] = -worldxyh[i][0]*imagexy[i][1];
   * A[3*i+2][1] = -worldxyh[i][1]*imagexy[i][1];
   * A[3*i+2][2] = -worldxyh[i][2]*imagexy[i][1];
   * A[3*i+2][3] =  worldxyh[i][0]*imagexy[i][0];
   * A[3*i+2][4] =  worldxyh[i][1]*imagexy[i][0];
   * A[3*i+2][5] =  worldxyh[i][2]*imagexy[i][0];
   */

  // only update upper-right. A'A is symmetric, we'll finish the lower left later.
  float a03 = -worldx;
  float a04 = -worldy;
  float a05 = -1;
  float a06 = worldx*imagey;
  float a07 = worldy*imagey;
  float a08 = imagey;

  fA(3, 3) += a03*a03;
  fA(3, 4) += a03*a04;
  fA(3, 5) += a03*a05;
  fA(3, 6) += a03*a06;
  fA(3, 7) += a03*a07;
  fA(3, 8) += a03*a08;

  fA(4, 4) += a04*a04;
  fA(4, 5) += a04*a05;
  fA(4, 6) += a04*a06;
  fA(4, 7) += a04*a07;
  fA(4, 8) += a04*a08;

  fA(5, 5) += a05*a05;
  fA(5, 6) += a05*a06;
  fA(5, 7) += a05*a07;
  fA(5, 8) += a05*a08;

  fA(6, 6) += a06*a06;
  fA(6, 7) += a06*a07;
  fA(6, 8) += a06*a08;

  fA(7, 7) += a07*a07;
  fA(7, 8) += a07*a08;

  fA(8, 8) += a08*a08;

  float a10 = worldx;
  float a11 = worldy;
  float a12 = 1;
  float a16 = -worldx*imagex;
  float a17 = -worldy*imagex;
  float a18 = -imagex;

  fA(0, 0) += a10*a10;
  fA(0, 1) += a10*a11;
  fA(0, 2) += a10*a12;
  fA(0, 6) += a10*a16;
  fA(0, 7) += a10*a17;
  fA(0, 8) += a10*a18;

  fA(1, 1) += a11*a11;
  fA(1, 2) += a11*a12;
  fA(1, 6) += a11*a16;
  fA(1, 7) += a11*a17;
  fA(1, 8) += a11*a18;

  fA(2, 2) += a12*a12;
  fA(2, 6) += a12*a16;
  fA(2, 7) += a12*a17;
  fA(2, 8) += a12*a18;

  fA(6, 6) += a16*a16;
  fA(6, 7) += a16*a17;
  fA(6, 8) += a16*a18;

  fA(7, 7) += a17*a17;
  fA(7, 8) += a17*a18;

  fA(8, 8) += a18*a18;

  float a20 = -worldx*imagey;
  float a21 = -worldy*imagey;
  float a22 = -imagey;
  float a23 = worldx*imagex;
  float a24 = worldy*imagex;
  float a25 = imagex;

  fA(0, 0) += a20*a20;
  fA(0, 1) += a20*a21;
  fA(0, 2) += a20*a22;
  fA(0, 3) += a20*a23;
  fA(0, 4) += a20*a24;
  fA(0, 5) += a20*a25;

  fA(1, 1) += a21*a21;
  fA(1, 2) += a21*a22;
  fA(1, 3) += a21*a23;
  fA(1, 4) += a21*a24;
  fA(1, 5) += a21*a25;

  fA(2, 2) += a22*a22;
  fA(2, 3) += a22*a23;
  fA(2, 4) += a22*a24;
  fA(2, 5) += a22*a25;

  fA(3, 3) += a23*a23;
  fA(3, 4) += a23*a24;
  fA(3, 5) += a23*a25;

  fA(4, 4) += a24*a24;
  fA(4, 5) += a24*a25;

  fA(5, 5) += a25*a25;
}
#endif

#ifdef STABLE_H
void Homography33::compute() {
  if ( valid ) return;

  std::vector<cv::Point2f> sPts;
  std::vector<cv::Point2f> dPts;
  for (int i=0; i<4; i++) {
    sPts.push_back(cv::Point2f(srcPts[i].first, srcPts[i].second));
  }
  for (int i=0; i<4; i++) {
    dPts.push_back(cv::Point2f(dstPts[i].first - cxy.first, dstPts[i].second - cxy.second));
  }
  cv::Mat homography = cv::findHomography(sPts, dPts);
  for (int c=0; c<3; c++) {
    for (int r=0; r<3; r++) {
      H(r,c) = homography.at<double>(r,c);
    }
  }

  valid = true;
}
#else
void Homography33::compute() {
  if ( valid ) return;

  // make symmetric
  for (int i = 0; i < 9; i++)
    for (int j = i+1; j < 9; j++)
      fA(j,i) = fA(i,j);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(fA, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd eigV = svd.matrixV();

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      H(i,j) = eigV(i*3+j, eigV.cols()-1);
    }
  }

  valid = true;
}
#endif

std::pair<float,float> Homography33::project(float worldx, float worldy) {
  compute();

  std::pair<float,float> ixy;
  ixy.first = H(0,0)*worldx + H(0,1)*worldy + H(0,2);
  ixy.second = H(1,0)*worldx + H(1,1)*worldy + H(1,2);
  float z = H(2,0)*worldx + H(2,1)*worldy + H(2,2);
  ixy.first = ixy.first/z + cxy.first;
  ixy.second = ixy.second/z + cxy.second;
  return ixy;
}

