#include <algorithm>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/make_shared.hpp>
#include <sm/assert_macros.hpp>
#include <sm/logging.hpp>
#include <aslam/cameras/GridCalibrationTargetAssymetricAprilgrid.hpp>

namespace aslam {
namespace cameras {

/// \brief Construct an Aprilgrid calibration target
///        tagRows:    number of tags in y-dir (gridRows = 2*tagRows)
///        tagCols:    number of tags in x-dir (gridCols = 2*tagCols)
///        tagSize:    size of a tag [m]
///        tagSpacing: space between tags (in tagSpacing [m] = tagSpacing*tagSize)
///
///        corner ordering in _points :
///          12-----13  14-----15
///          | TAG 3 |  | TAG 4 |
///          8-------9  10-----11
///          4-------5  6-------7
///    y     | TAG 1 |  | TAG 2 |
///   ^      0-------1  2-------3
///   |-->x
GridCalibrationTargetAssymetricAprilgrid::GridCalibrationTargetAssymetricAprilgrid(const std::vector<TargetPoint> &targetPoints,
    const GridCalibrationTargetAprilgrid::AprilgridOptions &options)
    : GridCalibrationTargetBase(),
    _targetPoints(targetPoints),
    _options(options),
    _tagCodes(TAGCODES)
      {
     
    //Assign cols and rows of the grid based on position
     targetPoints2Grid();
    
        
    _points.setConstant(size(), 3,-1);
    
    //initialize a normal grid (checkerboard and circlegrids)
    createGridPoints();

    //start the output window if requested
    initialize();


}
GridCalibrationTargetAssymetricAprilgrid::GridCalibrationTargetAssymetricAprilgrid(const boost::python::list& vector_list,const GridCalibrationTargetAprilgrid::AprilgridOptions &options):
  GridCalibrationTargetBase(),
  _options(options),
  _tagCodes(TAGCODES){
    
    std::vector<TargetPoint> targetPoints;

    for (int i = 0; i < boost::python::len(vector_list); ++i)
    {
        targetPoints.push_back((boost::python::extract<TargetPoint>(vector_list[i])));
    }

    _targetPoints =targetPoints;
    
    targetPoints2Grid();

    _points.setConstant(size(), 3,-1);
    
    //initialize a normal grid (checkerboard and circlegrids)
    createGridPoints();

    //start the output window if requested
    initialize();

  }
GridCalibrationTargetAssymetricAprilgrid::GridCalibrationTargetAssymetricAprilgrid():
  _tagCodes(TAGCODES)
  {}



  void GridCalibrationTargetAssymetricAprilgrid::targetPoints2Grid(){

    std::vector<float> rows;
    std::vector<float> cols;
    std::vector<float>::iterator it_rows;
    std::vector<float>::iterator it_cols;

    TargetPoint *tag;

    for (size_t i=0; i< _targetPoints.size(); i++){
      tag = &_targetPoints.at(i);
      tag->id = i; //already ordered by id in familycodes
      it_rows = find (rows.begin(), rows.end(), tag->y);  
      it_cols = find (cols.begin(), cols.end(), tag->x);      

      if (it_rows==rows.end())
        rows.push_back(tag->y);
      if(it_cols ==cols.end())
        cols.push_back(tag->x);

        //ou row col based on size
      it_rows = find (rows.begin(), rows.end(), tag->y + tag->size);  
      it_cols = find (cols.begin(), cols.end(), tag->x + tag->size);           

      if (it_rows==rows.end())
        rows.push_back(tag->y + tag->size);
      if(it_cols ==cols.end())
        cols.push_back(tag->x + tag->size);
    }

    std::sort(rows.begin(),rows.end());
    std::sort(cols.begin(),cols.end());

    for (size_t i=0; i< _targetPoints.size(); i++){
      tag = &_targetPoints.at(i);

      it_rows = find (rows.begin(), rows.end(), tag->y);  
      it_cols = find (cols.begin(), cols.end(), tag->x);

      tag->row[0] = std::distance (rows.begin(),it_rows);
      tag->col[0] = std::distance (cols.begin(),it_cols);

      it_rows = find (rows.begin(), rows.end(), tag->y + tag->size);  
      it_cols = find (cols.begin(), cols.end(), tag->x + tag->size);

      tag->row[1] =std::distance (rows.begin(),it_rows);
      tag->col[1] =std::distance (cols.begin(),it_cols);

    }
    
    _rows=rows.size(); 
    _cols=cols.size();
  }
void GridCalibrationTargetAssymetricAprilgrid::initialize()
{
  if (_options.showExtractionVideo) {
    cv::namedWindow("Aprilgrid: Tag detection");
    cv::namedWindow("Aprilgrid: Tag corners");
    cvStartWindowThread();
  }

  //create the tag detector
  _tagDetector = boost::make_shared<AprilTags::TagDetector>(_tagCodes, _options.blackTagBorder);
}

/// \brief initialize an april grid
///   point ordering: (e.g. 2x2 grid)
///          12-----13  14-----15
///          | TAG 3 |  | TAG 4 |
///          8-------9  10-----11
///          4-------5  6-------7   2     3
///    y     | TAG 1 |  | TAG 2 |
///   ^      0-------1  2-------3   0-----1
///   |-->x
void GridCalibrationTargetAssymetricAprilgrid::createGridPoints() {
   
  int index;

    for (unsigned i = 0; i < _targetPoints.size(); i++) {
        TargetPoint *tag = &_targetPoints.at(i);
        
        for (unsigned k1=0; k1<2; k1++)
          for (unsigned k2=0; k2<2; k2 ++){
            index = tag->row[k1]*_cols + tag->col[k2];
            Eigen::Matrix<double, 1, 3> point;
            point(0) = tag->x*0.001 + k2*tag->size*0.001; //mm -> m
            point(1) = tag->y*0.001 + k1*tag->size*0.001;
            point(2) = 0.0;
          
            _points.row(index) = point;
      }
    }
          

}

/// \brief extract the calibration target points from an image and write to an observation
 bool GridCalibrationTargetAssymetricAprilgrid::computeObservation(
    const cv::Mat & image, Eigen::MatrixXd & outImagePoints,
    std::vector<bool> &outCornerObserved ) const  {

  bool success = true;
    
  // detect the tags
  std::vector<AprilTags::TagDetection> detections = _tagDetector->extractTags(image);

 /* handle the case in which a tag is identified but not all tag
   * corners are in the image (all data bits in image but border
   * outside). tagCorners should still be okay as apriltag-lib
   * extrapolates them, only the subpix refinement will fail
   */

  //min. distance [px] of tag corners from image border (tag is not used if violated)
  std::vector<AprilTags::TagDetection>::iterator iter = detections.begin();

  for (iter = detections.begin(); iter != detections.end();) {
    // check all four corners for violation
    bool remove = false;

    for (int j = 0; j < 4; j++) {
      remove |= iter->p[j].first < _options.minBorderDistance;
      remove |= iter->p[j].first > (float) (image.cols) - _options.minBorderDistance;  //width
      remove |= iter->p[j].second < _options.minBorderDistance;
      remove |= iter->p[j].second > (float) (image.rows) - _options.minBorderDistance;  //height
    }

    //also remove tags that are flagged as bad
    if (iter->good != 1)
      remove |= true;

    //also remove if the tag ID is out-of-range for this grid (faulty detection)
    if (iter->id >= (int) size() / 4)
      remove |= true;

    // delete flagged tags
    if (remove) {
      SM_DEBUG_STREAM("Tag with ID " << iter->id << " is only partially in image (corners outside) and will be removed from the TargetObservation.\n");

      // delete the tag and advance in list
      iter = detections.erase(iter);
    } else {
      //advance in list
      ++iter;
    }
  }

  
  if (detections.size() < _options.minTagsForValidObs) {
    success = false;

    //immediate exit if we dont need to show video for debugging...
    //if video is shown, exit after drawing video...
    if (!_options.showExtractionVideo)
      return success;
  }

  //sort detections by tagId
  std::sort(detections.begin(), detections.end(),
            AprilTags::TagDetection::sortByIdCompare);

  // check for duplicate tagIds (--> if found: wild Apriltags in image not belonging to calibration target)
  // (only if we have more than 1 tag...)
  if (detections.size() > 1) {
    for (unsigned i = 0; i < detections.size() - 1; i++)
      if (detections[i].id == detections[i + 1].id) {
        //show the duplicate tags in the image
        cv::destroyAllWindows();
        cv::namedWindow("Wild Apriltag detected. Hide them!");
        cvStartWindowThread();

        cv::Mat imageCopy = image.clone();
        cv::cvtColor(imageCopy, imageCopy, CV_GRAY2RGB);

        //mark all duplicate tags in image
        for (int j = 0; i < detections.size() - 1; i++) {
          if (detections[j].id == detections[j + 1].id) {
            detections[j].draw(imageCopy);
            detections[j + 1].draw(imageCopy);      
          }
        }

        cv::putText(imageCopy, "Duplicate Apriltags detected. Hide them.",
                    cv::Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 0.8,
                    CV_RGB(255,0,0), 2, 8, false);
        cv::putText(imageCopy, "Press enter to exit...", cv::Point(50, 80),
                    CV_FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255,0,0), 2, 8, false);
        cv::imshow("Duplicate Apriltags detected. Hide them", imageCopy);  // OpenCV call

        // and exit
        SM_FATAL_STREAM("\n[ERROR]: Found apriltag not belonging to calibration board. Check the image for the tag and hide it.\n");
        
        cv::waitKey();
        // close the window
        cv::destroyAllWindows();

      }
  }

  // convert corners to cv::Mat (4 consecutive corners form one tag)
  /// point ordering here
  ///          11-----10  15-----14
  ///          | TAG 2 |  | TAG 3 |
  ///          8-------9  12-----13
  ///          3-------2  7-------6
  ///    y     | TAG 0 |  | TAG 1 |
  ///   ^      0-------1  4-------5
  ///   |-->x
  cv::Mat tagCorners(4 * detections.size(), 2, CV_32F);

  for (unsigned i = 0; i < detections.size(); i++) {
    for (unsigned j = 0; j < 4; j++) {
      tagCorners.at<float>(4 * i + j, 0) = detections[i].p[j].first;
      tagCorners.at<float>(4 * i + j, 1) = detections[i].p[j].second;
    }
  }

  //store a copy of the corner list before subpix refinement
  cv::Mat tagCornersRaw = tagCorners.clone();

  //optional subpixel refinement on all tag corners (four corners each tag)
  if (_options.doSubpixRefinement && success)
    cv::cornerSubPix(
        image, tagCorners, cv::Size(2, 2), cv::Size(-1, -1),
        cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

  if (_options.showExtractionVideo) {
    //image with refined (blue) and raw corners (red)
    cv::Mat imageCopy1 = image.clone();
    cv::cvtColor(imageCopy1, imageCopy1, CV_GRAY2RGB);
    for (unsigned i = 0; i < detections.size(); i++)
      for (unsigned j = 0; j < 4; j++) {
        //raw apriltag corners
        //cv::circle(imageCopy1, cv::Point2f(detections[i].p[j].first, detections[i].p[j].second), 2, CV_RGB(255,0,0), 1);

        //subpixel refined corners
        cv::circle(
            imageCopy1,
            cv::Point2f(tagCorners.at<float>(4 * i + j, 0),
                        tagCorners.at<float>(4 * i + j, 1)),
            3, CV_RGB(0,0,255), 1);

        if (!success)
          cv::putText(imageCopy1, "Detection failed! (frame not used)",
                      cv::Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 0.8,
                      CV_RGB(255,0,0), 3, 8, false);
      }

    cv::imshow("Aprilgrid: Tag corners", imageCopy1);  // OpenCV call
    cv::waitKey(1);

    /* copy image for modification */
    cv::Mat imageCopy2 = image.clone();
    cv::cvtColor(imageCopy2, imageCopy2, CV_GRAY2RGB);
    /* highlight detected tags in image */
    for (unsigned i = 0; i < detections.size(); i++) {
      detections[i].draw(imageCopy2);

      if (!success)
        cv::putText(imageCopy2, "Detection failed! (frame not used)",
                    cv::Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 0.8,
                    CV_RGB(255,0,0), 3, 8, false);
    }

    cv::imshow("Aprilgrid: Tag detection", imageCopy2);  // OpenCV call
    cv::waitKey(1);

    //if success is false exit here (delayed exit if _options.showExtractionVideo=true for debugging)
    if (!success)
      return success;
  }

  //insert the observed points into the correct location of the grid point array
  /// point ordering
  ///          12-----13  14-----15
  ///          | TAG 2 |  | TAG 3 |
  ///          8-------9  10-----11
  ///          4-------5  6-------7
  ///    y     | TAG 0 |  | TAG 1 |
  ///   ^      0-------1  2-------3
  ///   |-->x

  outCornerObserved.resize(size(), false);
  outImagePoints.resize(size(), 2);
  //int tagId_pos[11]={0,8,14,2,16,10,4,18,6,12,20};
  std::vector<TargetPoint>::const_iterator it;
  for (unsigned int i = 0; i < detections.size(); i++) {
    it = _targetPoints.begin();
    

    // get the tag id
    unsigned int tagId = detections[i].id;

    // calculate the grid idx for all four tag corners given the tagId and cols
   
        //unsigned pos= tagId_pos[tagId];
        //unsigned int baseId = (int) (pos / (_cols / 2)) * _cols * 2
        //  + (pos % (_cols / 2)) * 2;
      it = std::find_if(_targetPoints.begin(), _targetPoints.end(), 
        boost::bind(&TargetPoint::id, _1) == tagId);
      unsigned int pIdx[] = {0,0,0,0};

      if(it != _targetPoints.end()){
        pIdx[0] =it->row[0]*_cols + it->col[0];
        pIdx[1]= it->row[0]*_cols + it->col[1];
        pIdx[2] = it->row[1]*_cols + it->col[1];
        pIdx[3] =it->row[1]*_cols + it->col[0]  ;
      }

     

   // unsigned int pIdx[] = { baseId, baseId + 1, baseId + (unsigned int) _cols
     //   + 1, baseId + (unsigned int) _cols };

    // add four points per tag
    for (int j = 0; j < 4; j++) {
      //refined corners
      double corner_x = tagCorners.row(4 * i + j).at<float>(0);
      double corner_y = tagCorners.row(4 * i + j).at<float>(1);

      //raw corners
      double cornerRaw_x = tagCornersRaw.row(4 * i + j).at<float>(0);
      double cornerRaw_y = tagCornersRaw.row(4 * i + j).at<float>(1);


      //only add point if the displacement in the subpixel refinement is below a given threshold
      double subpix_displacement_squarred = (corner_x - cornerRaw_x)
          * (corner_x - cornerRaw_x)
          + (corner_y - cornerRaw_y) * (corner_y - cornerRaw_y);

      //add all points, but only set active if the point has not moved to far in the subpix refinement
      outImagePoints.row(pIdx[j]) = Eigen::Matrix<double, 1, 2>(corner_x,
                                                                corner_y);
      
      if (subpix_displacement_squarred <= _options.maxSubpixDisplacement2) {
        outCornerObserved[pIdx[j]] = true;
      } else {
        SM_DEBUG_STREAM("Subpix refinement failed for point: " << pIdx[j] << " with displacement: " << sqrt(subpix_displacement_squarred) << "(point removed) \n");
        outCornerObserved[pIdx[j]] = false;
        for (int w = 0; w < 4; w++) {
          outCornerObserved[pIdx[w]]=false;
        }
        break;
      }


    }
  }

  //succesful observation
  return success;
}

}  // namespace cameras
}  // namespace aslam

//export explicit instantions for all included archives
#include <sm/boost/serialization.hpp>
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::GridCalibrationTargetAssymetricAprilgrid);
BOOST_CLASS_EXPORT_IMPLEMENT(aslam::cameras::GridCalibrationTargetAssymetricAprilgrid::TargetPoint);
