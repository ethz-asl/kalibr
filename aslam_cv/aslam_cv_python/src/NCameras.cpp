#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/NCameraProcessor.hpp>
#include <aslam/NCameraPipeline.hpp>

void exportNCameras() {
  using namespace boost::python;
  using namespace aslam;

  class_<NCameraProcessor, boost::shared_ptr<NCameraProcessor>,
      boost::noncopyable>(
      "NCameraProcessor",
      init<size_t, bool, bool>(
          "NCameraProcessor(size_t nThreads, bool doBackProjection, bool doBackProjectionUncertainty)"))
      .def(init<const sm::PropertyTree &>())
  //     void addCamera(boost::shared_ptr<UndistorterBase> & undistorter,
  //                    boost::shared_ptr<FrameBuilder> & frameBuilder,
  //                    const sm::kinematics::Transformation & T_v_c);
      .def("addCamera", &NCameraProcessor::addCamera)
  // /// \brief process the images and produce a multi-frame
  // boost::shared_ptr<MultiFrame> processImages( boost::shared_ptr<ImageContainer> images );
      .def("processImages", &NCameraProcessor::processImages)
  // boost::shared_ptr<CameraSystemBase> getCameraSystemBase() const;
      .def("getCameraSytem", &NCameraProcessor::getCameraSystem)
  // boost::shared_ptr<NCameraSystem> getCameraSystem() const;
           ;

  class_<NCameraPipeline, boost::shared_ptr<NCameraPipeline>, boost::noncopyable>(
      "NCameraPipeline",
      init<const sm::PropertyTree&>(
          "NCameraPipeline(const sm::PropertyTree & config)")).def(
      "addImage", &NCameraPipeline::addEigenImage).def(
      "getCameraSystem", &NCameraPipeline::getCameraSystem);

}
