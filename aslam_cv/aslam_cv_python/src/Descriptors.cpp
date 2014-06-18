#include <numpy_eigen/boost_python_headers.hpp>
#include <numpy_eigen/NumpyEigenConverter.hpp>
#include <aslam/BriskDescriptor.hpp>
#include <aslam/SurfDescriptor.hpp>
using namespace boost::python;
using namespace aslam;

void briskInitFromArray(
    BriskDescriptor * bd,
    const Eigen::Matrix<unsigned char, BriskDescriptor::BriskDescriptorBytes, 1> & array) {
  bd->initFromArray(&array[0]);
}

Eigen::Matrix<unsigned char, BriskDescriptor::BriskDescriptorBytes, 1> briskData(
    BriskDescriptor * desc) {
  return desc->descriptor;
}

Eigen::Matrix<float, 64, 1> surfGetDescriptor(SurfDescriptor * sd) {
  return sd->descriptor;
}

void surfSetDescriptor(SurfDescriptor * sd,
                       Eigen::Matrix<float, 64, 1> & desc) {
  sd->descriptor = desc;
}

void exportDescriptors() {

  import_array();
  NumpyEigenConverter
      < Eigen::Matrix<unsigned char, BriskDescriptor::BriskDescriptorBytes, 1>
      > ::register_converter();
  NumpyEigenConverter < Eigen::Matrix<float, 64, 1> > ::register_converter();

  class_<DescriptorBase,  // The class being exported
      boost::shared_ptr<DescriptorBase>,  // Tell boost::python that we may use shared pointers
      boost::noncopyable  // This class is pure virtual. Tell boost::python that we will never pass it by value.
  >("DescriptorBase", no_init  // This class is pure virtual. Disable the constructor
    ).def("distance", &DescriptorBase::distance).def(
      "setRandom", &DescriptorBase::setRandom);

  class_<BriskDescriptor, boost::shared_ptr<BriskDescriptor>,
      bases<DescriptorBase>  // Tell boost::python that this is a child class of DescriptorBase
  >("BriskDescriptor", init<>()).def("distance", &BriskDescriptor::distance).def(
      "initFromArray", &briskInitFromArray).def("setDescriptor",
                                                &briskInitFromArray).def(
      "data", &briskData).def("descriptor", &briskData);

  class_<SurfDescriptor, boost::shared_ptr<SurfDescriptor>,
      bases<DescriptorBase> >("SurfDescriptor", init<>()).def(
      "distance", &SurfDescriptor::distance).def_readwrite(
      "laplacianBit", &SurfDescriptor::laplacianBit).def_readwrite(
      "size", &SurfDescriptor::size).def_readwrite("octave",
                                                   &SurfDescriptor::octave)
      .def_readwrite("angle", &SurfDescriptor::angle).def_readwrite(
      "response", &SurfDescriptor::response).def("descriptor",
                                                 &surfGetDescriptor).def(
      "setDescriptor", &surfSetDescriptor);

}
