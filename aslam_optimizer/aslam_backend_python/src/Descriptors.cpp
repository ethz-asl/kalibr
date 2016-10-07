#include <numpy_eigen/boost_python_headers.hpp>
#include <numpy_eigen/NumpyEigenConverter.hpp>
#include <aslam/BriskDescriptor.hpp>
#include <aslam/SurfDescriptor.hpp>
using namespace boost::python;
using namespace aslam;

void briskInitFromArray(BriskDescriptor * bd, const Eigen::Matrix<unsigned char, BriskDescriptor::BriskDescriptorBytes, 1> & array)
{
  bd->initFromArray(&array[0]);
}

Eigen::Matrix<unsigned char, BriskDescriptor::BriskDescriptorBytes, 1> briskData(BriskDescriptor * desc)
{
  return desc->descriptor;
}


Eigen::Matrix<float, 64, 1> surfGetDescriptor(SurfDescriptor * sd)
{
  return sd->descriptor;
}

void surfSetDescriptor(SurfDescriptor * sd, Eigen::Matrix<float, 64, 1> & desc)
{
  sd->descriptor = desc;
}

void exportDescriptors()
{

  
    import_array();
    NumpyEigenConverter<Eigen::Matrix<unsigned char, BriskDescriptor::BriskDescriptorBytes, 1> >::register_converter();
    NumpyEigenConverter< Eigen::Matrix<float, 64, 1> >::register_converter();

    class_<BriskDescriptor>("BriskDescriptor", init<>())
      .def("descriptorDistance", &BriskDescriptor::descriptorDistance)
      //virtual float descriptorDistanceDb(const DescriptorBase & other) const;
      // float descriptorDistance(const BriskDescriptor & other) const;
    // virtual void initFromArray(const unsigned char * descriptionVector);
      .def("initFromArray", &briskInitFromArray)
      .def("setDescriptor", &briskInitFromArray)
      // const unsigned char* data() const;
      .def("data", &briskData)
      .def("descriptor", &briskData)
    // Eigen::Matrix<unsigned char, BriskDescriptorBytes, 1> descriptor;
    ;


    class_<SurfDescriptor>("SurfDescriptor", init<>())
      .def("descriptorDistance", &SurfDescriptor::descriptorDistance)
      .def_readwrite("laplacianBit", &SurfDescriptor::laplacianBit)
      .def("descriptor", &surfGetDescriptor)
      .def("setDescriptor", &surfSetDescriptor)
      ;


}
