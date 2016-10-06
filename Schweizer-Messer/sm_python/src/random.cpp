#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/random.hpp>


void exportRandom() {
  using namespace boost::python;
  using namespace sm::random;

  /// \brief Return a sample from a normally distributed random distribution with zero mean and standard deviation 1.
  def("normal",&normal);
  /// \brief Return a sample from a normally distributed random distribution with zero mean and standard deviation 1.
  def("randn",&randn);

  /// \brief Return a sample from a uniform distribution between 0.0 and 1.0
  def("uniform",&uniform);

  /// \brief Return a sample from a uniform distribution between 0.0 and 1.0
  def("rand",&sm::random::rand);

  /// \brief Return a sample from a uniform distribution in the half-open range [lowerBound, upperBound)
  def("randLU",&randLU);

  /// \brief Return a sample from a uniform distribution in the half-open range [lowerBound, upperBound)
  def("randLUi",&randLUi);

  /// \brief Seed the random number generator.
  def("seed",&seed);

  
}
