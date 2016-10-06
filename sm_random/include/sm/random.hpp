#ifndef SM_RANDOM_HPP
#define SM_RANDOM_HPP

#include <boost/cstdint.hpp>

namespace sm {
  namespace random {
    

    /// \brief Return a sample from a normally distributed random distribution with zero mean and standard deviation 1.
    double normal();
    /// \brief Return a sample from a normally distributed random distribution with zero mean and standard deviation 1.
    double randn();

    /// \brief Return a sample from a uniform distribution between 0.0 and 1.0
    double uniform();

    /// \brief Return a sample from a uniform distribution between 0.0 and 1.0
    double rand();

    /// \brief Return a sample from a uniform distribution in the half-open range [lowerBound, upperBound)
    double randLU(double lowerBoundInclusive, double upperBoundExclusive);

    /// \brief Return a sample from a uniform distribution in the half-open range [lowerBound, upperBound)
    int randLUi(int lowerBoundInclusive, int upperBoundExclusive);

    /// \brief Seed the random number generator.
    void seed(boost::uint64_t s);
  
  } // namespace random
} // namespace sm


#endif /* SM_RANDOM_HPP */
