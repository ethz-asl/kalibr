#include <sm/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>


namespace sm {
  
namespace random {
    
class Random
{
 public:
  // http://www.boost.org/doc/libs/1_49_0/doc/html/boost_random/reference.html#boost_random.reference.generators
  typedef boost::mt19937 base_generator_type;

  base_generator_type _generator;
  // Define a uniform random number distribution which produces "double"
  // values between 0 and 1 (0 inclusive, 1 exclusive).
  boost::uniform_real<> _uni_dist;
  boost::variate_generator<base_generator_type&, boost::uniform_real<> > _uniform;
  boost::normal_distribution<> _normal_dist;
  boost::variate_generator<base_generator_type&, boost::normal_distribution<> > _normal;

  Random() : _uni_dist(0.0,1.0), _uniform(_generator, _uni_dist), _normal_dist(), _normal(_generator, _normal_dist)
  {
	// 0
  }

  double normal()
  {
	return _normal();
  }

  double uniform()
  {
	return _uniform();
  }

  void seed(boost::uint64_t s)
  {
    _normal.engine().seed(s);
    _normal.distribution().reset();
    _uniform.engine().seed(s);
    _uniform.distribution().reset();
  }

  static Random & instance()
  {
	static Random random;
	return random;
  }
};

double normal()
{
  return Random::instance().normal();
}

double randn()
{
  return Random::instance().normal();
}

double uniform()
{
  return Random::instance().uniform();
}

double rand()
{
  return Random::instance().uniform();
}

double randLU(double lowerBoundInclusive, double upperBoundExclusive)
{
  return Random::instance().uniform() * (upperBoundExclusive - lowerBoundInclusive) + lowerBoundInclusive;
}

int randLUi(int lowerBoundInclusive, int upperBoundExclusive)
{
  return (int)floor(Random::instance().uniform() * (upperBoundExclusive - lowerBoundInclusive)) + lowerBoundInclusive;
}

void seed(boost::uint64_t s)
{
  Random::instance().seed(s);
}

  
} // namespace random

} // namespace sm
