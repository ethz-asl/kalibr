///
/// @file   boost_serialization_pickle.hpp
/// @author Paul Furgale <paul.furgale@gmail.com>
/// @date   Tue Jul 23 21:22:15 2013
/// 
/// @brief  A class to leverage boost::serialization for Python pickle support
/// 
/// 
///

#ifndef SM_BOOST_SERIALIZATION_PICKLE_HPP
#define SM_BOOST_SERIALIZATION_PICKLE_HPP

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/python.hpp>
#include <sstream>
#include <sm/assert_macros.hpp>


namespace sm {
namespace python {


///
/// \class pickle_suite
///
/// \brief  A class to leverage boost::serialization for Python pickle support
///
/// This class provides pickle serialization support for classes that have:
/// (1) a default constructor that takes no arguments, and
/// (2) boost serialization implemented.
///
/// To include this support in your class, simply add a single line to
/// the list of your classes member functions when exporting the
/// boost::python wrapper:
///
/// class_<MyClass>("MyClass", init<>())
///   //  other .def(...) statements
///   .def_pickle( sm::python::pickle_suite< MyClass >() )
///   ;
///
///
/// Then, in Python, you should be able to pickle the object. See the very
/// simple example here: http://wiki.python.org/moin/UsingPickle
///
template<typename T>
struct pickle_suite : boost::python::pickle_suite
{
  /// \brief This class assumes the object has a default constructor that takes
  ///        no arguments.
  ///
  /// From the documentation:
  /// http://www.boost.org/doc/libs/1_35_0/libs/python/doc/v2/pickle.html
  ///
  /// "When an instance of a Boost.Python
  /// extension class is pickled, the pickler tests if the instance
  /// has a __getinitargs__ method. This method must return a Python
  /// tuple (it is most convenient to use a
  /// boost::python::tuple). When the instance is restored by the
  /// unpickler, the contents of this tuple are used as the
  /// arguments for the class constructor."
  //
  static
  boost::python::tuple
  getinitargs(const T & /* val */)
  {
    return boost::python::tuple();
  }

  static
  boost::python::tuple
  getstate(const T & val)
  {
    // Serialize to a string stream then return the string.
    std::ostringstream out;
    ::boost::archive::text_oarchive oa(out);
    oa << val;
    return boost::python::make_tuple( out.str() );
  }

  static
  void
  setstate(T & val, boost::python::tuple state)
  {
    boost::python::extract<std::string> str(state[0]);
    SM_ASSERT_TRUE(std::runtime_error, str.check(), "Error during de-serialization. Unable to convert the tuple object to a string");
    std::istringstream in(str());
    ::boost::archive::text_iarchive ia(in);
    ia >> val;
  }
};


} // namespace python
} // namespace sm



#endif /* SM_BOOST_SERIALIZATION_PICKLE_HPP */
