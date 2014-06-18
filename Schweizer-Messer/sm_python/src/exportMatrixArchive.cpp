#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/MatrixArchive.hpp>


// std::string getString(std::string const & stringName) const;
std::string getString(const sm::MatrixArchive * ma, std::string const & stringName)
{
  return ma->getString(stringName);
}

// void getMatrix(std::string const & matrixName) const;
Eigen::MatrixXd getMatrix(const sm::MatrixArchive * ma, std::string const & matrixName)
{
  Eigen::MatrixXd M;
  ma->getMatrix(matrixName,M);
  return M;
}

// void getVector(std::string const & vectorName, Eigen::VectorXd & outVector) const;
Eigen::VectorXd getVector(const sm::MatrixArchive * ma, const std::string & vectorName)
{
  Eigen::VectorXd V;
  ma->getVector(vectorName,V);
  return V;
}
// void getScalar(std::string const & scalarName, double & outScalar) const
double getScalar(const sm::MatrixArchive * ma, const std::string & vectorName)
{
  double d;
  ma->getScalar(vectorName,d);
  return d;
}

boost::python::list getNameList(const sm::MatrixArchive * ma)
{
  boost::python::list list;
  sm::MatrixArchive::matrix_map_t::const_iterator it = ma->begin();
  for( ; it != ma->end(); it++)
    {
      list.append(it->first);
    }
  return list;
}

boost::python::dict asDict(const sm::MatrixArchive * ma)
{
  boost::python::dict dict;
  sm::MatrixArchive::matrix_map_t::const_iterator it = ma->begin();
  for( ; it != ma->end(); it++)
    {
      dict[it->first] = it->second;
    }
  return dict;
}

boost::python::dict stringsAsDict(const sm::MatrixArchive * ma)
{
  boost::python::dict dict;
  auto & strings = ma->getStrings();
  for(auto it = strings.begin() ; it != strings.end(); it++)
    {
      dict[it->first] = it->second;
    }
  return dict;
}

void exportMatrixArchive()
{
  using namespace boost::python;
  using namespace sm;

  //void clear();
  void (MatrixArchive::*clearAll)(void) = &MatrixArchive::clear;
  //void clear(std::string const & entryName);
  void (MatrixArchive::*clearOne)(std::string const &) = &MatrixArchive::clear;

  // void load(std::string const & asaFilePath);
  void (MatrixArchive::*loadArchive)(std::string const &) = &MatrixArchive::load;
  // void save(std::string const & asaFilePath);
  void (MatrixArchive::*saveArchive)(std::string const &)const = &MatrixArchive::save;

  // void append(std::string const & asaFilePath) const;
  void (MatrixArchive::*appendArchive)(std::string const &)const = &MatrixArchive::append;

  // void setScalar(std::string const & scalarName, double scalar);



  // bool isSystemLittleEndian() const;

  // size_t maxNameSize();
  

  class_<MatrixArchive>("MatrixArchive",init<>())
    .def("clear",clearAll)
    .def("clearOne",clearOne)
    .def("size",&MatrixArchive::size)
    .def("load",loadArchive)
    .def("save",saveArchive)
    .def("append",appendArchive)
    .def("setMatrix",&MatrixArchive::setMatrixXd)
    .def("setVector",&MatrixArchive::setVectorXd)
    .def("setScalar",&MatrixArchive::setScalar)
    .def("isSystemLittleEndian",&MatrixArchive::isSystemLittleEndian)
    .def("maxNameSize",&MatrixArchive::maxNameSize)
    .def("getMatrix",&getMatrix)
    .def("getVector",&getVector)
    .def("getScalar",&getScalar)
    .def("getString", getString)
    .def("setString", &MatrixArchive::setString)
    .def("getNameList",&getNameList)
    .def("asDict",&asDict)
    .def("stringsAsDict",&stringsAsDict)
    ;


}
