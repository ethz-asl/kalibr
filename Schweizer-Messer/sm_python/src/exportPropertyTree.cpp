#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/BoostPropertyTree.hpp>

double getDouble(const sm::PropertyTree * p, const std::string & key) {
  return p->getDouble(key);
}

double getDoubleDefault(sm::PropertyTree * p, const std::string & key, double defaultValue) {
  return p->getDouble(key, defaultValue);
}

int getInt(const sm::PropertyTree * p, const std::string & key) {
  return p->getInt(key);
}

int getIntDefault(sm::PropertyTree * p, const std::string & key, int defaultValue) {
  return p->getInt(key, defaultValue);
}

bool getBool(const sm::PropertyTree * p, const std::string & key) {
  return p->getBool(key);
}

bool getBoolDefault(sm::PropertyTree * p, const std::string & key, bool defaultValue) {
  return p->getBool(key, defaultValue);
}

std::string getString(const sm::PropertyTree * p, const std::string & key) {
  return p->getString(key);
}

std::string getStringDefault(sm::PropertyTree * p, const std::string & key, const std::string & defaultValue) {
  return p->getString(key, defaultValue);
}

void exportPropertyTree() {
  using namespace boost::python;
  using namespace sm;

  class_<PropertyTree>("PropertyTree", init<const PropertyTree &, const std::string &>("PropertyTree(PropertyTree parent, string childNamespace)"))
      .def("getInt", &getInt).def("getIntDefault", &getIntDefault)
      .def("getString", &getString)
      .def("getStringDefault", &getStringDefault)
      .def("getBool", &getBool).def("getBoolDefault", &getBoolDefault)
      .def("getDouble", &getDouble).def("getDoubleDefault", &getDoubleDefault)
      .def("setInt", &PropertyTree::setInt)
      .def("setBool", &PropertyTree::setBool)
      .def("setString", &PropertyTree::setString)
      .def("setDouble", &PropertyTree::setDouble)
      .def("doesKeyExist", &PropertyTree::doesKeyExist);

  class_<BoostPropertyTree, bases<PropertyTree> >("BoostPropertyTree", init<>())
      .def(init<std::string>("BoostPropertyTree( string baseNamespace )"))
      .def("loadXml", &BoostPropertyTree::loadXmlStr)
      .def("saveXml", &BoostPropertyTree::saveXmlStr)
      .def("loadIni", &BoostPropertyTree::loadIniStr)
      .def("saveIni", &BoostPropertyTree::saveIniStr)
      .def("loadInfo", &BoostPropertyTree::loadInfoStr)
      .def("saveInfo", &BoostPropertyTree::saveInfoStr);
}
