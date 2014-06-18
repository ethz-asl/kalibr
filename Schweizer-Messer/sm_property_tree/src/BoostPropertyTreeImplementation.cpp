#include <sm/BoostPropertyTreeImplementation.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace sm {
  
  BoostPropertyTreeImplementation::BoostPropertyTreeImplementation()
  {

  }

  BoostPropertyTreeImplementation::BoostPropertyTreeImplementation(const boost::property_tree::ptree& ptree)
  : _ptree(ptree)
  {
  }

  BoostPropertyTreeImplementation::~BoostPropertyTreeImplementation()
  {

  }


  double BoostPropertyTreeImplementation::getDouble(const std::string & key) const
  {
    return get<double>(key);
  }

  double BoostPropertyTreeImplementation::getDouble(const std::string & key, double defaultValue) const
  {
    return get<double>(key,defaultValue);
  }

  double BoostPropertyTreeImplementation::getDouble(const std::string & key, double defaultValue)
  {
    return get<double>(key,defaultValue);
  }

  int BoostPropertyTreeImplementation::getInt(const std::string & key) const
  {
    return get<int>(key);
  }

  int BoostPropertyTreeImplementation::getInt(const std::string & key, int defaultValue) const
  {
    return get<int>(key, defaultValue);
  }

  int BoostPropertyTreeImplementation::getInt(const std::string & key, int defaultValue) 
  {
    return get<int>(key, defaultValue);
  }


  bool BoostPropertyTreeImplementation::getBool(const std::string & key) const
  {
    return get<bool>(key);
  }

  bool BoostPropertyTreeImplementation::getBool(const std::string & key, bool defaultValue) const
  {
    return get<bool>(key, defaultValue);
  }

 bool BoostPropertyTreeImplementation::getBool(const std::string & key, bool defaultValue) 
  {
    return get<bool>(key, defaultValue);
  }


  std::string BoostPropertyTreeImplementation::getString(const std::string & key) const
  {
    return get<std::string>(key);
  }

  std::string BoostPropertyTreeImplementation::getString(const std::string & key, const std::string & defaultValue) const
  {
    return get<std::string>(key, defaultValue);
  }

  std::string BoostPropertyTreeImplementation::getString(const std::string & key, const std::string & defaultValue) 
  {
    return get<std::string>(key, defaultValue);
  }


  bool BoostPropertyTreeImplementation::doesKeyExist(const std::string & key) const
  {
    boost::optional<std::string> val = _ptree.get<std::string>(boost::property_tree::ptree::path_type(key.c_str(),'/'));
    
    return (bool)val;
  }


  void BoostPropertyTreeImplementation::loadXml(const boost::filesystem::path & fileName)
  {
    boost::property_tree::read_xml(fileName.string(), _ptree);
  }

  void BoostPropertyTreeImplementation::saveXml(const boost::filesystem::path & fileName) const
  {
    boost::property_tree::write_xml(fileName.string(), _ptree);
  }
 
  void BoostPropertyTreeImplementation::loadIni(const boost::filesystem::path & fileName)
  {
    boost::property_tree::read_ini(fileName.string(), _ptree);
  }

  void BoostPropertyTreeImplementation::saveIni(const boost::filesystem::path & fileName) const
  {
    boost::property_tree::write_ini(fileName.string(), _ptree);
  }


  void BoostPropertyTreeImplementation::loadInfo(const boost::filesystem::path & fileName)
  {
    boost::property_tree::read_info(fileName.string(), _ptree);
  }

  void BoostPropertyTreeImplementation::saveInfo(const boost::filesystem::path & fileName) const
  {
    boost::property_tree::write_info(fileName.string(), _ptree);
  }

  void BoostPropertyTreeImplementation::setDouble(const std::string & key, double value)
  {
    set<double>(key,value);
  }

  void BoostPropertyTreeImplementation::setInt(const std::string & key, int value)
  {
    set<int>(key,value);
  }

  void BoostPropertyTreeImplementation::setBool(const std::string & key, bool value)
  {
    set<bool>(key,value);
  }

  void BoostPropertyTreeImplementation::setString(const std::string & key, const std::string & value)
  {
    set<std::string>(key,value);
  }

  BoostPropertyTreeImplementation::iterator BoostPropertyTreeImplementation::begin()
    {
      return _ptree.begin();
    }

    BoostPropertyTreeImplementation::const_iterator BoostPropertyTreeImplementation::begin() const
    {
      return _ptree.begin();
    }

    BoostPropertyTreeImplementation::iterator BoostPropertyTreeImplementation::end()
    {
      return _ptree.end();
    }

    BoostPropertyTreeImplementation::const_iterator BoostPropertyTreeImplementation::end() const
    {
      return _ptree.end();
    }


} // namespace sm
