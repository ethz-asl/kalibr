#include <sm/BoostPropertyTree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <sm/BoostPropertyTreeImplementation.hpp>

namespace sm {
  bool BoostPropertyTree::humanReadableInputOutput = false;
  bool BoostPropertyTree::ignoreComments = true;
  
  BoostPropertyTree::BoostPropertyTree(const std::string & baseNamespace) :
    PropertyTree(boost::shared_ptr<PropertyTreeImplementation>(new BoostPropertyTreeImplementation), baseNamespace)
  {

  }

  BoostPropertyTree::BoostPropertyTree(const boost::property_tree::ptree& ptree, const std::string & baseNamespace) :
    PropertyTree(boost::shared_ptr<PropertyTreeImplementation>(new BoostPropertyTreeImplementation(ptree)), baseNamespace)
  {

  }

  BoostPropertyTree::~BoostPropertyTree()
  {

  }

  void BoostPropertyTree::loadXml(const boost::filesystem::path & fileName)
  {
    dynamic_cast<BoostPropertyTreeImplementation*>(_imp.get())->loadXml(fileName);
  }
  void BoostPropertyTree::loadXmlFromString(const std::string & xml)
  {
    dynamic_cast<BoostPropertyTreeImplementation*>(_imp.get())->loadXmlFromString(xml);
  }

  void BoostPropertyTree::saveXml(const boost::filesystem::path & fileName) const
  {
    dynamic_cast<const BoostPropertyTreeImplementation*>(_imp.get())->saveXml(fileName);
  }

  void BoostPropertyTree::loadIni(const boost::filesystem::path & fileName)
  {
    dynamic_cast<BoostPropertyTreeImplementation*>(_imp.get())->loadIni(fileName);
  }

  void BoostPropertyTree::saveIni(const boost::filesystem::path & fileName) const
  {
    dynamic_cast<const BoostPropertyTreeImplementation*>(_imp.get())->saveIni(fileName);
  }


  void BoostPropertyTree::loadInfo(const boost::filesystem::path & fileName)
  {
    dynamic_cast<BoostPropertyTreeImplementation*>(_imp.get())->loadInfo(fileName);
  }

  void BoostPropertyTree::saveInfo(const boost::filesystem::path & fileName) const
  {
    dynamic_cast<const BoostPropertyTreeImplementation*>(_imp.get())->saveInfo(fileName);
  }

  std::string BoostPropertyTree::asInfoString() const {
    return dynamic_cast<const BoostPropertyTreeImplementation*>(_imp.get())->asInfoString();
  }

  void BoostPropertyTree::loadString(const std::string & string)
  {
    dynamic_cast<BoostPropertyTreeImplementation*>(_imp.get())->loadString(string);
  }

  void BoostPropertyTree::loadStrings(const std::vector<std::string> & strings)
  {
    loadString(boost::algorithm::join(strings, ","));
  }

  void BoostPropertyTree::loadStrings(int argc, const char ** argv)
  {
    std::vector<std::string> v;
    for(int i = 0; i < argc; i++){
      v.push_back(argv[i]);
    }
    loadStrings(v);
  }

  BoostPropertyTree::iterator BoostPropertyTree::begin()
  {
    return dynamic_cast<BoostPropertyTreeImplementation*>(_imp.get())->begin();
  }

  BoostPropertyTree::const_iterator BoostPropertyTree::begin() const
  {
    return dynamic_cast<const BoostPropertyTreeImplementation*>(_imp.get())->begin();
  }

  BoostPropertyTree::iterator BoostPropertyTree::end()
  {
    return dynamic_cast<BoostPropertyTreeImplementation*>(_imp.get())->end();
  }

  BoostPropertyTree::const_iterator BoostPropertyTree::end() const
  {
    return dynamic_cast<const BoostPropertyTreeImplementation*>(_imp.get())->end();
  }

  void BoostPropertyTree::update(const BoostPropertyTree & with, bool createIfNecessary, bool ignoreEmptyUpdates)
  {
    return dynamic_cast< BoostPropertyTreeImplementation*>(_imp.get())->update(
        *dynamic_cast<const BoostPropertyTreeImplementation*>(with._imp.get()), createIfNecessary, ignoreEmptyUpdates);
  }

} // namespace sm
