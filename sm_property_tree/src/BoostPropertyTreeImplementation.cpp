#include <sm/BoostPropertyTree.hpp>
#include <sm/BoostPropertyTreeImplementation.hpp>

#include <sstream>
#include <boost/make_shared.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/version.hpp>

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
    std::string keyNoLeadingSlash = (key.empty() || key[0] != '/') ? key : key.substr(1); // Note: _ptree.get_optional does not seem to like leading slashes
    boost::optional<std::string> val = _ptree.get_optional<std::string>(boost::property_tree::ptree::path_type(keyNoLeadingSlash.c_str(),'/'));

    return (bool)val;
  }


  void BoostPropertyTreeImplementation::loadXml(const boost::filesystem::path & fileName)
  {
    boost::property_tree::read_xml(fileName.string(), _ptree, getXmlReadOptions());
  }

  void BoostPropertyTreeImplementation::loadXmlFromString(const std::string & xml)
  {
    std::istringstream in(xml);
    boost::property_tree::read_xml(in, _ptree, getXmlReadOptions());
  }


  void BoostPropertyTreeImplementation::saveXml(const boost::filesystem::path & fileName) const
  {
    if(BoostPropertyTree::isHumanReadableInputOutput()){
#if BOOST_VERSION >= 105600
      boost::property_tree::write_xml(fileName.string(), _ptree, std::locale(),
                                      boost::property_tree::xml_writer_make_settings<ptree::key_type>('\t', 1));
#else
      boost::property_tree::xml_writer_settings<ptree::key_type::value_type> settings('\t', 1);
      boost::property_tree::write_xml(fileName.string(), _ptree, std::locale(), settings);
#endif
    }
    else {
      boost::property_tree::write_xml(fileName.string(), _ptree);
    }
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

  std::string BoostPropertyTreeImplementation::asInfoString() const {
    std::ostringstream iss;
    boost::property_tree::write_info(iss, _ptree);
    return iss.str();
  }

  void BoostPropertyTreeImplementation::loadString(const std::string & string){
    if(string.find('"') != string.npos) {
      throw boost::property_tree::file_parser_error("No escaped strings supported with loadString, yet.", string, 0);
    }
    std::stringstream s;
    std::vector<int> countBraceOpen(int(0));
    countBraceOpen.resize(1);

    const char * specilaChars = ",/={}";
    for(size_t oldPos = 0, pos = string.find_first_of(specilaChars); oldPos <= string.size(); (oldPos = pos, pos = string.find_first_of(specilaChars, pos))){
      char c;
      if(pos == string.npos){
        pos = string.size();
        c = ',';
      }else{
        c = string[pos];
      }
      s << string.substr(oldPos, pos - oldPos);
      switch(c){
        case '=':
          s << ' ';
          break;
        case '/':
        case '{':
          if(c == '/'){
            countBraceOpen.back()++;
          }
          else{
            countBraceOpen.push_back(0);
          }
          s << "\n{\n";
          break;
        case '}':
          if(countBraceOpen.size() <= 1){
            throw boost::property_tree::file_parser_error("Unmatched closing bracket '}' at ", string, pos);
          }
        case ',':
          for(; countBraceOpen.back(); countBraceOpen.back() --){
            s << "\n}";
          }
          s << '\n';
          if(c == '}'){
            countBraceOpen.pop_back();
            s << '}';
          }
          break;
      }
      pos ++;
    }
    boost::property_tree::read_info(s, _ptree);
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


  const std::vector<KeyPropertyTreePair> sm::BoostPropertyTreeImplementation::getChildren(const std::string & key) const {
    return const_cast<BoostPropertyTreeImplementation*>(this)->getChildren(key);
  }

  std::vector<KeyPropertyTreePair> sm::BoostPropertyTreeImplementation::getChildren(const std::string & key) {
    std::vector<KeyPropertyTreePair> ret;
    auto * pt = &_ptree;
    std::string k;
    if(!key.empty()){
      k = key[0] == '/' ? key.substr(1) : key;
      if (k.back() == '/'){
        k.resize(k.size() - 1);
      }
    }
    if (!k.empty()){
      std::replace(k.begin(), k.end(), '/', '.');
      pt = & _ptree.get_child(k);
    }
    for(auto& p : *pt){
      ret.push_back({p.first, PropertyTree(boost::make_shared<BoostPropertyTreeImplementation>(p.second), "")});
    }
    return ret;
  }


  /*
   * copied from http://stackoverflow.com/questions/8154107/how-do-i-merge-update-a-boostproperty-treeptree :
   */

  template<typename T>
  void traverse_recursive(const boost::property_tree::ptree::path_type &childPath, const boost::property_tree::ptree &child, T method)
  {
    using boost::property_tree::ptree;

    method(childPath, child);
    for(ptree::const_iterator it=child.begin();it!=child.end();++it) {
      ptree::path_type curPath = childPath / ptree::path_type(it->first);
      traverse_recursive(curPath, it->second, method);
    }
  }

  void updateOnly(boost::property_tree::ptree &dest, bool ignoreEmptyUpdates, const boost::property_tree::ptree::path_type &childPath, const boost::property_tree::ptree &child) {
    if(ignoreEmptyUpdates && (child.data().empty() || child.data().find_first_not_of(" \n\t\r") == std::string::npos)) return;
    if(!dest.get_optional<std::string>(childPath)) {
      throw PropertyTree::KeyNotFoundException(std::string("Could not find the destination '") + childPath.dump() + "' to update with '" + child.data() + "'.");
    }
    dest.put(childPath, child.data());
  }
  void merge(boost::property_tree::ptree &dest, bool ignoreEmptyUpdates, const boost::property_tree::ptree::path_type &childPath, const boost::property_tree::ptree &child) {
    if(ignoreEmptyUpdates && (child.data().empty() || child.data().find_first_not_of(" \n\t\r") == std::string::npos)) return;
    dest.put(childPath, child.data());
  }

  template<typename T>
  void traverse(const boost::property_tree::ptree &parent, T method)
  {
    traverse_recursive("", parent, method);
  }

  void BoostPropertyTreeImplementation::update(const BoostPropertyTreeImplementation & with, bool createIfNecessary, bool ignoreEmptyUpdates){
    using namespace boost;
    traverse(with._ptree, createIfNecessary ? bind(merge, boost::ref(_ptree), ignoreEmptyUpdates, _1, _2) : bind(updateOnly, boost::ref(_ptree), ignoreEmptyUpdates, _1, _2));
  }

  int BoostPropertyTreeImplementation::getXmlReadOptions() {
    int options = 0;
    if (BoostPropertyTree::isHumanReadableInputOutput()) { options |= boost::property_tree::xml_parser::trim_whitespace; }
    if (BoostPropertyTree::isIgnoreComments()) { options |= boost::property_tree::xml_parser::no_comments; }
    return options;
  }
} // namespace sm
