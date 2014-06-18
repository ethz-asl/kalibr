#ifndef SM_BOOST_PROPERTY_TREE_IMPLEMENTATION_HPP
#define SM_BOOST_PROPERTY_TREE_IMPLEMENTATION_HPP

#include "PropertyTreeImplementation.hpp"
#include "PropertyTree.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem/path.hpp>
#include <typeinfo>
namespace sm {
  
  class BoostPropertyTreeImplementation : public PropertyTreeImplementation
  {
  public:
    typedef boost::property_tree::ptree ptree;
    typedef ptree::iterator iterator;
    typedef ptree::const_iterator const_iterator;

    BoostPropertyTreeImplementation();
    BoostPropertyTreeImplementation(const boost::property_tree::ptree& ptree);
    virtual ~BoostPropertyTreeImplementation();

    void loadXml(const boost::filesystem::path & fileName);
    void saveXml(const boost::filesystem::path & fileName) const;
 
    void loadIni(const boost::filesystem::path & fileName);
    void saveIni(const boost::filesystem::path & fileName) const;

    void loadInfo(const boost::filesystem::path & fileName);
    void saveInfo(const boost::filesystem::path & fileName) const;

    virtual double getDouble(const std::string & key) const;
    virtual double getDouble(const std::string & key, double defaultValue) const;
    virtual double getDouble(const std::string & key, double defaultValue);

    virtual int getInt(const std::string & key) const;
    virtual int getInt(const std::string & key, int defaultValue) const;
    virtual int getInt(const std::string & key, int defaultValue);

    virtual bool getBool(const std::string & key) const;
    virtual bool getBool(const std::string & key, bool defaultValue) const;
    virtual bool getBool(const std::string & key, bool defaultValue);

    virtual std::string getString(const std::string & key) const;
    virtual std::string getString(const std::string & key, const std::string & defaultValue) const;
    virtual std::string getString(const std::string & key, const std::string & defaultValue);

    virtual void setDouble(const std::string & key, double value);
    virtual void setInt(const std::string & key, int value);
    virtual void setBool(const std::string & key, bool value);
    virtual void setString(const std::string & key, const std::string & value);

    virtual bool doesKeyExist(const std::string & key) const;

    iterator begin() ;
    const_iterator begin() const;
    iterator end() ;
    const_iterator end() const;    


  private:

    template<typename T>
    T get(const std::string & key) const;

    template<typename T>
    T get(const std::string & key, const T & defaultValue) const;

    template<typename T>
    T get(const std::string & key, const T & defaultValue);

    template<typename T>
    void set(const std::string & key, const T & value);
    
    // Make this a shared pointer so the datastructure can be easily copied around.
    // and refer to the same tree underneath.
    boost::property_tree::ptree _ptree;

  };

  template<typename T>
  T BoostPropertyTreeImplementation::get(const std::string & key) const
  {
    using namespace boost::property_tree;
    T rval;
    try 
      {
	rval = _ptree.get<T>(ptree::path_type(key.substr(1),'/'));
      }
    catch(const ptree_bad_data & e)
      {
          SM_THROW(PropertyTree::InvalidValueException, "Unable to get the value for key \"" << key << "\" with value \"" << _ptree.get<std::string>(ptree::path_type(key.substr(1),'/')) << "\" as type " << typeid(T).name() << ": " << e.what());
      }
    catch(const ptree_bad_path & e)
      {
	SM_THROW(PropertyTree::InvalidKeyException, "The key \"" << key << "\" was invalid: " << e.what());
      }

    return rval;
  }
  
  template<typename T>
  T BoostPropertyTreeImplementation::get(const std::string & key, const T & defaultValue) const
  {
   boost::optional<T> val = _ptree.get_optional<T>(boost::property_tree::ptree::path_type(key.substr(1),'/'));
    if(val)
      {
	return *val;
      }
    else
      {
	return defaultValue;
      }
  }

  template<typename T>
  T BoostPropertyTreeImplementation::get(const std::string & key, const T & defaultValue)
  {
    boost::optional<T> val = _ptree.get_optional<T>(boost::property_tree::ptree::path_type(key.substr(1),'/'));
    if(val)
      {
	return *val;
      }
    else
      {
	set(key,defaultValue);
	return defaultValue;
      }
  }

  template<typename T>
  void BoostPropertyTreeImplementation::set(const std::string & key, const T & value)
  {
    _ptree.put<T>(boost::property_tree::ptree::path_type(key.substr(1),'/'), value);
  }
} // namespace sm


#endif /* SM_BOOST_PROPERTY_TREE_IMPLEMENTATION_HPP */
