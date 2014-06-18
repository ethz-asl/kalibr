#include <sm/PropertyTree.hpp>
#include <sm/string_routines.hpp>
#include <sm/PropertyTreeImplementation.hpp>

namespace sm {
  
  PropertyTree::PropertyTree(boost::shared_ptr<PropertyTreeImplementation> imp, const std::string & baseNamespace) : 
    _namespace(ensureTrailingBackslash(baseNamespace)), _imp(imp)
  {
    if(_namespace.size() > 0 && _namespace[0] != '/')
      _namespace = "/" + _namespace;
  }
  
  PropertyTree::PropertyTree(const PropertyTree & parent, const std::string & childNamespace) :
     _imp(parent._imp)
  {
      if(childNamespace.size() > 0 && childNamespace[0] == '/')
      {
          _namespace = ensureTrailingBackslash(childNamespace);
      }
      else 
      {
          _namespace = ensureTrailingBackslash(parent._namespace + childNamespace);
      }

    if(_namespace.size() > 0 && _namespace[0] != '/')
      _namespace = "/" + _namespace;    
  }


  PropertyTree::~PropertyTree()
  {

  }

  // \todo This function could do more name checking.
  std::string PropertyTree::buildQualifiedKeyName(const std::string & key) const
  {
    SM_ASSERT_GT(InvalidKeyException, key.size(), 0, "The key must contain characters");
    if(key[0] == '/')
      {
	// The requested key is in the global namespace
	SM_ASSERT_GT(InvalidKeyException, key.size(), 1, "The key \"" << key << "\" is in the global namespace but does not contain a variable name");
	return key;
      }
    else
      {
	return _namespace + key;
      }
  }

    
  double PropertyTree::getDouble(const std::string & key) const
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->getDouble(buildQualifiedKeyName(key));
  }

  double PropertyTree::getDouble(const std::string & key, double defaultValue) const
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
      return _imp->getDouble(buildQualifiedKeyName(key), defaultValue);
  }

  double PropertyTree::getDouble(const std::string & key, double defaultValue)
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->getDouble(buildQualifiedKeyName(key), defaultValue);
  }
  
  int PropertyTree::getInt(const std::string & key) const
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->getInt(buildQualifiedKeyName(key));
  }

  int PropertyTree::getInt(const std::string & key, int defaultValue) const
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->getInt(buildQualifiedKeyName(key),defaultValue);
  }

  int PropertyTree::getInt(const std::string & key, int defaultValue)
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->getInt(buildQualifiedKeyName(key),defaultValue);
  }

  bool PropertyTree::getBool(const std::string & key) const
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->getBool(buildQualifiedKeyName(key));
  }

  bool PropertyTree::getBool(const std::string & key, bool defaultValue) const
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->getBool(buildQualifiedKeyName(key), defaultValue);
  }

  bool PropertyTree::getBool(const std::string & key, bool defaultValue) 
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->getBool(buildQualifiedKeyName(key), defaultValue);
  }

  
  std::string PropertyTree::getString(const std::string & key) const
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->getString(buildQualifiedKeyName(key));
  }

  std::string PropertyTree::getString(const std::string & key, const std::string & defaultValue) const
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->getString(buildQualifiedKeyName(key), defaultValue);
  }

  std::string PropertyTree::getString(const std::string & key, const std::string & defaultValue) 
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->getString(buildQualifiedKeyName(key), defaultValue);
  }
  
  bool PropertyTree::doesKeyExist(const std::string & key) const
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->doesKeyExist(buildQualifiedKeyName(key));
  }


  void PropertyTree::setDouble(const std::string & key, double value)
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->setDouble(buildQualifiedKeyName(key), value);
  }

  void PropertyTree::setInt(const std::string & key, int value)
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->setInt(buildQualifiedKeyName(key), value);
  }

  void PropertyTree::setBool(const std::string & key, bool value)
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->setBool(buildQualifiedKeyName(key), value);
  }
  
  void PropertyTree::setString(const std::string & key, const std::string & value)
  {
      SM_ASSERT_TRUE(Exception, _imp, "The implementation is NULL");
    return _imp->setString(buildQualifiedKeyName(key), value);
  }

  
} // namespace sm
