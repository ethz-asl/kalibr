/**
 * @file   PropertyTree.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>
 * @date   Tue Apr  3 09:39:01 2012
 * 
 * @brief  A class that stores heirarchically orgainized properties for
 *         easy initialization of other classes.
 * 
 *
 * 
 */
#ifndef SM_PROPERTY_TREE_HPP
#define SM_PROPERTY_TREE_HPP

#include <boost/shared_ptr.hpp>
#include <sm/assert_macros.hpp>

namespace sm {
  // Forward declaration.
  class PropertyTreeImplementation;
  
  
  /**
   * @class PropertyTree
   * @brief  A class that stores heirarchically orgainized properties for
   *         easy initialization of other classes.
   * 
   * This class is designed to hold properties organized heirarchically
   * and intended for the initialization of classes. In the basic use
   * case, a class would write a constructor accepting a property tree
   * and use it to initialize internal properties:
   *
   * \code
   * MyClass::MyClass(const PropertyTree & config)
   * {
   *    // Configuration "get" functions that are not given
   *    // a default value must exist in the property tree
   *    // or an exception will be thrown
   *    _theta = config.getDouble("theta");
   *    _numIterations = config.getInt("numIterations");
   *
   *    // Configuration "get" functions that are given
   *    // a default value will not throw an exception
   *    // if the key is not in the property tree.
   *    _phi = config.getDouble("phi", 0.01);
   *
   * }
   * \endcode
   *
   * If one class has a member variable of another class that must
   * also be initialized, the member variable may be pointed to 
   * properties lower in the property heirarchy by creating a new
   * child property tree with a child namespace:
   *
   * \code
   * ComposedClass::ComposedClass(const PropertyTree & config) :
   *   _memberClass1(PropertyTree(config, "member1")), _memberClass2(PropertyTree(config, "member2"))
   * {
   *    // ...
   * }
   * \endcode
   *
   * The class is designed to take an implementation derived from
   * PropertyTreeImplementaion. This should allow us to write
   * implementation classes for ROS and ADTF, two middlewear platforms
   * currently being used in robotics at ETHZ. A sample implementation
   * using the boost::property_tree library is available in this
   * package.
   *
   * To implement your own variant:
   * - First, create a subclass of PropertyTreeImplementation and implement all of 
   *    The virtual functions there. This should do the work of actually getting and
   *    setting parameter values. All values passed in to these functions will start with "/"
   *    and namespaces will be delimited by "/" such as "/namespace1/namespace2/key". If your
   *    middlewear/parameter container uses a different delimiter, you may have to find and 
   *    replace values.
   *
   * - Next create a subclass of PropertyTree that does two things: (1) implements a constructor
   *   that calls the PropertyTree constructor instantiating the correct implementation class, and 
   *   (2) forwards any extra functions from the implementation class that are needed on the outside,
   *   for example save/load functionality. It is important that this class stores no data because it
   *   will be stripped off when the property tree is passed in to sub-objects.
   *
   *   The classes BoostPropertyTreeImplementation and BoostPropertyTree are good examples of how to do this.
   *
   */
  class PropertyTree
  {
  public:
    SM_DEFINE_EXCEPTION(Exception, std::runtime_error);
    SM_DEFINE_EXCEPTION(InvalidKeyException, Exception);
    SM_DEFINE_EXCEPTION(InvalidValueException, Exception);
    SM_DEFINE_EXCEPTION(KeyNotFoundException, Exception);

    PropertyTree(boost::shared_ptr<PropertyTreeImplementation> imp, const std::string & baseNamespace = "");
    PropertyTree(const PropertyTree & parent, const std::string & childNamespace);

    virtual ~PropertyTree();
    
    double getDouble(const std::string & key) const;
    double getDouble(const std::string & key, double defaultValue) const;
    double getDouble(const std::string & key, double defaultValue);

    int getInt(const std::string & key) const;
    int getInt(const std::string & key, int defaultValue) const;
    // The non-const version will call "set" if the value is not already set.
    int getInt(const std::string & key, int defaultValue);

    bool getBool(const std::string & key) const;
    bool getBool(const std::string & key, bool defaultValue) const;
    // The non-const version will call setBool if the value is not already set.
    bool getBool(const std::string & key, bool defaultValue);

    std::string getString(const std::string & key) const;
    std::string getString(const std::string & key, const std::string & defaultValue) const;
    // The non-const version will call setBool if the value is not already set.
    std::string getString(const std::string & key, const std::string & defaultValue) ;

    void setDouble(const std::string & key, double value);
    void setInt(const std::string & key, int value);
    void setBool(const std::string & key, bool value) ;
    void setString(const std::string & key, const std::string & value);


    bool doesKeyExist(const std::string & key) const;
  protected:
    std::string _namespace;
    std::string buildQualifiedKeyName(const std::string & key) const;
    boost::shared_ptr<PropertyTreeImplementation> _imp;
  };

} // namespace sm



#endif /* SM_PROPERTY_TREE_HPP */
