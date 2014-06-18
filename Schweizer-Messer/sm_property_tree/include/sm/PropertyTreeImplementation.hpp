#ifndef SM_PROPERTY_TREE_IMPLEMENTATION_HPP
#define SM_PROPERTY_TREE_IMPLEMENTATION_HPP

#include <string>

namespace sm {

  class PropertyTreeImplementation
  {
  public:
    PropertyTreeImplementation();
    virtual ~PropertyTreeImplementation();

    virtual double getDouble(const std::string & key) const = 0;
    virtual double getDouble(const std::string & key, double defaultValue) const = 0;
    virtual double getDouble(const std::string & key, double defaultValue) = 0;

    virtual int getInt(const std::string & key) const = 0;
    virtual int getInt(const std::string & key, int defaultValue) const = 0;
    // The non-const version will call "set" if the value is not already set.
    virtual int getInt(const std::string & key, int defaultValue) = 0;

    virtual bool getBool(const std::string & key) const = 0;
    virtual bool getBool(const std::string & key, bool defaultValue) const = 0;
    // The non-const version will call setBool if the value is not already set.
    virtual bool getBool(const std::string & key, bool defaultValue) = 0;

    virtual std::string getString(const std::string & key) const = 0;
    virtual std::string getString(const std::string & key, const std::string & defaultValue) const = 0;
    // The non-const version will call setBool if the value is not already set.
    virtual std::string getString(const std::string & key, const std::string & defaultValue)  = 0;

    virtual void setDouble(const std::string & key, double value) = 0;
    virtual void setInt(const std::string & key, int value) = 0;
    virtual void setBool(const std::string & key, bool value) = 0;
    virtual void setString(const std::string & key, const std::string & value) = 0;

    virtual bool doesKeyExist(const std::string & key) const = 0;

  };

} // namespace sm


#endif /* SM_PROPERTY_TREE_IMPLEMENTATION_HPP */
