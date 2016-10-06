/*
 * Adapted from the Willow Garage pose_graph package by Paul Furgale on October 22, 2010.
 * 
 * Original file:
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef SM_ID_HPP
#define SM_ID_HPP

#include <boost/cstdint.hpp>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <functional>
#include <boost/serialization/nvp.hpp>

namespace sm {
    typedef boost::uint64_t id_type;

  ///
  /// \class Id
  /// \brief superclass for stronly-typed uint64 ids
  ///
  /// Usage:
  /// \code
  /// // MyId is a stronly typed Id for my object.
  /// class MyId : public Id
  /// {
  /// public:
  ///
  ///   explicit MyId (id_type id = -1) : Id(id) {}
  ///
  ///   template<class Archive>
  ///   void serialize(Archive & ar, const unsigned int version)
  ///   {
  ///     ar & id_;
  ///   }
  /// };
  /// \endcode
  ///
  class Id
  {
  public:
      
    explicit Id (id_type id) : _id(id) {}

    /// Only for stl use
    Id () : _id(-1) {}

    friend std::ostream& operator<< (std::ostream& str, const Id& n)
    {
      str << n._id;
      return str;
    }

    bool operator== (const Id& other) const
    {
      return other._id == _id;
    }

    bool operator!= (const Id& other) const
    {
      return other._id != _id;
    }

    bool operator< (const Id& other) const
    {
      return _id < other._id;
    }

    bool operator> (const Id& other) const
    {
      return _id > other._id;
    }

    bool operator<=(const Id& other) const
    {
      return _id <= other._id;
    }

    bool operator>=(const Id& other) const
    {
      return _id >= other._id;
    }

    id_type getId () const
    {
      return _id;
    }

    // postincrement.
    Id operator++ (int /*unused*/)
    {
        Id rval(_id);
      ++_id;
      return rval;
    }

    // preincrement
    Id & operator++ ()
    {
      ++_id;
      return (*this);
    }

    Id& operator= (const Id& other)
    {
      _id = other._id;
      return *this;
    }

    bool isSet() const
    {
      return _id != id_type(-1);
    }

    id_type value() const
    {
      return _id;
    }

    void clear()
    {
      _id = -1;
    }

    void swap( Id & rhs)
    {
      std::swap(_id,rhs._id);
    }

    void setRandom()
    {
      _id = rand();
    }

    bool isBinaryEqual(const Id & rhs)
    {
      return _id == rhs._id;
    }
  protected:
    id_type _id;
  
  };

} // namespace aslam


#define SM_DEFINE_ID(IdTypeName)                                        \
    class IdTypeName : public sm::Id                                    \
    {                                                                   \
    public:                                                             \
        explicit IdTypeName (sm::id_type id = -1) : sm::Id(id) {}       \
        IdTypeName(const sm::Id & id) : sm::Id(id){}                    \
        template<class Archive>                                         \
        void serialize(Archive & ar, const unsigned int /* version */)  \
        {                                                               \
            ar & BOOST_SERIALIZATION_NVP(_id);                          \
        }                                                               \
    };									

// If you need to use the ID in a tr1 hashing container,
// use this macro outside of any namespace:
// SM_DEFINE_ID_HASH(my_namespace::myIdType);
#define SM_DEFINE_ID_HASH(FullyQualifiedIdTypeName)                 \
    namespace std {                                                 \
        template<>                                                  \
        struct hash<FullyQualifiedIdTypeName>                       \
        {                                                           \
            hash<boost::uint64_t> _hash;                            \
            size_t operator()(const FullyQualifiedIdTypeName & id) const  \
            {                                                       \
                return _hash(id.getId());                           \
            }                                                       \
        };                                                          \
    }                                                                   \
    namespace boost {                                                   \
        template<>                                                      \
        struct hash<FullyQualifiedIdTypeName>                           \
        {                                                               \
            hash<boost::uint64_t> _hash;                                \
            size_t operator()(const FullyQualifiedIdTypeName & id) const \
            {                                                           \
                return _hash(id.getId());                               \
            }                                                           \
        };                                                              \
        } // namespace boost

#endif /* SM_ID_HPP */
