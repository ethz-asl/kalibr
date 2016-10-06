/*

Copyright (c) 2012, Simon Lynen, ASL, ETH Zurich, Switzerland
You can contact the author at <slynen at ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


#ifndef SM_COMMON_TYPETRAITS_HPP_
#define SM_COMMON_TYPETRAITS_HPP_

namespace sm{
namespace common{

//two types of same type
template <typename T, typename U> struct SameType{enum { value = false };};
template <typename T> struct SameType<T,T>{enum { value = true };};

//strip qualifiers
template<typename T>
struct StripReference{
  typedef T result_t;
};
template<typename T>
struct StripReference<T&>{
  typedef T result_t;
};
template<typename T>
struct StripReference<const T>{
  typedef const T result_t;
};
template<typename T>
struct StripReference<const T&>{
  typedef const T result_t;
};

template<typename T>
struct StripConstReference{
  typedef T result_t;
};
template<typename T>
struct StripConstReference<T&>{
  typedef T result_t;
};
template<typename T>
struct StripConstReference<const T&>{
  typedef T result_t;
};
template<typename T>
struct StripConstReference<const T>{
  typedef T result_t;
};

template<typename T>
struct StripConstPtr{
  typedef T result_t;
};
template<typename T>
struct StripConstPtr<T*>{
  typedef T result_t;
};
template<typename T>
struct StripConstPtr<const T*>{
  typedef T result_t;
};
template<typename T>
struct StripConstPtr<const T>{
  typedef T result_t;
};

//add qualifiers
template<typename T>
struct AddReference{
  typedef T& result_t;
};
template<typename T>
struct AddReference<T&>{
  typedef T& result_t;
};
template<typename T>
struct AddReference<const T>{
  typedef const T& result_t;
};
template<typename T>
struct AddReference<const T&>{
  typedef const T& result_t;
};

template<typename T>
struct AddConstReference{
  typedef const T& result_t;
};
template<typename T>
struct AddConstReference<T&>{
  typedef const T& result_t;
};
template<typename T>
struct AddConstReference<const T>{
  typedef const T& result_t;
};
template<typename T>
struct AddConstReference<const T&>{
  typedef const T& result_t;
};

template<typename T>
struct AddPtr{
  typedef T* result_t;
};
template<typename T>
struct AddPtr<T*>{
  typedef T* result_t;
};
template<typename T>
struct AddPtr<const T>{
  typedef const T* result_t;
};
template<typename T>
struct AddPtr<const T*>{
  typedef const T* result_t;
};

template<typename T>
struct AddConstPtr{
  typedef const T* result_t;
};
template<typename T>
struct AddConstPtr<T*>{
  typedef const T* result_t;
};
template<typename T>
struct AddConstPtr<const T>{
  typedef const T* result_t;
};
template<typename T>
struct AddConstPtr<const T*>{
  typedef const T* result_t;
};

template<typename T>
struct IsReferenceType{
  enum{
    value = false
  };
};
template<typename T>
struct IsReferenceType<T&>{
  enum{
    value = true
  };
};
template<typename T>
struct IsReferenceType<const T&>{
  enum{
    value = true
  };
};


template<typename T>
struct IsPointerType{
  enum{
    value = false
  };
};
template<typename T>
struct IsPointerType<T*>{
  enum{
    value = true
  };
};
template<typename T>
struct IsPointerType<const T*>{
  enum{
    value = true
  };
};

}
}

#endif /* SM_COMMON_TYPETRAITS_HPP_ */
