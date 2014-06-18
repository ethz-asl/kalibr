#include <cstdlib>
#include <boost/shared_ptr.hpp>
#include "gtest/gtest.h"
#include <sm/serialization_macros.hpp>
#include <sm/typetraits.hpp>

#ifndef TEST
#define TEST(a, b) void TEST_##a_##b()
#endif  // TEST

class PolyBase {
 public:
  virtual ~PolyBase(){};
 virtual bool isBinaryEqual(const PolyBase& rhs) const = 0;
};

class PolyDerived : public PolyBase {
 private:
  int key_;
  double value_;
 public:
  virtual ~PolyDerived(){};
 void setRandom() {
   key_ = rand();
   value_ = static_cast<double>(rand()) / RAND_MAX;
 }
 virtual bool isBinaryEqual(const PolyDerived& rhs) const {
   bool same = true;
   same = same && SM_CHECKMEMBERSSAME(rhs, key_);
   same = same && SM_CHECKMEMBERSSAME(rhs, value_);
   return same;
 }
};

template<typename T>
class ctPoly{
 private:
  int key_;
  double value_;
 public:
  void setRandom() {
    key_ = rand();
    value_ = static_cast<double>(rand()) / RAND_MAX;
  }
  bool isBinaryEqual(const ctPoly<T>& rhs) const {
    bool same = true;
    same = same && SM_CHECKMEMBERSSAME(rhs, key_);
    same = same && SM_CHECKMEMBERSSAME(rhs, value_);
    return same;
  }
};


template<typename T>
class ctPoly2{
 private:
  int key_;
  double value_;
 public:
  void setRandom() {
    key_ = rand();
    value_ = static_cast<double>(rand()) / RAND_MAX;
  }
  bool operator==(const ctPoly2<T>& other) const {
    bool same = true;
    same = same && SM_CHECKMEMBERSSAME(other, key_);
    same = same && SM_CHECKMEMBERSSAME(other, value_);
    return same;
  }
  friend std::ostream& operator<<(std::ostream &os, const ctPoly2<T>& lhs);
};


class OverloadBase {
public:
  int key_;
  double value_;
};

class Overload : public OverloadBase {
 public:
  void setRandom() {
    key_ = rand();
    value_ = static_cast<double>(rand()) / RAND_MAX;
  }
  bool isBinaryEqual(const Overload& other) const {
    bool same = true;
    same = same && SM_CHECKMEMBERSSAME(other, key_);
    same = same && SM_CHECKMEMBERSSAME(other, value_);
    return same;
  }
  bool isBinaryEqual(const OverloadBase& other) const {
    bool same = true;
      same = same && SM_CHECKMEMBERSSAME(other, key_);
      same = same && SM_CHECKMEMBERSSAME(other, value_);
      return same;
  }
};

class SimpleEntry {
 private:
  int key_;
  double value_;
 public:
  void setRandom() {
    key_ = rand();
    value_ = static_cast<double>(rand()) / RAND_MAX;
  }
  bool operator==(const SimpleEntry& other) const {
    bool same = true;
    same = same && SM_CHECKMEMBERSSAME(other, key_);
    same = same && SM_CHECKMEMBERSSAME(other, value_);
    return same;
  }
  friend std::ostream& operator<<(std::ostream &os, const SimpleEntry& lhs);
};


std::ostream& operator<<(std::ostream &os, const SimpleEntry& lhs)
{
  os << lhs.key_ << " " << lhs.value_;
  return os;
}


class ComplexEntry {
 private:
  int key_;
  double value_;
  SimpleEntry* pSimple_;
  boost::shared_ptr<SimpleEntry> pSharedSimple_;

  bool operator==(const ComplexEntry& other) const;

 public:

  ComplexEntry(){
    key_ = 0;
    value_ = 0;
    pSimple_ = new SimpleEntry;
    pSharedSimple_.reset(new SimpleEntry);
  }

  ~ComplexEntry() {
    delete pSimple_;
    pSimple_ = NULL;
  }

  ComplexEntry(const ComplexEntry& rhs) {
    key_ = rhs.key_;
    value_ = rhs.value_;
    pSimple_ = new SimpleEntry(*rhs.pSimple_);
    pSharedSimple_.reset(new SimpleEntry(*rhs.pSharedSimple_));
  }

  ComplexEntry operator=(const ComplexEntry& rhs) {
    key_ = rhs.key_;
    value_ = rhs.value_;
    pSimple_ = new SimpleEntry(*rhs.pSimple_);
    pSharedSimple_.reset(new SimpleEntry(*rhs.pSharedSimple_));
    return *this;
  }

  void setRandom() {
    key_ = rand();
    value_ = static_cast<double>(rand()) / RAND_MAX;
    pSimple_->setRandom();
    pSharedSimple_->setRandom();
  }

  bool isBinaryEqual(const ComplexEntry& other) const {
    bool same = true;
    same = same && SM_CHECKMEMBERSSAME(other, key_);
    same = same && SM_CHECKMEMBERSSAME(other, value_);
    same = same && SM_CHECKMEMBERSSAME(other, pSimple_);
    same = same && SM_CHECKMEMBERSSAME(other, pSharedSimple_);
    return same;
  }
};

TEST(SerializationMacros, TestClassesComparisonWorks) {
  ComplexEntry e1, e2;
  e1.setRandom();
  e2.setRandom();

  ASSERT_TRUE(e1.isBinaryEqual(e1));
  ASSERT_FALSE(e1.isBinaryEqual(e2));
  ASSERT_FALSE(e2.isBinaryEqual(e1));
}


TEST(SerializationMacros, TestClassesMacroWorks) {
  ComplexEntry e1, e2;
  e1.setRandom();
  e2.setRandom();

  SET_CHECKSAME_SILENT;
  ASSERT_FALSE(SM_CHECKSAME(e1, e2));
  ASSERT_FALSE(SM_CHECKSAME(e2, e1));

  e2 = e1;

  SET_CHECKSAME_VERBOSE;
  ASSERT_TRUE(SM_CHECKSAME(e1, e2));
  ASSERT_TRUE(SM_CHECKSAME(e2, e1));

  ComplexEntry e3(e1);

  ASSERT_TRUE(SM_CHECKSAME(e1, e3));
  ASSERT_TRUE(SM_CHECKSAME(e3, e1));

  SET_CHECKSAME_SILENT;
}

TEST(SerializationMacros, TestClassesCopyCtorAssignWorks) {
  ComplexEntry e1, e2;
  e1.setRandom();
  e2.setRandom();

  ASSERT_TRUE(e1.isBinaryEqual(e1));
  ASSERT_FALSE(e1.isBinaryEqual(e2));
  ASSERT_FALSE(e2.isBinaryEqual(e1));

  e2 = e1;
  ASSERT_TRUE(e2.isBinaryEqual(e1));
  ASSERT_TRUE(e1.isBinaryEqual(e2));

  ComplexEntry e3(e1);
  ASSERT_TRUE(e3.isBinaryEqual(e1));
  ASSERT_TRUE(e1.isBinaryEqual(e3));

}

TEST(SerializationMacros, TestSharedPointer) {
  boost::shared_ptr<ComplexEntry> e1(new ComplexEntry);
  boost::shared_ptr<ComplexEntry> e2(new ComplexEntry);
  boost::shared_ptr<ComplexEntry> e3(new ComplexEntry);

  e1->setRandom();
  e2->setRandom();
  e3->setRandom();

  ASSERT_FALSE(SM_CHECKSAME(e1, e2));

  e1 = e2;

  ASSERT_TRUE(SM_CHECKSAME(e1, e2));

  boost::shared_ptr<ComplexEntry> e4(new ComplexEntry(*e1));

  ASSERT_TRUE(SM_CHECKSAME(e1, e4));
  ASSERT_TRUE(SM_CHECKSAME(e1, e4));
  ASSERT_FALSE(SM_CHECKSAME(e1, e3));
}

TEST(SerializationMacros, TestPointer) {
  ComplexEntry* e1 = new ComplexEntry;
  ComplexEntry* e2 = new ComplexEntry;
  ComplexEntry* e3 = new ComplexEntry;

  e1->setRandom();
  e2->setRandom();
  e3->setRandom();

  ASSERT_FALSE(SM_CHECKSAME(e1, e2));

  ComplexEntry* tmpe1 = e1;
  e1 = e2;

  ASSERT_TRUE(SM_CHECKSAME(e1, e2));

  ComplexEntry* e4 = new ComplexEntry(*e1);

  ASSERT_TRUE(SM_CHECKSAME(e1, e4));
  ASSERT_TRUE(SM_CHECKSAME(e1, e4));
  ASSERT_FALSE(SM_CHECKSAME(e1, e3));

  delete tmpe1;
  delete e2;
  delete e3;
  delete e4;
}


TEST(SerializationMacros, TestClassHasMethodDeduction) {
  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<ComplexEntry>::value, 1);
  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<SimpleEntry>::value, 0);
}

TEST(SerializationMacros, TestClassSharedPtrHasMethodDeduction) {
  typedef boost::shared_ptr<ComplexEntry> T1;
  typedef boost::shared_ptr<SimpleEntry> T2;
  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<T1>::value, 1);
  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<T2>::value, 0);

  T1 t1;
  T2 t2;

  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<decltype(t1) >::value, 1);
  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<decltype(t2) >::value, 0);

  T1& t1r = t1;
  T2& t2r = t2;

  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<decltype(t1r) >::value, 1);
  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<decltype(t2r) >::value, 0);

  const T1& t1cr = t1;
  const T2& t2cr = t2;

  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<
            sm::common::StripConstReference<decltype(t1cr)>::result_t >::value, 1);
  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<
            sm::common::StripConstReference<decltype(t2cr)>::result_t >::value, 0);

}

TEST(SerializationMacros, TestClassPtrHasMethodDeduction) {
  typedef ComplexEntry* T1;
  typedef SimpleEntry* T2;
  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<T1>::value, 1);
  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<T2>::value, 0);
}


TEST(SerializationMacros, TestClassHasStreamOperator) {
  ComplexEntry e1;
  e1.setRandom();

  sm::serialization::internal::streamIf<
  sm::serialization::internal::HasOStreamOperator<std::ostream,
  decltype(e1)>::value, decltype(e1) >::eval(e1);

  ASSERT_EQ((sm::serialization::internal::HasOStreamOperator<
      std::ostream, SimpleEntry>::value), 1);
  ASSERT_EQ((sm::serialization::internal::HasOStreamOperator<
      std::ostream, ComplexEntry>::value), 0);
}

TEST(SerializationMacros, TestSharedPtrHasStreamOperator) {
  typedef boost::shared_ptr<ComplexEntry> T1;
  typedef boost::shared_ptr<SimpleEntry> T2;

  T1 e1(new ComplexEntry);
  e1->setRandom();

  sm::serialization::internal::streamIf<
  sm::serialization::internal::HasOStreamOperator<std::ostream, T1>::value, T1 >::eval(e1);

  ASSERT_EQ((sm::serialization::internal::HasOStreamOperator<std::ostream, T2>::value), 1);
  ASSERT_EQ((sm::serialization::internal::HasOStreamOperator<std::ostream, T1>::value), 0);
}


TEST(SerializationMacros, TestPtrHasStreamOperator) {
  typedef ComplexEntry* T1;
  typedef SimpleEntry* T2;

  T1 e1 = new ComplexEntry;
  e1->setRandom();

  sm::serialization::internal::streamIf<
  sm::serialization::internal::HasOStreamOperator<std::ostream, T1>::value, T1 >::eval(e1);

  ASSERT_EQ((sm::serialization::internal::HasOStreamOperator<std::ostream, T2>::value), 1);
  ASSERT_EQ((sm::serialization::internal::HasOStreamOperator<std::ostream, T1>::value), 0);

  delete e1;
}

TEST(SerializationMacros, TestClassCTPolyHasMethodDeduction) {
  typedef ctPoly<int> T1;
  typedef ctPoly<int>* T2;
  typedef ctPoly2<int> T3;
  typedef ctPoly2<int>* T4;
  typedef ctPoly<T3> T5;
  typedef ctPoly<T3>* T6;
  typedef ctPoly2<T1> T7;
  typedef ctPoly2<T1>* T8;
  ASSERT_TRUE(sm::serialization::internal::HasIsBinaryEqual<T1>::value);
  ASSERT_TRUE(sm::serialization::internal::HasIsBinaryEqual<T2>::value);
  ASSERT_FALSE(sm::serialization::internal::HasIsBinaryEqual<T3>::value);
  ASSERT_FALSE(sm::serialization::internal::HasIsBinaryEqual<T4>::value);
  ASSERT_TRUE(sm::serialization::internal::HasIsBinaryEqual<T5>::value);
  ASSERT_TRUE(sm::serialization::internal::HasIsBinaryEqual<T6>::value);
  ASSERT_FALSE(sm::serialization::internal::HasIsBinaryEqual<T7>::value);
  ASSERT_FALSE(sm::serialization::internal::HasIsBinaryEqual<T8>::value);
}

TEST(SerializationMacros, TestClassPolyPtrHasMethodDeduction) {
  typedef PolyDerived T1;
  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<T1>::value, 1);
}


TEST(SerializationMacros, TestClassOverloadHasMethodDeduction) {
  typedef Overload T1;
  ASSERT_EQ(sm::serialization::internal::HasIsBinaryEqual<T1>::value, 1);
}
