#ifndef ASLAM_BACKEND_FIXED_POINT_NUMBER_HPP
#define ASLAM_BACKEND_FIXED_POINT_NUMBER_HPP

#include <cstdint>
#include <limits>

namespace aslam {
namespace backend {

template <typename Integer_>
struct BiggerType;


template <>
struct BiggerType<char> {
  typedef std::int16_t type;
};

template <>
struct BiggerType<std::int8_t> {
  typedef std::int16_t type;
};

template <>
struct BiggerType<std::int16_t> {
  typedef std::int32_t type;
};

template <>
struct BiggerType<std::int32_t> {
  typedef std::int64_t type;
};
template <>
struct BiggerType<std::int64_t> {
  typedef long double type;
};

#ifndef __APPLE__
// On apple long long and std::int64_t are the same
// and this causes a compiler error
template <>
struct BiggerType<long long> {
  typedef long double type;
};
#endif

template <typename Integer_, std::uintmax_t Divider>
class FixedPointNumber{
 public:
  typedef Integer_ Integer;
  typedef typename BiggerType<Integer>::type NextBiggerType;

  inline constexpr static std::uintmax_t getDivider(){ return Divider; }


  inline FixedPointNumber() = default;
  inline FixedPointNumber(const FixedPointNumber & other) = default;

  inline FixedPointNumber(Integer p) : _p(p){}
  inline explicit FixedPointNumber(double const & other) : _p(other * getDivider()) {}

  template <typename OtherInteger_, std::uintmax_t OtherDivider_>
  inline explicit FixedPointNumber(FixedPointNumber<OtherInteger_, OtherDivider_> const & other) { _p = other._p * getDivider() / other.getDivider(); }

  ~FixedPointNumber(){
    static_assert(std::numeric_limits<Integer_>::is_integer, "only integral types are allowed as Integer_");
    static_assert(Divider != 0, "the Divider must be zero");
    static_assert((1.0/(double)Divider) * Divider == 1, "the Divider must be loss less convertible to double");
    static_assert((std::uintmax_t)((Integer)Divider) == Divider, "the Divider must be loss less convertible to the Integer_ type");
  }

  inline operator double() const { return (long double) _p / getDivider(); }
//  inline operator Integer() = delete; TODO find out : why does this disable conversions to double?
//  inline operator Integer() const { return getNumerator(); }
  inline Integer getNumerator() const { return _p; }
  inline Integer getDenominator() const { return Integer(getDivider()); }

  FixedPointNumber operator - () const {
    return FixedPointNumber((Integer)(-_p));
  }

  FixedPointNumber operator + (const FixedPointNumber & other) const {
    return FixedPointNumber((Integer)(_p + other._p));
  }
  FixedPointNumber & operator += (const FixedPointNumber & other) {
    _p += other._p;
    return *this;
  }

  FixedPointNumber operator - (const FixedPointNumber & other) const {
    return FixedPointNumber((Integer)(_p - other._p));
  }
  FixedPointNumber & operator -= (const FixedPointNumber & other) {
    _p -= other._p;
    return *this;
  }

  FixedPointNumber operator * (const FixedPointNumber & other) const {
    return FixedPointNumber((Integer)(((NextBiggerType)_p * (NextBiggerType)other._p) / (NextBiggerType)other.getDivider()));
  }
  FixedPointNumber & operator *= (const FixedPointNumber & other) {
    *this = *this * other;
    return *this;
  }

  FixedPointNumber operator / (const FixedPointNumber & other) const {
    return FixedPointNumber((Integer)(((NextBiggerType)_p * (NextBiggerType)other.getDivider()) / (NextBiggerType)other._p));
  }
  FixedPointNumber & operator /= (const FixedPointNumber & other) {
    *this = *this / other;
    return *this;
  }

  inline bool operator == (const FixedPointNumber & other) const {
    return _p == other._p;
  }
  inline bool operator != (const FixedPointNumber & other) const {
    return _p != other._p;
  }


 private:
  Integer _p;
  template<typename OtherInteger_, std::uintmax_t OtherDivider>
  friend class FixedPointNumber;
};

template <typename T>
struct is_fixed_point_number {
  constexpr static bool value = false;
};

template <typename Integer_, std::uintmax_t Divider>
struct is_fixed_point_number<FixedPointNumber<Integer_, Divider>> {
  constexpr static bool value = true;
};

}  // namespace backend
}  // namespace aslam

namespace std {

template <typename Integer_, std::uintmax_t Divider>
struct numeric_limits<aslam::backend::FixedPointNumber<Integer_, Divider> > {
  constexpr static double epsilon() { return 1.0 / Divider; }
  constexpr static bool is_integer = Divider == 1;
  constexpr static bool is_signed = std::numeric_limits<Integer_>::is_signed;
};
}

#endif /* ASLAM_BACKEND_FIXED_POINT_NUMBER_HPP */
