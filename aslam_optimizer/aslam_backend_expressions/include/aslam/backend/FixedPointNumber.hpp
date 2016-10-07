#ifndef ASLAM_BACKEND_FIXED_POINT_NUMBER_HPP
#define ASLAM_BACKEND_FIXED_POINT_NUMBER_HPP

#include <cstdint>
#include <limits>
#include <type_traits>
#include <iosfwd>

namespace aslam {
namespace backend {

namespace internal {
template <bool Cond, typename IfTrue, typename IfFalse>
struct SwitchType;

template <typename IfTrue, typename IfFalse>
struct SwitchType<true, IfTrue, IfFalse>{
  typedef IfTrue type;
};

template <typename IfTrue, typename IfFalse>
struct SwitchType<false, IfTrue, IfFalse>{
  typedef IfFalse type;
};

template <unsigned char SizeInBytes>
struct NextBiggerNumberTypeBySize;

template <>
struct NextBiggerNumberTypeBySize<1> {
  typedef std::int16_t type;
};

template <>
struct NextBiggerNumberTypeBySize<2> {
  typedef std::int32_t type;
};

template <>
struct NextBiggerNumberTypeBySize<4> {
  typedef std::int64_t type;
};

template <>
struct NextBiggerNumberTypeBySize<8> {
  typedef SwitchType<(sizeof(long long) > 8), long long, long double>::type type;
};

} // namespace internal

template <typename Integer_, std::uintmax_t Divider>
class FixedPointNumber{
 public:
  typedef Integer_ Integer;
  typedef typename internal::NextBiggerNumberTypeBySize<sizeof(Integer)>::type NextBiggerType;

  inline constexpr static std::uintmax_t getDivider(){ return Divider; }


  inline FixedPointNumber() = default;
  inline FixedPointNumber(const FixedPointNumber & other) = default;

  inline FixedPointNumber(Integer p) : _p(p){}
  inline FixedPointNumber(double const & other) : _p(other * getDivider()) {}

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
  inline operator Integer() const { return getNumerator(); }
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
  inline bool operator < (const FixedPointNumber & other) const {
    return _p < other._p;
  }
  inline bool operator > (const FixedPointNumber & other) const {
    return _p > other._p;
  }
  inline bool operator <= (const FixedPointNumber & other) const {
    return _p <= other._p;
  }
  inline bool operator >= (const FixedPointNumber & other) const {
    return _p >= other._p;
  }

  friend std::ostream & operator << (std::ostream & o, const FixedPointNumber & v){
    o << typename std::enable_if<(sizeof(v) > 0), char>::type('[') << v.getNumerator() << " / " << v.getDenominator() << ']';
    return o;
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
