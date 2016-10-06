#ifndef HASH_ID_HPP_
#define HASH_ID_HPP_

#include <chrono>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <random>
#include <mutex>

namespace sm {

constexpr bool kIs64BitArch = (sizeof(void*) == 8);

struct HashPrimeAndBase {
  static constexpr std::size_t kPrime =
      kIs64BitArch ? 1099511628211ull : 16777619u;
  static constexpr std::size_t kOffsetBasis =
      kIs64BitArch ? 14695981039346656037ull : 2166136261u;
};

/**
 * 128 bit hash. Can be used as key to unordered containers.
 */
class HashId {
 public:
  inline HashId() {
    setInvalid();
  }
  inline HashId(const HashId& other){
    *this = other;
  }
  inline HashId(const uint64_t source[2]) {
    fromUint64(source);
  }

  inline void fromUint64(const uint64_t source[2]) {
    memcpy(reinterpret_cast<void*>(&val_.u64),
           reinterpret_cast<const void*>(source), sizeof(val_.u64));
  }

  inline void toUint64(uint64_t destination[2]) const {
    memcpy(reinterpret_cast<void*>(destination),
           reinterpret_cast<const void*>(&val_.u64), sizeof(val_.u64));
  }

  /**
   * Generates a random Hash ID seeded from the nanosecond time of the first
   * call of this function
   */
  inline static HashId random() {
    HashId generated;
    generated.randomize();
    return generated;
  }

  /**
   * Returns hexadecimal string for debugging or serialization
   */
  inline const std::string hexString() const {
    char buffer[2*sizeof(val_) + 1]; // 1 for the \0 character
    buffer[2*sizeof(val_)] = '\0';
    for (size_t i = 0; i < sizeof(val_); ++i){
      buffer[2 * i + 1] = kHexConversion[val_.c[i] & 0xf];
      buffer[2 * i] = kHexConversion[val_.c[i] >> 4];
    }
    return std::string(buffer);
  }

  /**
   * Deserialize from hexadecimal string. Serialization and Deserialization
   * could be made more performant by using blobs.
   */
  inline bool fromHexString(const std::string& hexString) {
    // hexadecimal string takes 2 characters per byte
    if (hexString.size() != 2*sizeof(val_)){
      return false;
    }
    for (size_t i = 0; i < sizeof(val_); ++i){
      val_.c[i] = static_cast<unsigned char>(
          stoul(std::string(hexString, 2*i, 2), 0, 16));
    }
    return true;
  }

  /**
   * Rehashes the 128 bit hash to a 32/64 bit hash that can be used in STL
   * containers. This means that we will get collisions on the buckets, which
   * is fine since we can disambiguate the actual hashes using operator==.
   * So this does not increase the probability of ID collision.
   */
  inline size_t hashToSizeT() const {
    size_t hash = HashPrimeAndBase::kOffsetBasis;
    hash ^= val_.u64[0];
    hash *= HashPrimeAndBase::kPrime;
    hash ^= val_.u64[1];
    hash *= HashPrimeAndBase::kPrime;
    return hash;
  }
  /**
   * Rehashes the 128 bit hash to a 32/64 bit hash that can be used in STL
   * containers. This means that we will get collisions on the buckets, which
   * is fine since we can disambiguate the actual hashes using operator==.
   * So this does not increase the probability of ID collision.
   * Version that skips prime multiplication for seeds that are already well
   * distributed.
   */
  inline size_t hashToSizeTFast() const {
    size_t hash = HashPrimeAndBase::kOffsetBasis;
    hash ^= val_.u64[0];
    hash ^= val_.u64[1];
    return hash;
  }

  /**
   * Randomizes to ID seeded from the nanosecond time of the first
   * call of this function
   */
  inline void randomize(){
    static std::mt19937_64 rng(time64());
    static std::mutex m_rng;
    {
      std::unique_lock<std::mutex> lock(m_rng);
      val_.u64[0] = rng();
      val_.u64[1] = rng();
    }
  }

  inline void operator =(const HashId& other) {
    memcpy(&val_, &other.val_, sizeof(val_));
  }

  inline bool operator <(const HashId& other) const {
    if (val_.u64[0] == other.val_.u64[0]){
      return val_.u64[1] < other.val_.u64[1];
    }
    return val_.u64[0] < other.val_.u64[0];
  }
  inline bool operator >(const HashId& other) const {
    if (val_.u64[0] == other.val_.u64[0]){
      return val_.u64[1] > other.val_.u64[1];
    }
    return val_.u64[0] > other.val_.u64[0];
  }
  inline bool operator ==(const HashId& other) const {
    return val_.u64[0] == other.val_.u64[0] && val_.u64[1] == other.val_.u64[1];
  }
  inline bool operator !=(const HashId& other) const{
    return !(*this == other);
  }

  /**
   * Invalidation mechanism
   */
  inline void setInvalid() {
    memset(&val_, 0, sizeof(val_));
  }
  inline bool isValid() const {
    return val_.u64[0] != 0 || val_.u64[1] != 0;
  }

 private:
  /**
   * Time seed from nanoseconds. Covers 584 years if we assume no two agents
   * initialize in the same nanosecond.
   */
  inline static int64_t time64() {
    std::chrono::high_resolution_clock::duration current =
        std::chrono::high_resolution_clock::now().time_since_epoch();
    using std::chrono::duration_cast;
    using std::chrono::nanoseconds;
    // count() specified to return at least 64 bits
    return duration_cast<nanoseconds>(current).count();
  }

  /**
   * Internal representation
   */
  union HashVal {
    unsigned char c[16];
    uint64_t u64[2];
  };
  HashVal val_;

  static const char kHexConversion[];
};

} // namespace sm

namespace std{

inline ostream& operator<<(ostream& out, const sm::HashId& hash) {
  out << hash.hexString();
  return out;
}

template<>
struct hash<sm::HashId>{
  typedef sm::HashId argument_type;
  typedef std::size_t value_type;

  value_type operator()(const argument_type& hash_id) const {
    return hash_id.hashToSizeT();
  }
};
} // namespace std


/// \brief Define the hash function for types derived from HashId
#define SM_DEFINE_HASHID_HASH(FullyQualifiedIdTypeName)                 \
    namespace std {                                                       \
  template<>                                                            \
  struct hash<FullyQualifiedIdTypeName>{                                \
      typedef FullyQualifiedIdTypeName argument_type;                     \
      typedef std::size_t value_type;                                     \
      value_type operator()(const argument_type& hash_id) const {         \
        return hash_id.hashToSizeT();                                     \
      }                                                                   \
    };                                                                    \
}  /* namespace std */                                                \
    extern void DefineIDHash ## __FILE__ ## __LINE__(void)

#endif /* HASH_ID_HPP_ */
