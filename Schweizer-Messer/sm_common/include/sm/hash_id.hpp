#ifndef HASH_ID_HPP_
#define HASH_ID_HPP_

#include <chrono>
#include <cstring>
#include <functional>
#include <iomanip>
#include <random>
#include <sstream>

namespace sm {

/**
 * 128 bit hash. Can be used as key to unordered containers.
 */
class HashId {
 public:
  /**
   * Initializes to an invalid Hash
   */
  inline HashId() {
    setInvalid();
  }
  /**
   * Copy constructor
   */
  inline HashId(const HashId& other){
    *this = other;
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
    std::ostringstream ss;
    for (size_t i = 0; i < sizeof(val_); ++i){
      ss << std::hex << std::setfill('0') << std::setw(2) <<
          static_cast<int>(val_.c[i]);
    }
    return ss.str();
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
      unsigned int integerValue;
      std::istringstream ss(std::string(hexString, 2*i, 2));
      ss >> std::hex >> integerValue;
      val_.c[i] = static_cast<unsigned char>(integerValue);
    }
    return true;
  }

  /**
   * Randomizes to ID seeded from the nanosecond time of the first
   * call of this function
   */
  inline void randomize(){
    static std::mt19937_64 rng(time64());
    val_.u64[0] = rng();
    val_.u64[1] = rng();
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
    uint_fast64_t u64[2];
  };
  HashVal val_;
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

  value_type operator()(const argument_type& hashId) const {
    return std::hash<std::string>()(hashId.hexString());
  }
};
} // namespace std


#endif /* HASH_ID_HPP_ */
