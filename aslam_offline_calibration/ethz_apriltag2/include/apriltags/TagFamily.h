#ifndef TAGFAMILY_H
#define TAGFAMILY_H

#include <climits>
#include <cmath>
#include <stdio.h>
#include <vector>
#include <map>

#include "apriltags//TagDetection.h"
using namespace std;

namespace AprilTags {

class TagCodes {
public:
  int bits;
  int minHammingDistance;
  std::vector<unsigned long long> codes;
public:
 TagCodes(int bits, int minHammingDistance,
          const unsigned long long* codesA, int num)
   : bits(bits), minHammingDistance(minHammingDistance),
    codes(codesA, codesA+num) // created vector for all entries of codesA
      {}
};

//! Generic class for all tag encoding families
class TagFamily {
public:
  //! The codes array is not copied internally and so must not be modified externally.
  TagFamily(const TagCodes& tagCodes, const size_t blackBorder);

  void setErrorRecoveryBits(int b);

  void setErrorRecoveryFraction(float v);

  /* if the bits in w were arranged in a d*d grid and that grid was
   * rotated, what would the new bits in w be?
   * The bits are organized like this (for d = 3):
   *
   *  8 7 6       2 5 8      0 1 2
   *  5 4 3  ==>  1 4 7 ==>  3 4 5    (rotate90 applied twice)
   *  2 1 0       0 3 6      6 7 8
   */
  static unsigned long long rotate90(unsigned long long w, int d);

  //! Computes the hamming distance between two unsigned long longs.
  static int hammingDistance(unsigned long long a, unsigned long long b);

  //! How many bits are set in the unsigned long long?
  static unsigned char popCountReal(unsigned long long w);

  static int popCount(unsigned long long w);

  //! Given an observed tag with code 'rCode', try to recover the id.
  /*  The corresponding fields of TagDetection will be filled in. */
  void decode(TagDetection& det, unsigned long long rCode) const;

  //! Prints the hamming distances of the tag codes.
  void printHammingDistances() const;

  //! Numer of pixels wide of the inner black border.
  int blackBorder;

  //! Number of bits in the tag. Must be n^2.
  int bits;

  //! Dimension of tag. e.g. for 16 bits, dimension=4. Must be sqrt(bits).
  int dimension;

  //! Minimum hamming distance between any two codes.
  /*  Accounting for rotational ambiguity? The code can recover
   *  (minHammingDistance-1)/2 bit errors.
   */
  int minimumHammingDistance;

  /* The error recovery value determines our position on the ROC
   * curve. We will report codes that are within errorRecoveryBits
   * of a valid code. Small values mean greater rejection of bogus
   * tags (but false negatives). Large values mean aggressive
   * reporting of bad tags (but with a corresponding increase in
   * false positives).
   */
  int errorRecoveryBits;

  //! The array of the codes. The id for a code is its index.
  std::vector<unsigned long long> codes;

  static const int  popCountTableShift = 12;
  static const unsigned int popCountTableSize = 1 << popCountTableShift;
  static unsigned char popCountTable[popCountTableSize];

  //! Initializes the static popCountTable
  static class TableInitializer {
  public:
    TableInitializer() {
      for (unsigned int i = 0; i < TagFamily::popCountTableSize; i++)
        TagFamily::popCountTable[i] = TagFamily::popCountReal(i);
    }
  } initializer;
};

} // namespace

#endif
