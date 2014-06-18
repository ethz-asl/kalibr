#include <iostream>

#include "apriltags/TagFamily.h"

/**

// example of instantiation of tag family:

#include "apriltags/TagFamily.h"
#include "apriltags/Tag36h11.h"
TagFamily *tag36h11 = new TagFamily(tagCodes36h11);

// available tag families:

#include "apriltags/Tag16h5.h"
#include "apriltags/Tag16h5_other.h"
#include "apriltags/Tag25h7.h"
#include "apriltags/Tag25h9.h"
#include "apriltags/Tag36h11.h"
#include "apriltags/Tag36h11_other.h"
#include "apriltags/Tag36h9.h"

*/


namespace AprilTags {

TagFamily::TagFamily(const TagCodes& tagCodes, const size_t blackBorder)
  : blackBorder(blackBorder), bits(tagCodes.bits), dimension((int)std::sqrt((float)bits)),
    minimumHammingDistance(tagCodes.minHammingDistance),
    errorRecoveryBits(1), codes() {
  if ( bits != dimension*dimension )
    cerr << "Error: TagFamily constructor called with bits=" << bits << "; must be a square number!" << endl;
  codes = tagCodes.codes;
}

void TagFamily::setErrorRecoveryBits(int b) {
  errorRecoveryBits = b;
}

void TagFamily::setErrorRecoveryFraction(float v) {
  errorRecoveryBits = (int) (((int) (minimumHammingDistance-1)/2)*v);
}

unsigned long long TagFamily::rotate90(unsigned long long w, int d) {
  unsigned long long wr = 0;
  const unsigned long long oneLongLong = 1;

  for (int r = d-1; r>=0; r--) {
    for (int c = 0; c<d; c++) {
      int b = r + d*c;
      wr = wr<<1;
      
      if ((w & (oneLongLong<<b)) != 0)
	wr |= 1;
    }
  }
  return wr;
}

int TagFamily::hammingDistance(unsigned long long a, unsigned long long b) {
  return popCount(a^b);
}

unsigned char TagFamily::popCountReal(unsigned long long w) {
  unsigned char cnt = 0;
  while (w != 0) {
    w &= (w-1);
    ++cnt;
  }
  return cnt;
}

int TagFamily::popCount(unsigned long long w) {
  int count = 0;
  while (w != 0) {
    count += popCountTable[(unsigned int) (w & (popCountTableSize-1))];
    w >>= popCountTableShift;
  }
  return count;
}

void TagFamily::decode(TagDetection& det, unsigned long long rCode) const {
  int  bestId = -1;
  int  bestHamming = INT_MAX;
  int  bestRotation = 0;
  unsigned long long bestCode = 0;

  unsigned long long rCodes[4];
  rCodes[0] = rCode;
  rCodes[1] = rotate90(rCodes[0], dimension);
  rCodes[2] = rotate90(rCodes[1], dimension);
  rCodes[3] = rotate90(rCodes[2], dimension);

  for (unsigned int id = 0; id < codes.size(); id++) {
    for (unsigned int rot = 0; rot < 4; rot++) {
      int thisHamming = hammingDistance(rCodes[rot], codes[id]);
      if (thisHamming < bestHamming) {
	bestHamming = thisHamming;
	bestRotation = rot;
	bestId = id;
	bestCode = codes[id];
      }
    }
  }
  det.id = bestId;
  det.hammingDistance = bestHamming;
  det.rotation = bestRotation;
  det.good = (det.hammingDistance <= errorRecoveryBits);
  det.obsCode = rCode;
  det.code = bestCode;
}

void TagFamily::printHammingDistances() const {
  vector<int> hammings(dimension*dimension+1);
  for (unsigned i = 0; i < codes.size(); i++) {
    unsigned long long r0 = codes[i];
    unsigned long long r1 = rotate90(r0, dimension);
    unsigned long long r2 = rotate90(r1, dimension);
    unsigned long long r3 = rotate90(r2, dimension);
    for (unsigned int j = i+1; j < codes.size(); j++) {
      int d = min(min(hammingDistance(r0, codes[j]),
		      hammingDistance(r1, codes[j])),
		  min(hammingDistance(r2, codes[j]),
		      hammingDistance(r3, codes[j])));
      hammings[d]++;
    }
  }

  for (unsigned int i = 0; i < hammings.size(); i++)
    printf("hammings: %u = %d\n", i, hammings[i]);
}

unsigned char TagFamily::popCountTable[TagFamily::popCountTableSize];

TagFamily::TableInitializer TagFamily::initializer;

} // namespace
