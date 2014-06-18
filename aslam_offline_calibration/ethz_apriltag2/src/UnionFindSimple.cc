#include "apriltags/UnionFindSimple.h"
#include <iostream>

namespace AprilTags {

int UnionFindSimple::getRepresentative(int thisId) {
  // terminal case: a node is its own parent
  if (data[thisId].id == thisId)
    return thisId;

  // otherwise, recurse...
  int root = getRepresentative(data[thisId].id);

  // short circuit the path
  data[thisId].id = root;

  return root;
}

void UnionFindSimple::printDataVector() const {
  for (unsigned int i = 0; i < data.size(); i++)
    std::cout << "data[" << i << "]: " << " id:" << data[i].id << " size:" << data[i].size << std::endl;
}

int UnionFindSimple::connectNodes(int aId, int bId) {
  int aRoot = getRepresentative(aId);
  int bRoot = getRepresentative(bId);

  if (aRoot == bRoot)
    return aRoot;

  int asz = data[aRoot].size;
  int bsz = data[bRoot].size;

  if (asz > bsz) {
    data[bRoot].id = aRoot;
    data[aRoot].size += bsz;
    return aRoot;
  } else {
    data[aRoot].id = bRoot;
    data[bRoot].size += asz;
    return bRoot;
  }
}

void UnionFindSimple::init() {
  for (unsigned int i = 0; i < data.size(); i++) {
    // everyone is their own cluster of size 1
    data[i].id = i;
    data[i].size = 1;
  }
}

} // namespace
