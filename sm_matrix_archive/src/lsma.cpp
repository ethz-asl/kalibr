#include <iostream>
#include <sm/MatrixArchive.hpp>

void usage(const char * cmd) {
  std::cerr << "USAGE: " << cmd << " <PATH_TO_MATRIX_ARCHIVE>" << std::endl;
}

int main(int argc, char **argv) {
  if(argc != 2){
    usage(argv[0]);
    return -1;
  }
  using sm::MatrixArchive;
  MatrixArchive ma;
  ma.load(std::string(argv[1]));

  for (auto & s : ma.getStrings()){
    std::cout << s.first << " : " << s.second << std::endl;
  }
  for (auto & m : ma){
    std::cout << m.first << " :\n" << m.second << std::endl;
  }
  return 0;
}
