#include <sm/BoostPropertyTreeSupport.hpp>
#include <sm/assert_macros.hpp>
#include <sm/string_routines.hpp>

#include <boost/filesystem/operations.hpp>
#include <cstdlib>

namespace sm {
  
  /** 
   * @brief Helper to find a file with name filenameToFind in the directory specified by an environment variable envVarNameContainingSearchDir
   **/
  boost::filesystem::path findFile(const std::string& filenameToFind, const std::string& envVarNameContainingSearchDir) {

    boost::filesystem::path fileFound;
    char* dir = std::getenv(envVarNameContainingSearchDir.c_str());
    SM_ASSERT_TRUE( std::runtime_error, dir, "Environment variable " << envVarNameContainingSearchDir << " could not be found");

    std::string searchDirectory = std::string(dir);
    searchDirectory = sm::ensureTrailingBackslash(searchDirectory);

    std::string fullPath = searchDirectory + filenameToFind;
    SM_ASSERT_TRUE( std::runtime_error, boost::filesystem::exists(fullPath),
      "Environment variable " << envVarNameContainingSearchDir << " exists, but file " << filenameToFind << " could not be found in "  << searchDirectory);

    fileFound = boost::filesystem::path(fullPath.c_str());
    return fileFound;
  }

} /* namespace sm */
