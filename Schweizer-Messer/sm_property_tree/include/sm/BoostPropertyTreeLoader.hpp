#ifndef H050B8D6A_0312_469D_90C1_CC9482370CBF
#define H050B8D6A_0312_469D_90C1_CC9482370CBF

#include <sm/BoostPropertyTree.hpp>
#include <set>
#include <vector>

namespace sm {

class BoostPropertyTreeLoader {
 public:
  sm::BoostPropertyTree readFilesAndMergeIntoPropertyTree(const std::string& commaSeparatedFileList);
  sm::BoostPropertyTree readFilesAndMergeIntoPropertyTree(const std::vector<std::string>& files);

  std::string resolveFullFilePath(const std::string& path, const std::string& relativeToFile);

  std::vector<std::string> & getSearchPaths() {
    return searchPaths;
  }

  std::vector<std::string> & getFileNameInfixes() {
    return fileNameInfixes;
  }

  virtual ~BoostPropertyTreeLoader(){}

 protected:
  virtual bool fileExists(const std::string& path) const;
 private:
  virtual void goingToMergeIn(const std::string & updatePath);
  virtual void startingWith(const std::string & path);
  virtual void warn(const std::string & path);


  sm::BoostPropertyTree readFiles(const std::vector<std::string>& configFiles, std::set<std::string>& ignore);

  std::vector<std::string> searchPaths;
  std::vector<std::string> fileNameInfixes;
  friend class UpdateablePropertyTree;
};

} // namespace sm

#endif /* H050B8D6A_0312_469D_90C1_CC9482370CBF */
