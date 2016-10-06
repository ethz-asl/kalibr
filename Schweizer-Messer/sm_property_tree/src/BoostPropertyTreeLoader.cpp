#include <functional>
#include <set>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <sm/BoostPropertyTreeLoader.hpp>

namespace sm {

std::vector<std::string> splitCommaSeparatedList(const std::string& commaSeparatedList) {
  std::vector<std::string> strs;
  boost::split(strs, commaSeparatedList, boost::is_any_of(","));
  return strs;
}

struct UpdateablePropertyTree {
  sm::BoostPropertyTree pt;
  bool confIsSet = false;
  BoostPropertyTreeLoader & ptl;

  UpdateablePropertyTree(BoostPropertyTreeLoader & ptl) : ptl(ptl) {}

  void updateOrSet(sm::BoostPropertyTree newPt, std::string name, bool updateOnly) {
    if (confIsSet) {
      ptl.goingToMergeIn(name);
      pt.update(newPt, !updateOnly);
    } else {
      if(updateOnly) {
        ptl.warn(std::string("Ignoring '!' at the end of ") + name + " because there is nothing to update!");
      }
      ptl.startingWith(name);
      pt = newPt;
      confIsSet = true;
    }
  }

  void loadConfigFileWithParents(std::string configFile, std::set<std::string> & ignore, const std::string & relativeToFile) {
    const size_t lastIndex = configFile.size() - 1;
    const bool updateOnly = configFile[lastIndex] == '!';
    if (updateOnly) {
      configFile = configFile.substr(0, lastIndex);
    }

    std::string configFileFullPath = ptl.resolveFullFilePath(configFile, relativeToFile);

    sm::BoostPropertyTree pt;
    pt.loadXml(configFileFullPath);

    std::string parents = pt.getString("parents", "");
    if (parents.empty()) {
      auto children = pt.getChildren();
      if(!children.empty()) {
        parents = children.front().pt.getString("parents", "");
      }
    }

    if (!parents.empty()) {
      readFile(splitCommaSeparatedList(parents), ignore, configFile);
    }
    updateOrSet(pt, configFileFullPath, updateOnly);
  }

  void readFile(const std::vector<std::string> & configFiles, std::set<std::string> & ignore, const std::string & relativeToFile) {
    for(auto configFile : configFiles) {
      if(configFile.empty()) continue;
      if(ignore.count(configFile)) continue;
      ignore.insert(configFile);

      loadConfigFileWithParents(configFile, ignore, relativeToFile);
    }
  }
};

void BoostPropertyTreeLoader::goingToMergeIn(const std::string& updatePath) {
   std::cout << "Updating with " << updatePath << "." << std::endl;
}

void BoostPropertyTreeLoader::startingWith(const std::string& path) {
  std::cout << "Starting loading with " << path << "." << std::endl;
}

void BoostPropertyTreeLoader::warn(const std::string& message){
  std::cerr << message << std::endl;
}

sm::BoostPropertyTree BoostPropertyTreeLoader::readFiles(const std::vector<std::string>& configFiles, std::set<std::string>& ignore) {
  UpdateablePropertyTree upt(*this);
  upt.readFile(configFiles, ignore, "");
  return upt.pt;
}

sm::BoostPropertyTree BoostPropertyTreeLoader::readFilesAndMergeIntoPropertyTree(const std::string& configFileList) {
  return readFilesAndMergeIntoPropertyTree(splitCommaSeparatedList(configFileList));
}

sm::BoostPropertyTree BoostPropertyTreeLoader::readFilesAndMergeIntoPropertyTree(const std::vector<std::string>& configFiles) {
  std::set<std::string> ignore;
  return readFiles(configFiles, ignore);
}

bool BoostPropertyTreeLoader::fileExists(const std::string& path) const {
  return boost::filesystem::exists(path);
}

std::string BoostPropertyTreeLoader::resolveFullFilePath(const std::string& path, const std::string& relativeToFile) {
  if (getSearchPaths().empty() || path[0] == '/') {
    return path;
  } else if (boost::starts_with(path, "./") || boost::starts_with(path, "../")) {
    if (relativeToFile.empty() && fileExists(path)) {
      return path;
    } else {
      std::string dir = boost::filesystem::path(relativeToFile).remove_filename().string();
      std::string candidate = dir + '/' + path;
      if (fileExists(candidate)) {
        return candidate;
      }
    }
  }

  std::string candidate;
  for (auto& bp : getSearchPaths()) {
    if (getFileNameInfixes().empty()) {
      candidate = bp + "/" + path;
      if (fileExists(candidate)) {
        return candidate;
      }
    } else {
      for (auto& infix : getFileNameInfixes()) {
        candidate = bp + "/" + (infix.empty() ? "" : infix + "/") + path;
        if (fileExists(candidate)) {
          return candidate;
        }
      }
    }
  }
  return getSearchPaths().front() + "/" + (getFileNameInfixes().empty() ? "" : getFileNameInfixes().front() + "/") + path;
}

} // namespace sm
