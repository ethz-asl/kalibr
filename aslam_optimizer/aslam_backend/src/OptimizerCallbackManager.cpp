#include <aslam/backend/OptimizerCallbackManager.hpp>
#include <map>
#include <vector>
#include <algorithm>

namespace aslam {
namespace backend {
namespace callback {

class RegistryData{
 public:
  std::map<Occasion, std::vector<OptimizerCallback>> callbacks;
};

Registry::Registry() {
  data = new RegistryData();
}

Registry::~Registry() {
  delete data;
}

void Registry::add(std::initializer_list<Occasion> occasions, const OptimizerCallback & callback) {
  for(auto occasion : occasions){
    data->callbacks[occasion].push_back(callback);
  }
}
void Registry::remove(std::initializer_list<Occasion> occasions, const OptimizerCallback & callback) {
  for(auto occasion : occasions){
    std::vector<OptimizerCallback> & vec = data->callbacks[occasion];
    auto p = std::remove(vec.begin(), vec.end(), callback);
    vec.erase(p, vec.end());
  }
}
void Registry::clear() {
  data->callbacks.clear();
}

void Registry::clear(Occasion occasion) {
  data->callbacks[occasion].clear();
}

std::size_t Registry::numCallbacks(Occasion occasion) const {
  return data->callbacks[occasion].size();
}

ProceedInstruction Manager::issueCallback(const Argument & arg) {
  for(auto & c : data->callbacks[arg.occasion]){
    auto r =  c(arg);
    if(r != ProceedInstruction::CONTINUE){
      return r;
    }
  }
  return ProceedInstruction::CONTINUE;
}

}  // namespace callback
}  // namespace backend
}  // namespace aslam

