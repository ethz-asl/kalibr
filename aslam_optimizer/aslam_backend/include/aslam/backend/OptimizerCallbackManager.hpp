#ifndef OPTIMIZERCALLBACKMANAGER_HPP_
#define OPTIMIZERCALLBACKMANAGER_HPP_

#include "OptimizerCallback.hpp"
namespace aslam {
namespace backend {
namespace callback {

class RegistryData;

class Registry {
 public:
  Registry();
  ~Registry();
  template <typename T>
  OptimizerCallback add(std::initializer_list<Occasion> occasions, T callback){
    OptimizerCallback optCallback(callback);
    add(occasions, optCallback);
    return optCallback;
  }

  void add(std::initializer_list<Occasion> occasions, const OptimizerCallback & callback);
  void add(Occasion occasion, const OptimizerCallback & callback){
    add({occasion}, callback);
  }
  void remove(std::initializer_list<Occasion> occasions, const OptimizerCallback & callback);
  void remove(Occasion occasion, const OptimizerCallback & callback){
    remove({occasion}, callback);
  }

  void clear();
  void clear(Occasion occasion);

  std::size_t numCallbacks(Occasion occasion) const;

 private:
  friend class Manager;
  RegistryData * data;
};

class Manager : public Registry {
 public:
  ProceedInstruction issueCallback(const Argument & arg);
};

}  // namespace callback
}  // namespace backend
}  // namespace aslam

#endif /* OPTIMIZERCALLBACKMANAGER_HPP_ */
