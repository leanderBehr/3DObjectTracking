#pragma once

#include <m3t/subscriber.h>

namespace m3t {
template <class F>
class LambdaSubscriber : public Subscriber {
 public:
  template <class CF>
  LambdaSubscriber(std::string const& name, CF&& func)
      : Subscriber(name), func_(std::forward<CF>(func)) {}

  bool SetUp() override {
    set_up_ = true;
    return true;
  }

  bool UpdateSubscriber(int iteration) override {
    func_(iteration);
    return true;
  }

 private:
  F func_;
};

template <class F>
LambdaSubscriber(std::string const&, F) -> LambdaSubscriber<F>;

}  // namespace m3t