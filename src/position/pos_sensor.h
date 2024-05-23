#pragma once
#include "pos_observer.h"
#include <memory>

namespace position {

class IPosSensor {
  public:
    virtual void subscribe(std::shared_ptr<IPosObserver> observer) = 0;
    virtual ~IPosSensor() = default;
};

} // namespace position