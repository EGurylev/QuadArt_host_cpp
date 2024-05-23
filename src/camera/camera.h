#pragma once
#include "image_observer.h"
#include <cstddef>
#include <memory>
#include <stdint.h>

namespace camera {

class ICamera {
  public:
    virtual void process() = 0;
    virtual void subscribe(std::shared_ptr<IImageObserver> observer) = 0;
    virtual ~ICamera() = default;
};

} // namespace camera