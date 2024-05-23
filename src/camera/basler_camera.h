#pragma once
#include "camera.h"
#include "image_observer.h"
#include <thread>
#include <pylon/PylonIncludes.h>

namespace camera {

class BaslerCamera : public ICamera {
  public:
    BaslerCamera();
    void process() override;

    void subscribe(std::shared_ptr<IImageObserver> observer) override;

  private:
    std::vector<std::shared_ptr<IImageObserver>> subscribers;
    Pylon::CInstantCamera camera;
    std::jthread thread;
};

} // namespace camera