#pragma once
#include "camera/image_observer.h"
#include "position/pos_observer.h"
#include "position/pos_sensor.h"
#include <opencv2/aruco.hpp>

namespace image {

class ArucoPosSensor : public position::IPosSensor,
                       public camera::IImageObserver {
  public:
    void subscribe(std::shared_ptr<position::IPosObserver> observer) override;
    void update(const camera::ImageView &image) override;

  private:
    std::vector<std::shared_ptr<position::IPosObserver>> subsribers;
};

} // namespace image