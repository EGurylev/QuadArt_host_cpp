#include "aruco_pos_sensor.h"

namespace image {

void ArucoPosSensor::subscribe(
    std::shared_ptr<position::IPosObserver> observer) {
    subsribers.push_back(observer);
}

void ArucoPosSensor::update(const camera::ImageView &image) {
    auto img = cv::Mat(image.height, image.width, CV_8UC3, image.buffer);
}

} // namespace image