#include "aruco_pos_sensor.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <chrono>
#include <iostream>

namespace image {

void ArucoPosSensor::subscribe(
    std::shared_ptr<position::IPosObserver> observer) {
    subsribers.push_back(observer);
}

void ArucoPosSensor::update(const camera::ImageView &image) {
    auto t1{std::chrono::steady_clock::now()};
    auto input_image =
        cv::Mat(image.height, image.width, CV_8UC3, image.buffer);
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
    auto params = cv::aruco::DetectorParameters::create();

    auto dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
    cv::aruco::detectMarkers(input_image, dictionary, marker_corners,
                             marker_ids, params);
    // auto output_image = input_image.clone();
    // cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
    // cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    // cv::imshow("Display window", output_image);
    // cv::imwrite("test.jpg", input_image);
    // cv::waitKey(1);
    auto t2{std::chrono::steady_clock::now()};
    std::cout << "dt "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                     .count()
              << "\n";
}

} // namespace image