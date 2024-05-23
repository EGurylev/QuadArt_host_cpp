#pragma once
#include "camera/basler_camera.h"
#include "image/aruco_pos_sensor.h"

class Processor {
  public:
    Processor();
    void run();

  private:
    std::shared_ptr<image::ArucoPosSensor> aruco;
    camera::BaslerCamera camera;
};