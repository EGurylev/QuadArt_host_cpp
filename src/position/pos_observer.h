#pragma once
#include <cstddef>

namespace position {

struct PosMeasurement {
    struct Pose6D {
        double x{};
        double y{};
        double z{};
        double pitch{};
        double roll{};
        double yaw{};
    };

    Pose6D pose;
    std::size_t sensor_id{};
};

class IPosObserver {
  public:
    virtual void update(const PosMeasurement &pos);
    virtual ~IPosObserver() = default;
};

} // namespace position