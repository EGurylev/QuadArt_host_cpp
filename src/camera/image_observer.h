#pragma once
#include <cstddef>
#include <stdint.h>

namespace camera {

struct ImageView {
    uint8_t *buffer{};
    std::size_t width{};
    std::size_t height{};
};

class IImageObserver {
  public:
    virtual void update(const ImageView &image) = 0;
    virtual ~IImageObserver() = default;
};

} // namespace camera