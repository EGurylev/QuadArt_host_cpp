#include "basler_camera.h"

namespace camera {

BaslerCamera::BaslerCamera() {
    thread = std::jthread(&BaslerCamera::process, this);
}

void BaslerCamera::subscribe(std::shared_ptr<IImageObserver> observer) {
    subscribers.push_back(observer);
}

void BaslerCamera::process() {
    using namespace Pylon;

    Pylon::PylonAutoInitTerm autoInitTerm;
    camera.Attach(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
    camera.StartGrabbing(Pylon::GrabStrategy_OneByOne);

    CGrabResultPtr ptrGrabResult;
    std::cout << "Using device " << camera.GetDeviceInfo().GetModelName()
              << std::endl;

    while (camera.IsGrabbing()) {
        camera.RetrieveResult(5000, ptrGrabResult,
                              TimeoutHandling_ThrowException);
        if (ptrGrabResult->GrabSucceeded()) {
            // Access the image data.
            std::cout << "SizeX: " << ptrGrabResult->GetWidth() << std::endl;
            std::cout << "SizeY: " << ptrGrabResult->GetHeight() << std::endl;
            uint8_t *image_buffer = (uint8_t *)ptrGrabResult->GetBuffer();
            std::cout << "Gray value of first pixel: "
                      << (uint32_t)image_buffer[0] << std::endl
                      << std::endl;
            for (auto &subsciber : subscribers) {
                subsciber->update(ImageView{image_buffer,
                                            ptrGrabResult->GetWidth(),
                                            ptrGrabResult->GetHeight()});
            }
        }
    }
}

} // namespace camera