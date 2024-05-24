#include "processor.h"

Processor::Processor() : aruco{std::make_shared<image::ArucoPosSensor>()} {
    camera.subscribe(aruco);
}

void Processor::run() {
    while (true) {
    }
}

int main() {
    try {
        Processor proc;
        proc.run();
    } catch (std::exception &e) {
        std::cout << e.what() << std::endl;
    }
}