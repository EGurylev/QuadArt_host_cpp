#include "processor.h"

Processor::Processor() { camera.subscribe(aruco); }

void Processor::run() {
    while (true) {
    }
}

int main() {
    Processor proc;
    proc.run();
}