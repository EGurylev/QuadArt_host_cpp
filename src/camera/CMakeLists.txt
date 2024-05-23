find_package(pylon 7.4.0 REQUIRED)

add_library(camera basler_camera.cpp)

target_link_libraries(camera pylon::pylon)

set_target_properties(camera PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

target_compile_options(camera PRIVATE
        -Wall -Wextra -pedantic -Werror
)