add_subdirectory(camera)
add_subdirectory(image)
add_subdirectory(position)

add_executable(quadart processor.cpp)
target_link_libraries(quadart aruco camera)

set_target_properties(quadart PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

target_include_directories(quadart
    PUBLIC "${CMAKE_SOURCE_DIR}/src/")