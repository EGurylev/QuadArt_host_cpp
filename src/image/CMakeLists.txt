find_package(OpenCV REQUIRED)

add_library(aruco aruco_pos_sensor.cpp)
target_link_libraries(aruco ${OpenCV_LIBS})

target_include_directories(aruco
    PUBLIC "${CMAKE_SOURCE_DIR}/src/" ${OpenCV_INCLUDE_DIRS})

set_target_properties(aruco PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)