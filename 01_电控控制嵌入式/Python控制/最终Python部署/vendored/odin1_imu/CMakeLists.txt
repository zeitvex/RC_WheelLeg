cmake_minimum_required(VERSION 3.16)
project(odin1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(PkgConfig REQUIRED)
find_package(OpenSSL REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_library(odin1_imu_bridge SHARED
    src/odin1_imu_bridge.cpp
)

target_include_directories(odin1_imu_bridge
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${LIBUSB_INCLUDE_DIRS}
)

target_link_directories(odin1_imu_bridge
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

target_link_libraries(odin1_imu_bridge
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/liblydHostApi_arm.a
        ${LIBUSB_LIBRARIES}
        OpenSSL::SSL
        OpenSSL::Crypto
        pthread
        rt
        dl
)
