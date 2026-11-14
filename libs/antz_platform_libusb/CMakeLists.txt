# antz_platform_libusb: the libusb transport for a Garmin handheld over USB.
#
# antz_garmin_usb_libusb is the logic: which device, which endpoints, what to undo,
# how a write and a read behave, over a table of libusb calls. It needs no libusb
# and is tested with a fake table. The real binding to libusb 1.0 is a separate
# target, antz_garmin_usb_libusb_real, built only where libusb is found.
add_library(antz_garmin_usb_libusb STATIC src/garmin_usb_libusb.cpp src/garmin_usb_libusb.h)
target_include_directories(antz_garmin_usb_libusb PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(antz_garmin_usb_libusb PUBLIC antz_core)
target_compile_features(antz_garmin_usb_libusb PUBLIC cxx_std_17)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(LIBUSB QUIET IMPORTED_TARGET libusb-1.0)
endif()
if(TARGET PkgConfig::LIBUSB)
    add_library(antz_garmin_usb_libusb_real STATIC src/garmin_usb_libusb_ops.cpp)
    target_link_libraries(antz_garmin_usb_libusb_real PUBLIC antz_garmin_usb_libusb PkgConfig::LIBUSB)
    message(STATUS "Garmin USB libusb transport: real binding built (libusb ${LIBUSB_VERSION})")
else()
    message(STATUS "Garmin USB libusb transport: libusb-1.0 not found, the real binding is not built")
endif()
