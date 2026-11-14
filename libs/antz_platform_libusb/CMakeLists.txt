# antz_platform_libusb: the libusb transport for a Garmin handheld over USB.
#
# antz_garmin_usb_libusb is the logic: which device, which endpoints, what to undo,
# how a write and a read behave, over a table of libusb calls. It needs no libusb
# and is tested with a fake table. The real binding to libusb 1.0 is a separate
# target, antz_garmin_usb_libusb_real, built only where libusb is found.
#
# (Interface libraries until the implementation lands; the sources join then.)
add_library(antz_garmin_usb_libusb INTERFACE)
target_include_directories(antz_garmin_usb_libusb INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(antz_garmin_usb_libusb INTERFACE antz_core)
target_compile_features(antz_garmin_usb_libusb INTERFACE cxx_std_17)
