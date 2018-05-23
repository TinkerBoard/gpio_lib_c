cmake_minimum_required(VERSION 3.9)
project(gpio_lib_c VERSION 1.2.3)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Set common compilation options

set(CMAKE_CXX_STANDARD 11)

add_compile_options(
    -Wall
    -fPIC
)

if(${COVERAGE})
  message("Building with coverege support")
  add_compile_options(-g -O0 -coverage)
  link_libraries(-coverage)
endif()

set(wiringPi_SOURCE 
  ../wiringPi/wiringPi.c
  ../wiringPi/wiringTB.c
  ../wiringPi/wiringSerial.c
  ../wiringPi/wiringShift.c
  ../wiringPi/piHiPri.c
  ../wiringPi/piThread.c
  ../wiringPi/wiringPiSPI.c
  ../wiringPi/wiringPiI2C.c
  ../wiringPi/softPwm.c
  ../wiringPi/softTone.c
  ../wiringPi/mcp23008.c
  ../wiringPi/mcp23016.c
  ../wiringPi/mcp23017.c
  ../wiringPi/mcp23s08.c
  ../wiringPi/mcp23s17.c
  ../wiringPi/sr595.c
  ../wiringPi/pcf8574.c
  ../wiringPi/pcf8591.c
  ../wiringPi/mcp3002.c
  ../wiringPi/mcp3004.c
  ../wiringPi/mcp4802.c
  ../wiringPi/mcp3422.c
  ../wiringPi/max31855.c
  ../wiringPi/max5322.c
  ../wiringPi/sn3218.c
  ../wiringPi/drcSerial.c
  ../wiringPi/wpiExtensions.c
)

set(wiringPi_HEADER 
  ../wiringPi/wiringPi.h
  ../wiringPi/wiringTB.h
  ../wiringPi/RKIO.h
  ../wiringPi/wiringSerial.h
  ../wiringPi/wiringShift.h
  ../wiringPi/wiringPiSPI.h
  ../wiringPi/wiringPiI2C.h
  ../wiringPi/softPwm.h
  ../wiringPi/softTone.h
  ../wiringPi/mcp23008.h
  ../wiringPi/mcp23016.h
  ../wiringPi/mcp23017.h
  ../wiringPi/mcp23s08.h
  ../wiringPi/mcp23s17.h
  ../wiringPi/sr595.h
  ../wiringPi/pcf8574.h
  ../wiringPi/pcf8591.h
  ../wiringPi/mcp3002.h
  ../wiringPi/mcp3004.h
  ../wiringPi/mcp4802.h
  ../wiringPi/mcp3422.h
  ../wiringPi/max31855.h
  ../wiringPi/max5322.h
  ../wiringPi/sn3218.h
  ../wiringPi/drcSerial.h
  ../wiringPi/wpiExtensions.h
)


set(devLib_SOURCE
  ../devLib/ds1302.c
  ../devLib/maxdetect.c
  ../devLib/piNes.c
  ../devLib/gertboard.c
  ../devLib/piFace.c
  ../devLib/lcd128x64.c
  ../devLib/lcd.c
  ../devLib/piGlow.c
)
set(devLib_HEADER
  ../devLib/ds1302.h
  ../devLib/gertboard.h
  ../devLib/lcd128x64.h
  ../devLib/lcd.h
  ../devLib/maxdetect.h
  ../devLib/piFace.h
  ../devLib/piGlow.h
  ../devLib/piNes.h
)

set (gpio_SOURCE 
  ../gpio/gpio.c
  ../gpio/readall.c
  ../gpio/pins.c
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(gpio_lib_c SHARED
  ${wiringPi_SOURCE}  
  ${devLib_SOURCE}  
)

target_include_directories(
    gpio_lib_c PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../wiringPi>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../devLib>"
)

add_executable(gpio
  ${gpio_SOURCE}
)

target_link_libraries(gpio
  gpio_lib_c
  Threads::Threads
)

include(GNUInstallDirs)
set(config_install_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(namespace "${PROJECT_NAME}::")

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${version_config}" COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "cmake/Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
)

install(
  TARGETS
    gpio_lib_c
    gpio
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  )

install(
    FILES ${wiringPi_HEADER} ${devLib_HEADER}
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${namespace}"
    DESTINATION "${config_install_dir}"
)
