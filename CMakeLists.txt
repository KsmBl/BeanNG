cmake_minimum_required(VERSION 3.20)
project(BeanNG CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=x86-64-v3 -fno-math-errno -fno-trapping-math -flto=auto")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter -fno-exceptions -fno-rtti")

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL3 REQUIRED IMPORTED_TARGET sdl3)
pkg_check_modules(EPOXY REQUIRED IMPORTED_TARGET epoxy)
find_package(Threads REQUIRED)

# Physics core: no graphics dependency, so the headless test bench links only this.
add_library(core STATIC
  src/sim.cpp src/world.cpp src/vehicle.cpp src/xantia.cpp src/one1.cpp src/hummer.cpp src/tank.cpp src/props.cpp src/carmesh.cpp src/net.cpp src/duel.cpp)
target_link_libraries(core PUBLIC Threads::Threads)

add_executable(beanng-test src/tests.cpp)
target_link_libraries(beanng-test PRIVATE core)

# The duel server: physics and network, nothing else, so it runs on a machine without a screen.
add_executable(beanng-server src/server.cpp)
target_link_libraries(beanng-server PRIVATE core)

if(EXISTS ${CMAKE_SOURCE_DIR}/src/main.cpp)
  add_executable(beanng src/main.cpp src/render.cpp src/audio.cpp)
  target_link_libraries(beanng PRIVATE core PkgConfig::SDL3 PkgConfig::EPOXY)
endif()
