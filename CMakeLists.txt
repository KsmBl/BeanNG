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
  src/sim.cpp src/world.cpp src/vehicle.cpp src/xantia.cpp src/props.cpp)
target_link_libraries(core PUBLIC Threads::Threads)

add_executable(beanng-test src/tests.cpp)
target_link_libraries(beanng-test PRIVATE core)

if(EXISTS ${CMAKE_SOURCE_DIR}/src/main.cpp)
  add_executable(beanng src/main.cpp src/render.cpp)
  target_link_libraries(beanng PRIVATE core PkgConfig::SDL3 PkgConfig::EPOXY)
endif()
