cmake_minimum_required(VERSION 3.20)
project(BeanNG CXX)
if(WIN32)
  enable_language(ASM)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# On Windows without AVX: it reaches more machines, and GCC there misaligns the stack for 32-byte vectors.
if(WIN32)
  set(BEANNG_ARCH x86-64-v2)
else()
  set(BEANNG_ARCH x86-64-v3)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=${BEANNG_ARCH} -fno-math-errno -fno-trapping-math -flto=auto")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter -fno-exceptions -fno-rtti")

if(NOT WIN32)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(SDL3 REQUIRED IMPORTED_TARGET sdl3)
  pkg_check_modules(EPOXY REQUIRED IMPORTED_TARGET epoxy)
endif()
find_package(Threads REQUIRED)

# Physics core: no graphics dependency, so the headless test bench links only this.
add_library(core STATIC
  src/sim.cpp src/world.cpp src/vehicle.cpp src/xantia.cpp src/one1.cpp src/hummer.cpp src/tank.cpp src/props.cpp src/carmesh.cpp src/net.cpp src/duel.cpp src/asset.cpp src/platform.cpp)
target_link_libraries(core PUBLIC Threads::Threads)

if(WIN32)
  # One program with everything in it: the game, the server behind --server, and the models. Nothing to install.
  find_package(SDL3 REQUIRED CONFIG)
  find_library(EPOXY_LIB NAMES libepoxy.a epoxy REQUIRED)
  find_path(EPOXY_INCLUDE epoxy/gl.h REQUIRED)
  target_link_libraries(core PUBLIC ws2_32 winmm)
  target_compile_definitions(core PRIVATE BEANNG_EMBED_ASSETS)
  add_executable(beanng WIN32 src/main.cpp src/render.cpp src/vegetation.cpp src/audio.cpp src/server.cpp
    ${BEANNG_EMBED_DIR}/embed.S ${BEANNG_EMBED_DIR}/embed_table.cpp)
  set_source_files_properties(src/server.cpp PROPERTIES COMPILE_DEFINITIONS BEANNG_ONE_PROGRAM)
  target_include_directories(beanng PRIVATE ${EPOXY_INCLUDE})
  target_link_libraries(beanng PRIVATE core SDL3::SDL3-static ${EPOXY_LIB})
  # The stack as on Linux, and no DLLs of the compiler's to carry around.
  target_link_options(beanng PRIVATE -static -static-libgcc -static-libstdc++ -Wl,--stack,8388608)
else()
  add_executable(beanng-test src/tests.cpp)
  target_link_libraries(beanng-test PRIVATE core)

  # The duel server: physics and network, nothing else, so it runs on a machine without a screen.
  add_executable(beanng-server src/server.cpp)
  target_link_libraries(beanng-server PRIVATE core)

  if(EXISTS ${CMAKE_SOURCE_DIR}/src/main.cpp)
    add_executable(beanng src/main.cpp src/render.cpp src/vegetation.cpp src/audio.cpp)
    target_link_libraries(beanng PRIVATE core PkgConfig::SDL3 PkgConfig::EPOXY)
  endif()
endif()
