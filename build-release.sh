#!/usr/bin/env bash
# The four files of a release, in dist/release/: client and server for Windows and for Linux, each standing on its own.
# The Linux client has SDL3 and libepoxy linked in and its models inside; it needs only what any desktop has.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"

./build-windows.sh

# The same two libraries as for Windows, from the same downloads.
SDL_VERSION=$(sed -n 's/^SDL_VERSION=//p' build-windows.sh)
EPOXY_VERSION=$(sed -n 's/^EPOXY_VERSION=//p' build-windows.sh)
DEPS=$PWD/build-windows/deps
ROOT=$PWD/build-release
PREFIX=$ROOT/prefix
mkdir -p "$PREFIX" dist/release
PIN="-include $PWD/cmake/glibc-pin.h"

if [[ ! -f $PREFIX/lib/libSDL3.a ]]; then
  echo "== SDL3 für Linux =="
  cmake -S "$DEPS/SDL3-$SDL_VERSION" -B "$ROOT/sdl" -G Ninja -DCMAKE_C_FLAGS="$PIN" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" -DCMAKE_INSTALL_LIBDIR=lib \
    -DSDL_SHARED=OFF -DSDL_STATIC=ON -DSDL_TESTS=OFF -DSDL_TEST_LIBRARY=OFF -DSDL_EXAMPLES=OFF > /dev/null
  cmake --build "$ROOT/sdl" --target install > /dev/null
fi
if [[ ! -f $PREFIX/lib/libepoxy.a ]]; then
  echo "== libepoxy für Linux =="
  rm -rf "$ROOT/epoxy"
  CFLAGS="$PIN" meson setup "$ROOT/epoxy" "$DEPS/libepoxy-$EPOXY_VERSION" --prefix "$PREFIX" --libdir lib --buildtype release --default-library static -Dtests=false > /dev/null
  meson install -C "$ROOT/epoxy" > /dev/null
fi

echo "== Spiel für Linux =="
cmake -S . -B "$ROOT/game" -G Ninja -DCMAKE_CXX_FLAGS="$PIN" -DCMAKE_BUILD_TYPE=Release -DBEANNG_RELEASE=ON -DBEANNG_ARCH=x86-64-v2 \
  -DCMAKE_PREFIX_PATH="$PREFIX" -DBEANNG_EMBED_DIR="$PWD/build-windows/embed" > /dev/null
cmake --build "$ROOT/game" --target beanng beanng-server

cp dist/BeanNG.exe dist/release/BeanNG-Client-Windows.exe
cp dist/BeanNG-Server.exe dist/release/BeanNG-Server-Windows.exe
cp "$ROOT/game/beanng" dist/release/BeanNG-Client-Linux
cp "$ROOT/game/beanng-server" dist/release/BeanNG-Server-Linux
strip dist/release/BeanNG-Client-Linux dist/release/BeanNG-Server-Linux
ls -la dist/release
