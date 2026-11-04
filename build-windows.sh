#!/usr/bin/env bash
# Builds the whole game into one Windows program, dist/BeanNG.exe, from Linux with MinGW-w64.
# On Arch:  pacman -S mingw-w64-gcc cmake ninja meson python curl
# SDL3 and libepoxy are downloaded once and linked in statically: the .exe needs nothing else, and carries its models too.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"

SDL_VERSION=3.4.18
SDL_SHA256=9c75cf16330322c217dedd2e0609f1124f1b54b8633e763467b4684d0f4334a3
EPOXY_VERSION=1.5.10
EPOXY_SHA256=a7ced37f4102b745ac86d6a70a9da399cc139ff168ba6b8002b4d8d43c900c15

for tool in x86_64-w64-mingw32-g++ cmake ninja meson python3 curl; do
  command -v "$tool" > /dev/null || { echo "$tool fehlt. Auf Arch: pacman -S mingw-w64-gcc cmake ninja meson python curl" >&2; exit 1; }
done

ROOT=$PWD/build-windows
DEPS=$ROOT/deps
PREFIX=$ROOT/prefix
mkdir -p "$DEPS" "$PREFIX" dist

fetch() {  # url, file, sha256: unpacks into deps/ unless it is there already
  local url=$1 file=$DEPS/$2 sum=$3
  [[ -f $file ]] || curl -fsSL -o "$file" "$url"
  echo "$sum  $file" | sha256sum -c --quiet -
  tar -xzf "$file" -C "$DEPS"
}

if [[ ! -f $PREFIX/lib/libSDL3.a ]]; then
  echo "== SDL3 $SDL_VERSION =="
  fetch "https://github.com/libsdl-org/SDL/releases/download/release-$SDL_VERSION/SDL3-$SDL_VERSION.tar.gz" sdl.tar.gz "$SDL_SHA256"
  cmake -S "$DEPS/SDL3-$SDL_VERSION" -B "$DEPS/sdl-build" -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/mingw-w64.cmake" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DSDL_SHARED=OFF -DSDL_STATIC=ON -DSDL_TESTS=OFF -DSDL_TEST_LIBRARY=OFF -DSDL_EXAMPLES=OFF > /dev/null
  cmake --build "$DEPS/sdl-build" --target install > /dev/null
fi

if [[ ! -f $PREFIX/lib/libepoxy.a ]]; then
  echo "== libepoxy $EPOXY_VERSION =="
  fetch "https://github.com/anholt/libepoxy/archive/refs/tags/$EPOXY_VERSION.tar.gz" epoxy.tar.gz "$EPOXY_SHA256"
  cat > "$DEPS/cross.ini" <<CROSS
[binaries]
c = 'x86_64-w64-mingw32-gcc'
ar = 'x86_64-w64-mingw32-ar'
strip = 'x86_64-w64-mingw32-strip'
windres = 'x86_64-w64-mingw32-windres'
[host_machine]
system = 'windows'
cpu_family = 'x86_64'
cpu = 'x86_64'
endian = 'little'
CROSS
  rm -rf "$DEPS/epoxy-build"
  meson setup "$DEPS/epoxy-build" "$DEPS/libepoxy-$EPOXY_VERSION" --cross-file "$DEPS/cross.ini" --prefix "$PREFIX" --libdir lib \
    --buildtype release --default-library static -Dtests=false -Degl=no -Dglx=no -Dx11=false > /dev/null
  meson install -C "$DEPS/epoxy-build" > /dev/null
fi

echo "== Modelle =="
tools/gen_assets.sh
python3 tools/gen_embed.py "$ROOT/embed" assets/*.mesh assets/ui.atlas

echo "== Spiel =="
cmake -S . -B "$ROOT/game" -G Ninja -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/mingw-w64.cmake" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_FIND_ROOT_PATH="$PREFIX" -DBEANNG_EMBED_DIR="$ROOT/embed" > /dev/null
cmake --build "$ROOT/game" --target beanng

cp "$ROOT/game/beanng.exe" dist/BeanNG.exe
x86_64-w64-mingw32-strip dist/BeanNG.exe
echo "Fertig: dist/BeanNG.exe ($(du -h dist/BeanNG.exe | cut -f1))"
