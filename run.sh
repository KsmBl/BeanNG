#!/usr/bin/env bash
# Builds BeanNG if needed and starts it. Arguments are passed on to the game.
set -euo pipefail

cd "$(dirname "$(readlink -f "$0")")"

if [[ ! -f build/build.ninja ]]; then
  cmake -S . -B build -G Ninja
fi
cmake --build build --target beanng

# The car mesh is generated, and only rebuilt when the generator has changed.
if [[ ! -f assets/xantia.mesh || tools/gen_xantia.py -nt assets/xantia.mesh ]]; then
  mkdir -p assets
  python3 tools/gen_xantia.py assets/xantia.mesh
fi

exec ./build/beanng "$@"
