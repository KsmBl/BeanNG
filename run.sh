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

if [[ ! -f assets/leopard.mesh || tools/gen_leopard.py -nt assets/leopard.mesh ]]; then
  python3 tools/gen_leopard.py assets/leopard.mesh
fi

if [[ ! -f assets/one1.mesh || tools/gen_one1.py -nt assets/one1.mesh ]]; then
  python3 tools/gen_one1.py assets/one1.mesh
fi
if [[ ! -f assets/ui.atlas || tools/gen_ui.py -nt assets/ui.atlas ]]; then
  python3 tools/gen_ui.py assets/ui.atlas
fi

exec ./build/beanng "$@"
