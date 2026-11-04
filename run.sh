#!/usr/bin/env bash
# Builds BeanNG if needed and starts it. Arguments are passed on to the game.
set -euo pipefail

cd "$(dirname "$(readlink -f "$0")")"

if [[ ! -f build/build.ninja ]]; then
  cmake -S . -B build -G Ninja
fi
cmake --build build --target beanng beanng-server

# The models are generated, and only rebuilt when a generator has changed.
tools/gen_assets.sh

exec ./build/beanng "$@"
