#!/usr/bin/env bash
# Builds BeanNG if needed and starts it. Arguments are passed on to the game.
set -euo pipefail

cd "$(dirname "$(readlink -f "$0")")"

if [[ ! -f build/build.ninja ]]; then
  cmake -S . -B build -G Ninja
fi
cmake --build build --target beanng

exec ./build/beanng "$@"
