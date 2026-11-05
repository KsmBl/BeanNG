#!/usr/bin/env bash
# Generates the models and the UI atlas, each only when its generator has changed.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/.."
mkdir -p assets

if [[ ! -f assets/xantia.mesh || tools/gen_xantia.py -nt assets/xantia.mesh ]]; then
  python3 tools/gen_xantia.py assets/xantia.mesh
fi
for hull in medium heavy fast; do
  for v in std light arty missile; do
    name=leopard
    [[ $hull != medium ]] && name=${name}_$hull
    [[ $v != std ]] && name=${name}_$v
    if [[ ! -f assets/$name.mesh || tools/gen_leopard.py -nt assets/$name.mesh ]]; then
      python3 tools/gen_leopard.py assets/$name.mesh $v $hull
    fi
  done
done
if [[ ! -f assets/soldier.mesh || tools/gen_soldier.py -nt assets/soldier.mesh ]]; then
  python3 tools/gen_soldier.py assets/soldier.mesh olive olive2
  python3 tools/gen_soldier.py assets/soldier2.mesh sand rust
fi
if [[ ! -f assets/one1.mesh || tools/gen_one1.py -nt assets/one1.mesh ]]; then
  python3 tools/gen_one1.py assets/one1.mesh
fi
if [[ ! -f assets/hummer.mesh || tools/gen_hummer.py -nt assets/hummer.mesh ]]; then
  python3 tools/gen_hummer.py assets/hummer.mesh
fi
if [[ ! -f assets/tarp.mesh || tools/gen_tarp.py -nt assets/tarp.mesh ]]; then
  python3 tools/gen_tarp.py assets/tarp.mesh
fi
if [[ ! -f assets/ui.atlas || tools/gen_ui.py -nt assets/ui.atlas ]]; then
  python3 tools/gen_ui.py assets/ui.atlas
fi
