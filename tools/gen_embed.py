#!/usr/bin/env python3
"""Puts files into the program: gen_embed.py OUT_DIR FILE...  writes embed.S with the bytes and embed_table.cpp to find them."""
import os
import sys

out, files = sys.argv[1], sys.argv[2:]
os.makedirs(out, exist_ok=True)
asm, labels = ['.section .rdata,"dr"'], []
for i, path in enumerate(files):
    asm += [f".balign 16", f".global beanng_asset_{i}", f"beanng_asset_{i}:", f'.incbin "{os.path.abspath(path)}"', f".global beanng_asset_{i}_end", f"beanng_asset_{i}_end:"]
    labels.append(i)
with open(os.path.join(out, "embed.S"), "w") as f:
    f.write("\n".join(asm) + "\n")
with open(os.path.join(out, "embed_table.cpp"), "w") as f:
    f.write("struct EmbeddedAsset { const char* name; const unsigned char* start; const unsigned char* end; };\n")
    f.write('extern "C" {\n')
    for i in labels:
        f.write(f"extern const unsigned char beanng_asset_{i}[], beanng_asset_{i}_end[];\n")
    f.write("}\n")
    f.write("extern const EmbeddedAsset kEmbeddedAssets[] = {\n")
    for i, path in zip(labels, files):
        f.write(f'  {{"{path}", beanng_asset_{i}, beanng_asset_{i}_end}},\n')
    f.write("};\n")
    f.write(f"extern const int kEmbeddedAssetCount = {len(files)};\n")
