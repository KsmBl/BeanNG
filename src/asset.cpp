#include "asset.h"

#include <cstring>

#ifdef BEANNG_EMBED_ASSETS
struct EmbeddedAsset {
  const char* name;
  const unsigned char* start;
  const unsigned char* end;
};
extern const EmbeddedAsset kEmbeddedAssets[];
extern const int kEmbeddedAssetCount;
#endif

AssetFile::AssetFile(const char* path) {
  file_ = fopen(path, "rb");
#ifdef BEANNG_EMBED_ASSETS
  if (file_) return;
  for (int i = 0; i < kEmbeddedAssetCount; i++) {
    if (strcmp(kEmbeddedAssets[i].name, path) != 0) continue;
    bytes_ = kEmbeddedAssets[i].start;
    left_ = size_t(kEmbeddedAssets[i].end - kEmbeddedAssets[i].start);
    return;
  }
#endif
}

AssetFile::~AssetFile() {
  if (file_) fclose(file_);
}

size_t AssetFile::read(void* out, size_t size, size_t count) {
  if (file_) return fread(out, size, count, file_);
  if (!bytes_ || size == 0) return 0;
  const size_t items = count < left_ / size ? count : left_ / size;
  memcpy(out, bytes_, items * size);
  bytes_ += items * size;
  left_ -= items * size;
  return items;
}
