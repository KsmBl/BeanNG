// Reading what the game ships with. From a file if there is one, otherwise out of the program itself: the Windows build
// carries all of it inside the .exe.
#pragma once
#include <cstdio>
#include <cstddef>

class AssetFile {
 public:
  explicit AssetFile(const char* path);
  ~AssetFile();
  AssetFile(const AssetFile&) = delete;
  AssetFile& operator=(const AssetFile&) = delete;
  explicit operator bool() const { return file_ || bytes_; }
  // As fread: how many whole items there were.
  size_t read(void* out, size_t size, size_t count);

 private:
  FILE* file_ = nullptr;
  const unsigned char* bytes_ = nullptr;
  size_t left_ = 0;
};
