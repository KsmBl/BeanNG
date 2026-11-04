#include "platform.h"

#include <cstdio>
#include <cstdlib>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <process.h>
#include <timeapi.h>

std::string tempPath(const char* prefix) {
  const char* dir = getenv("TEMP");
  return std::string(dir ? dir : ".") + "\\" + prefix + "-" + std::to_string(_getpid()) + ".log";
}

void platformStart() { timeBeginPeriod(1); }

void platformConsole(bool wantOwn) {
  // Already going somewhere, into a file say: then that is where it should stay.
  if (GetStdHandle(STD_OUTPUT_HANDLE) && GetFileType(GetStdHandle(STD_OUTPUT_HANDLE)) != FILE_TYPE_UNKNOWN) return;
  if (!AttachConsole(ATTACH_PARENT_PROCESS) && (!wantOwn || !AllocConsole())) return;
  freopen("CONIN$", "r", stdin);
  freopen("CONOUT$", "w", stdout);
  freopen("CONOUT$", "w", stderr);
  SetConsoleOutputCP(CP_UTF8);
  // The server's prompt is drawn with the same escape sequences as on Linux.
  const HANDLE out = GetStdHandle(STD_OUTPUT_HANDLE);
  DWORD mode = 0;
  if (GetConsoleMode(out, &mode)) SetConsoleMode(out, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
}

#else
#include <unistd.h>

std::string tempPath(const char* prefix) { return std::string("/tmp/") + prefix + "-" + std::to_string(int(getpid())) + ".log"; }
void platformStart() {}
void platformConsole(bool) {}
#endif
