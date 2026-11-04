// What differs between Linux and Windows, kept out of the headers everyone includes: windows.h defines far too much.
#pragma once
#include <string>

// Where a log of this run goes.
std::string tempPath(const char* prefix);
// Windows sleeps in steps of 15 ms unless asked not to; the server and the frame clock need better.
void platformStart();
// On Windows the game has no console of its own: this one belongs to whoever started it, or a new one if asked for.
void platformConsole(bool wantOwn);
