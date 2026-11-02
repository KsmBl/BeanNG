// Minimal math types. Physics runs in double, rendering converts to float.
#pragma once
#include <cmath>
#include <cstdint>

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

struct V3 {
  double x = 0, y = 0, z = 0;
  V3() = default;
  constexpr V3(double a, double b, double c) : x(a), y(b), z(c) {}
  V3 operator+(V3 o) const { return {x + o.x, y + o.y, z + o.z}; }
  V3 operator-(V3 o) const { return {x - o.x, y - o.y, z - o.z}; }
  V3 operator-() const { return {-x, -y, -z}; }
  V3 operator*(double s) const { return {x * s, y * s, z * s}; }
  V3 operator/(double s) const { return {x / s, y / s, z / s}; }
  V3& operator+=(V3 o) { x += o.x; y += o.y; z += o.z; return *this; }
  V3& operator-=(V3 o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
  V3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
};
inline V3 operator*(double s, V3 v) { return v * s; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double len2(V3 a) { return dot(a, a); }
inline double len(V3 a) { return std::sqrt(dot(a, a)); }
inline V3 norm(V3 a) {
  double l = len(a);
  return l > 1e-12 ? a / l : V3{0, 0, 0};
}
inline double clampd(double v, double a, double b) { return v < a ? a : (v > b ? b : v); }
inline double lerpd(double a, double b, double t) { return a + (b - a) * t; }
inline double smooth01(double t) {
  t = clampd(t, 0, 1);
  return t * t * (3 - 2 * t);
}
inline double smoothstepd(double a, double b, double x) { return smooth01((x - a) / (b - a)); }
inline V3 lerpv(V3 a, V3 b, double t) { return a + (b - a) * t; }

constexpr double PI = 3.14159265358979323846;
