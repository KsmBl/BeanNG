#include "vegetation.h"
#include <epoxy/gl.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>

namespace {
// What is common to both: the pressing, the shrinking at the edge of the range, the light, and fog.
const char* kCommon = R"(#version 330 core
uniform mat4 uVP; uniform mat4 uPrevVP; uniform mat4 uShadowVP;
uniform sampler2D uMarks; uniform sampler2DShadow uShadow;
uniform vec3 uEye; uniform vec3 uSun; uniform float uTime; uniform float uRange;
layout(location=2) in vec4 aInst;
layout(location=3) in vec4 aTint;
out vec3 vCol; out vec4 vClip; out vec4 vPrevClip;
vec2 marksHere(){ return texture(uMarks, (aInst.xy + 1024.0)/2048.0).rg; }
float shadowHere(vec3 p){
  vec4 q = uShadowVP*vec4(p,1.0);
  vec3 u = q.xyz/q.w*0.5+0.5;
  if(u.x<0.0||u.x>1.0||u.y<0.0||u.y>1.0||u.z>1.0) return 1.0;
  return texture(uShadow, vec3(u.xy, u.z-0.002));
}
vec3 lit(vec3 albedo, vec3 n, float wrap, vec3 p, float sh){
  float diff = clamp((dot(n,uSun)+wrap)/(1.0+wrap),0.0,1.0)*sh;
  vec3 amb = mix(vec3(0.20,0.19,0.17), vec3(0.34,0.40,0.50), n.z*0.5+0.5);
  vec3 c = albedo*(amb + diff*vec3(1.05,1.0,0.92));
  float fog = 1.0-exp(-length(uEye-p)*0.0006);
  return mix(c, vec3(0.66,0.70,0.74), clamp(fog,0.0,0.85));
}
void place(vec3 p){
  gl_Position = uVP*vec4(p,1.0);
  vClip = gl_Position;
  vPrevClip = uPrevVP*vec4(p,1.0);
}
)";

const char* kGrassVS = R"(
layout(location=0) in vec3 aPos;   // x and y across the tuft, z from the foot to the tip
void main(){
  vec2 m = marksHere();
  float seed = aInst.w;
  // Thick close by and thinning out with distance, each tuft at its own range. Nothing changes shape: some just shrink
  // away sooner, and those that stay grow broader, so that the meadow stays closed. The list is sorted by this same range.
  float d = length(uEye.xy-aInst.xy);
  float thin = fract(seed*7.13);
  float range = uRange*(0.18+0.82*thin*thin);
  float keep = (1.0-smoothstep(range*0.72,range,d))*(m.g>0.5?0.0:1.0);
  // Nothing right in front of the lens.
  keep *= smoothstep(0.7,1.7,length(uEye-aInst.xyz));
  float h = aTint.a*1.4*keep;
  float a = seed*6.2832;
  vec2 ca = vec2(cos(a),sin(a));
  float broad = 1.0+d/38.0;
  vec2 xy = vec2(aPos.x*ca.x-aPos.y*ca.y, aPos.x*ca.y+aPos.y*ca.x)*(h*0.5+0.14*keep)*broad;
  float t = aPos.z;
  float press = m.r;
  // Gusts that run across the land in waves, and each tuft's own flutter on top.
  vec2 wind = vec2(0.8,0.6);
  float phase = dot(aInst.xy,wind)*0.085-uTime*1.05;
  float wave = sin(phase)+0.5*sin(phase*2.3+aInst.y*0.04);
  float flutter = 0.22*sin(uTime*3.3+seed*40.0);
  vec2 lean = xy*t*0.9 + vec2(ca.y,-ca.x)*press*h*0.9*t + wind*(wave*0.75+flutter+0.35)*t*t*h*0.42*(1.0-press);
  vec3 p = vec3(aInst.xy+xy*(1.0-t*0.55)+lean, aInst.z-0.03+t*h*mix(1.0-0.12*max(wave,0.0),0.13,press)+0.02*press);
  vec3 base = aTint.rgb;
  // Dark down among itself, bright at the tips, and brighter still where a gust is passing. Pressed down it goes dull.
  vec3 albedo = base*mix(0.50,1.42,t*t*0.6+t*0.4);
  albedo *= 1.0+0.16*max(wave,0.0)*t;
  // Here and there one in flower.
  float bloom = fract(seed*53.1);
  if(bloom > 0.97 && aTint.a < 0.3 && t > 0.82) albedo = mix(albedo, mix(vec3(1.0,0.8,0.18), vec3(0.95,0.93,0.86), step(0.986,bloom)), 0.85);
  albedo = mix(albedo, base*0.92+vec3(0.05,0.045,0.012), press);
  vec3 n = normalize(vec3(xy*2.0/(h+0.05), 1.0));
  vCol = lit(albedo, n, 0.55, p, shadowHere(vec3(aInst.xy,aInst.z+0.15)));
  place(p);
})";

const char* kBushVS = R"(
layout(location=0) in vec3 aPos;
layout(location=1) in vec4 aNrm;  // w: how deep in among the leaves
void main(){
  vec2 m = marksHere();
  float seed = aInst.w;
  float d = length(uEye.xy-aInst.xy);
  float keep = (1.0-smoothstep(uRange*0.88,uRange,d))*(m.g>0.5?0.0:1.0);
  float size = aTint.a*5.0*keep;
  float a = seed*6.2832;
  vec2 ca = vec2(cos(a),sin(a));
  vec3 q = vec3(aPos.x*ca.x-aPos.y*ca.y, aPos.x*ca.y+aPos.y*ca.x, aPos.z);
  vec3 n = vec3(aNrm.x*ca.x-aNrm.y*ca.y, aNrm.x*ca.y+aNrm.y*ca.x, aNrm.z);
  // No two alike: wider or narrower, taller or squatter.
  q.xy *= mix(0.85,1.25,fract(seed*3.7));
  q.z *= mix(0.72,1.15,fract(seed*5.3));
  float press = m.r;
  q.xy *= mix(1.0,1.3,press);
  q.z *= mix(1.0,0.14,press);
  float gust = sin(uTime*1.3+aInst.x*0.2+aInst.y*0.17);
  q.xy += vec2(0.018,0.012)*gust*q.z*(1.0-press)*2.0;
  vec3 p = aInst.xyz+q*size;
  vec3 albedo = aTint.rgb*mix(1.55,0.30,aNrm.w)*mix(0.72,1.15,clamp(aPos.z*1.6,0.0,1.0));
  albedo = mix(albedo, aTint.rgb*0.72+vec3(0.05,0.04,0.012), press*0.8);
  float sh = shadowHere(aInst.xyz+vec3(0.0,0.0,size*0.6));
  vCol = lit(albedo, normalize(n), 0.45, p, mix(sh,1.0,0.15));
  place(p);
})";

const char* kFS = R"(#version 330 core
in vec3 vCol; in vec4 vClip; in vec4 vPrevClip;
layout(location=0) out vec4 frag;
layout(location=1) out vec4 velOut;
void main(){
  vec3 c = vCol/(vCol*0.22+0.85);
  frag = vec4(pow(c,vec3(1.0/1.6)),1.0);
  velOut = vec4(vPrevClip.w > 0.05 ? (vClip.xy/vClip.w - vPrevClip.xy/vPrevClip.w)*0.5 : vec2(0.0), 0.0, 0.0);
})";

const char* kShadowVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=2) in vec4 aInst;
layout(location=3) in vec4 aTint;
uniform mat4 uVP; uniform sampler2D uMarks;
void main(){
  vec2 m = texture(uMarks,(aInst.xy+1024.0)/2048.0).rg;
  float a = aInst.w*6.2832;
  vec2 ca = vec2(cos(a),sin(a));
  vec3 q = vec3(aPos.x*ca.x-aPos.y*ca.y, aPos.x*ca.y+aPos.y*ca.x, aPos.z*mix(1.0,0.14,m.r));
  gl_Position = uVP*vec4(aInst.xyz+q*aTint.a*5.0*(m.g>0.5?0.0:1.0),1.0);
})";
const char* kShadowFS = R"(#version 330 core
void main(){})";

GLuint shader(GLenum type, const std::string& text) {
  const GLuint s = glCreateShader(type);
  const char* src = text.c_str();
  glShaderSource(s, 1, &src, nullptr);
  glCompileShader(s);
  GLint ok = 0;
  glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char log[2048];
    glGetShaderInfoLog(s, sizeof log, nullptr, log);
    fprintf(stderr, "Vegetation, shader: %s\n", log);
  }
  return s;
}
GLuint link(const std::string& vs, const char* fs) {
  const GLuint p = glCreateProgram(), a = shader(GL_VERTEX_SHADER, vs), b = shader(GL_FRAGMENT_SHADER, fs);
  glAttachShader(p, a);
  glAttachShader(p, b);
  glLinkProgram(p);
  GLint ok = 0;
  glGetProgramiv(p, GL_LINK_STATUS, &ok);
  if (!ok) {
    char log[2048];
    glGetProgramInfoLog(p, sizeof log, nullptr, log);
    fprintf(stderr, "Vegetation, link: %s\n", log);
  }
  glDeleteShader(a);
  glDeleteShader(b);
  return p;
}
void instanceAttribs() {
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 20, (void*)0);
  glVertexAttribDivisor(2, 1);
  glEnableVertexAttribArray(3);
  glVertexAttribPointer(3, 4, GL_UNSIGNED_BYTE, GL_TRUE, 20, (void*)16);
  glVertexAttribDivisor(3, 1);
}
struct Rng {
  u32 s;
  float next(float a, float b) {
    s = s * 1664525u + 1013904223u;
    return a + (b - a) * float(s >> 16) / 65535.f;
  }
};
float softNoise(float x, float y) {
  // Rolling blots, for where things grow thick and where thin.
  return std::sin(x * 0.043f + std::sin(y * 0.031f) * 1.7f) * std::cos(y * 0.037f + std::sin(x * 0.029f) * 1.3f);
}
u8 toByte(float v) { return u8(std::clamp(v, 0.f, 1.f) * 255.f + .5f); }
}  // namespace

void Vegetation::buildMeshes() {
  // A tuft: seven fine blades, of two triangles each with a kink in the middle. What makes a meadow is how many there
  // are, and not how well each of them is made.
  std::vector<float> g;
  Rng r{4711};
  for (int b = 0; b < 7; b++) {
    const float a = 0.8976f * float(b) + r.next(-.4f, .4f), wid = r.next(.055f, .085f);
    const float rad = r.next(.04f, .42f);
    const float cx = std::cos(a) * rad, cy = std::sin(a) * rad;
    const float sa = a + r.next(1.1f, 2.0f);
    const float sx = std::cos(sa) * wid, sy = std::sin(sa) * wid;
    const float out = r.next(.15f, .62f);
    const float ox = std::cos(a) * out, oy = std::sin(a) * out;
    const float tall = r.next(.62f, 1.f);
    const auto vtx = [&](float side, float t) {
      const float bend = t * t;
      g.push_back(cx + ox * bend + sx * side);
      g.push_back(cy + oy * bend + sy * side);
      g.push_back(t * tall);
    };
    vtx(-1, 0); vtx(1, 0); vtx(.75f, .55f);
    vtx(-1, 0); vtx(.75f, .55f); vtx(-.15f, 1);
  }
  grassVerts_ = int(g.size() / 3);
  glGenVertexArrays(1, &grassVao_);
  glGenBuffers(1, &grassMesh_);
  glGenBuffers(1, &grassInst_);
  glBindVertexArray(grassVao_);
  glBindBuffer(GL_ARRAY_BUFFER, grassMesh_);
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(g.size() * sizeof(float)), g.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 12, (void*)0);
  glBindBuffer(GL_ARRAY_BUFFER, grassInst_);
  instanceAttribs();

  // A bush: three lobes with a dark body each, which gives it its mass and closes it up, and small pointed leaves sitting
  // on the outside of them, lit as if each lay on the dome so that it reads as one soft shape.
  std::vector<float> m;
  Rng q{90210};
  const float lobes[3][4] = {{0.f, 0.f, 0.f, .50f}, {.27f, .13f, -.06f, .36f}, {-.2f, -.22f, -.08f, .33f}};
  const auto vtxAt = [&](float x, float y, float z, float nx, float ny, float nz, float deep) {
    for (float f : {x, y, std::max(z, 0.f), nx, ny, nz, deep}) m.push_back(f);
  };
  constexpr int kBodyTris = 18 * 3;
  for (const auto& lobe : lobes) {
    const float rad = lobe[3] * .8f;
    const auto ring = [&](int i, float elev, float* out) {
      const float a = 1.0472f * float(i) + elev * .7f, wob = 1.f + .13f * std::sin(float(i) * 2.4f + lobe[0] * 9.f);
      const float cz = std::cos(elev), sz = std::sin(elev);
      out[0] = lobe[0] + std::cos(a) * cz * rad * wob;
      out[1] = lobe[1] + std::sin(a) * cz * rad * wob;
      out[2] = std::max(lobe[2] + sz * rad * 1.25f + .08f, 0.f);
      out[3] = std::cos(a) * cz;
      out[4] = std::sin(a) * cz;
      out[5] = sz;
    };
    const auto put = [&](const float* p) { vtxAt(p[0], p[1], p[2], p[3], p[4], p[5] + .25f, .8f); };
    for (int i = 0; i < 6; i++) {
      float a0[6], a1[6], b0[6], b1[6];
      ring(i, -.25f, a0);
      ring(i + 1, -.25f, a1);
      ring(i, .75f, b0);
      ring(i + 1, .75f, b1);
      const float top[6] = {lobe[0], lobe[1], std::max(lobe[2] + rad * 1.25f + .08f, 0.f), 0, 0, 1};
      put(a0); put(a1); put(b1);
      put(a0); put(b1); put(b0);
      put(b0); put(b1); put(top);
    }
  }
  const int leaves = (kBushTris - kBodyTris) / 2;
  for (int k = 0; k < leaves; k++) {
    const float* lobe = lobes[k % 7 == 0 ? 2 : (k % 3 == 0 ? 1 : 0)];
    const float a = q.next(0.f, 6.283f), u = q.next(-.05f, 1.f), depth = q.next(.86f, 1.12f);
    const float rad = std::sqrt(std::max(1.f - u * u, 0.f));
    const float dx0 = std::cos(a) * rad, dy0 = std::sin(a) * rad, dz0 = u;
    const float px = lobe[0] + dx0 * lobe[3] * depth, py = lobe[1] + dy0 * lobe[3] * depth, pz = std::max(lobe[2] + dz0 * lobe[3] * depth * 1.25f + .08f, .02f);
    // The top of the bush catches the light; further down and on the inside, less.
    const float deep = std::clamp(.42f - u * .42f + (1.f - depth) * .9f + q.next(-.16f, .16f), 0.f, .7f);
    const float len = q.next(.07f, .115f), wid = q.next(.03f, .048f);
    // Lying along the dome and hanging a little, not sticking out of it.
    const float ta = q.next(0.f, 6.283f);
    const float tx = std::cos(ta), ty = std::sin(ta);
    const float along = tx * dx0 + ty * dy0;
    const float dx = (tx - dx0 * along) * len, dy = (ty - dy0 * along) * len, dz = (-dz0 * along - q.next(.05f, .4f)) * len;
    const float sx = -ty * wid, sy = tx * wid, sz = q.next(-.018f, .018f);
    const float lift = q.next(.0f, .02f);
    const auto leaf = [&](float x, float y, float z) { vtxAt(x + dx0 * lift, y + dy0 * lift, z + dz0 * lift, dx0, dy0, dz0 + .25f, deep); };
    leaf(px - dx, py - dy, pz - dz); leaf(px - dx * .1f + sx, py - dy * .1f + sy, pz - dz * .1f + sz); leaf(px + dx, py + dy, pz + dz);
    leaf(px - dx, py - dy, pz - dz); leaf(px + dx, py + dy, pz + dz); leaf(px - dx * .1f - sx, py - dy * .1f - sy, pz - dz * .1f - sz);
  }
  bushVerts_ = int(m.size() / 7);
  glGenVertexArrays(1, &bushVao_);
  glGenBuffers(1, &bushMesh_);
  glGenBuffers(1, &bushInst_);
  glBindVertexArray(bushVao_);
  glBindBuffer(GL_ARRAY_BUFFER, bushMesh_);
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(m.size() * sizeof(float)), m.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 28, (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 28, (void*)12);
  glBindBuffer(GL_ARRAY_BUFFER, bushInst_);
  instanceAttribs();
  glBindVertexArray(0);
}

void Vegetation::placeBushes() {
  Rng r{31337};
  std::vector<std::vector<Inst>> cells(64 * 64);
  const auto addBush = [&](float x, float y, float size) {
    if (world_->surfaceAt(x, y) != SURF_GRASS) return;
    const float z = float(world_->height(x, y));
    if (world_->isWater(x, y, z) || world_->terrainNormal(x, y).z < 0.8) return;
    Contact c;
    if (world_->contact(V3{x, y, z + 0.4}, c)) return;
    for (const World::House& h : world_->houses())
      if (std::hypot(h.x - x, h.y - y) < 8.5f) return;
    Inst b;
    b.x = x;
    b.y = y;
    b.z = z - 0.04f;
    b.seed = r.next(0.f, 1.f);
    const float dry = r.next(0.f, 1.f);
    b.r = toByte(.085f + dry * .13f);
    b.g = toByte(.27f + dry * .11f);
    b.b = toByte(.085f + dry * .015f);
    b.size = toByte(size / 5.f);
    const int cx = std::clamp(int((x + 1024.f) / 32.f), 0, 63), cy = std::clamp(int((y + 1024.f) / 32.f), 0, 63);
    cells[size_t(cy) * 64 + size_t(cx)].push_back(b);
  };
  for (int n = 0; n < 9000; n++) {
    const float x = r.next(-880.f, 880.f), y = r.next(-880.f, 880.f);
    // Thick in some places and absent in others, and in groups rather than strewn about.
    if (softNoise(x, y) + r.next(-.4f, .4f) < -0.15f) continue;
    const int group = int(r.next(1.f, 7.f));
    for (int k = 0; k < group; k++) addBush(x + r.next(-8.f, 8.f), y + r.next(-8.f, 8.f), 2.0f + 2.9f * std::pow(r.next(0.f, 1.f), 1.4f));
  }
  for (size_t i = 0; i < cells.size(); i++) {
    if (cells[i].empty()) continue;
    const u32 from = u32(bushes_.size());
    bushes_.insert(bushes_.end(), cells[i].begin(), cells[i].end());
    bushCells_.push_back({from, u32(cells[i].size())});
    bushCellMid_.push_back(V3{(double(i % 64) + .5) * 32.0 - 1024.0, (double(i / 64) + .5) * 32.0 - 1024.0, 0});
  }
}

void Vegetation::init(const World& world, std::function<void(int, int, float&, float&, float&)> groundColor) {
  world_ = &world;
  groundColor_ = std::move(groundColor);
  marks_.assign(size_t(kMapSize) * kMapSize * 2, 0);
  glGenTextures(1, &marksTex_);
  glBindTexture(GL_TEXTURE_2D, marksTex_);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, kMapSize, kMapSize, 0, GL_RG, GL_UNSIGNED_BYTE, marks_.data());
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  const std::string common = kCommon;
  progGrass_ = link(common + kGrassVS, kFS);
  progBush_ = link(common + kBushVS, kFS);
  progBushShadow_ = link(kShadowVS, kShadowFS);
  buildMeshes();
  if (world.isArena()) placeBushes();
  printf("Vegetation: %zu Buesche in %zu Zellen\n", bushes_.size(), bushCells_.size());
}

const Vegetation::Cell& Vegetation::cellAt(int cx, int cy, int& budget, bool& missing) {
  static const Cell kEmpty;
  const u64 key = (u64(u32(cx)) << 32) | u32(cy);
  auto it = cells_.find(key);
  if (it != cells_.end()) return it->second;
  // Never too many in one frame: a tank at speed crosses a cell a second, and that must not show.
  if (budget <= 0) {
    missing = true;
    return kEmpty;
  }
  budget--;
  Cell& cell = cells_[key];
  const float x0 = float(cx) * float(kCell), y0 = float(cy) * float(kCell);
  Rng r{u32(cx) * 73856093u ^ u32(cy) * 19349663u ^ 0x5bd1e995u};
  // The ground's colour, every four metres: it changes slowly, and asking for it costs more than all the rest.
  const int kTable = 9;
  float table[kTable * kTable][3];
  for (int j = 0; j < kTable; j++)
    for (int i = 0; i < kTable; i++) {
      float* c = table[j * kTable + i];
      c[0] = .2f; c[1] = .3f; c[2] = .1f;
      groundColor_(std::clamp(int(x0 + float(i) * 4.f + 1024.f), 0, World::N - 1), std::clamp(int(y0 + float(j) * 4.f + 1024.f), 0, World::N - 1), c[0], c[1], c[2]);
    }
  // Roughly ten tufts a square metre to try for.
  constexpr int kTries = 10500;
  cell.grass.reserve(kTries);
  for (int n = 0; n < kTries; n++) {
    const float x = x0 + r.next(0.f, float(kCell)), y = y0 + r.next(0.f, float(kCell));
    const float seed = r.next(0.f, 1.f), roll = r.next(0.f, 1.f), hue = r.next(-1.f, 1.f), keepRoll = r.next(0.f, 1.f);
    // A meadow, with thinner places in it, and everywhere a different grass from a few metres on.
    const float patch = softNoise(x * 3.1f, y * 3.1f), wide = softNoise(x * 0.7f + 40.f, y * 0.7f - 15.f);
    if (keepRoll > 0.88f + 0.16f * patch) continue;
    if (std::fabs(x) > 950.f || std::fabs(y) > 950.f || world_->surfaceAt(x, y) != SURF_GRASS) continue;
    const float z = float(world_->height(x, y));
    if (world_->isWater(x, y, z)) continue;
    Contact c;
    if (world_->contact(V3{x, y, z + 0.3}, c)) continue;
    Inst g;
    g.x = x;
    g.y = y;
    g.z = z;
    g.seed = seed;
    const float* ground = table[std::clamp(int((y - y0) / 4.f + .5f), 0, kTable - 1) * kTable + std::clamp(int((x - x0) / 4.f + .5f), 0, kTable - 1)];
    // Lush greens going over to yellow in great soft patches, a little of the ground in it, and no two tufts the same.
    const float sun = std::clamp(.5f + .55f * wide + .18f * hue, 0.f, 1.f);
    const float lr = .17f + .24f * sun, lg = .37f + .15f * sun, lb = .075f + .025f * sun;
    g.r = toByte((lr * .72f + ground[0] * .34f) * (1.f + hue * .10f));
    g.g = toByte((lg * .72f + ground[1] * .34f) * (1.f + hue * .07f));
    g.b = toByte((lb * .72f + ground[2] * .30f));
    // A short carpet, grass to the knee, and stands of it to the hip where the land is lush.
    const float lush = std::clamp(patch * .5f + .5f, 0.f, 1.f);
    float tall;
    if (roll < .60f - .25f * lush) tall = .26f + r.next(0.f, .20f);
    else if (roll < .95f - .10f * lush) tall = .46f + r.next(0.f, .36f);
    else tall = .85f + r.next(0.f, .5f);
    g.size = toByte(tall / 1.4f);
    cell.grass.push_back(g);
  }
  // Those that reach furthest first: a cell far off sends only the start of its list.
  const auto reach = [](const Inst& i) {
    const float f = i.seed * 7.13f;
    return f - std::floor(f);
  };
  std::sort(cell.grass.begin(), cell.grass.end(), [&](const Inst& a, const Inst& b) { return reach(a) > reach(b); });
  return cell;
}

void Vegetation::stamp(V3 at, V3 fwd, V3 left, double halfL, double halfW) {
  const double ex = std::fabs(fwd.x) * halfL + std::fabs(left.x) * halfW, ey = std::fabs(fwd.y) * halfL + std::fabs(left.y) * halfW;
  const int x0 = std::max(int(at.x - ex + 1024.0), 0), x1 = std::min(int(at.x + ex + 1024.0) + 1, kMapSize - 1);
  const int y0 = std::max(int(at.y - ey + 1024.0), 0), y1 = std::min(int(at.y + ey + 1024.0) + 1, kMapSize - 1);
  if (x0 > x1 || y0 > y1) return;
  bool changed = false;
  for (int y = y0; y <= y1; y++)
    for (int x = x0; x <= x1; x++) {
      const V3 q{double(x) - 1024.0 + 0.5 - at.x, double(y) - 1024.0 + 0.5 - at.y, 0};
      if (std::fabs(dot(q, fwd)) > halfL || std::fabs(dot(q, left)) > halfW) continue;
      u8& m = marks_[(size_t(y) * kMapSize + size_t(x)) * 2];
      if (m != 255) {
        m = 255;
        changed = true;
      }
    }
  if (!changed) return;
  glBindTexture(GL_TEXTURE_2D, marksTex_);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, kMapSize);
  glTexSubImage2D(GL_TEXTURE_2D, 0, x0, y0, x1 - x0 + 1, y1 - y0 + 1, GL_RG, GL_UNSIGNED_BYTE, &marks_[(size_t(y0) * kMapSize + size_t(x0)) * 2]);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
}

void Vegetation::setDensity(double d) {
  if (d != density_) builtEye_ = V3{1e9, 0, 0};
  density_ = d;
}

V3 Vegetation::nearestBush(V3 from, double beyond) const {
  V3 best = from;
  double bestD = 1e30;
  for (const Inst& b : bushes_) {
    const double d = std::hypot(double(b.x) - from.x, double(b.y) - from.y);
    if (d > beyond && d < bestD) {
      // One standing free, for a clear look at it.
      bool clear = true;
      for (const V3& t : world_->trees())
        if (std::hypot(t.x - b.x, t.y - b.y) < 16.0) clear = false;
      if (!clear) continue;
      bestD = d;
      best = V3{b.x, b.y, b.z};
    }
  }
  return best;
}

void Vegetation::clearMarks() {
  std::fill(marks_.begin(), marks_.end(), u8(0));
  cratersSeen_ = 0;
  glBindTexture(GL_TEXTURE_2D, marksTex_);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, kMapSize, kMapSize, GL_RG, GL_UNSIGNED_BYTE, marks_.data());
}

void Vegetation::rebuild(V3 eye, V3 viewDir) {
  const auto t0 = std::chrono::steady_clock::now();
  const V3 flat = norm(V3{viewDir.x, viewDir.y, 1e-6});
  // What can be in the picture, with room to spare: the list stays as it is for several metres and degrees.
  const auto wanted = [&](V3 mid, double range) {
    const V3 to{mid.x - eye.x, mid.y - eye.y, 0};
    const double d = len(to);
    if (d > range + kCell) return false;
    return d < 1.5 * kCell || dot(to, flat) / d > 0.3;
  };
  scratch_.clear();
  int budget = 2;
  bool missing = false;
  const int cr = int(kGrassRange / kCell) + 1;
  const int ccx = int(std::floor(eye.x / kCell)), ccy = int(std::floor(eye.y / kCell));
  // Nearest first, so that what is built first is what is seen.
  std::vector<std::pair<double, std::pair<int, int>>> order;
  for (int cy = ccy - cr; cy <= ccy + cr; cy++)
    for (int cx = ccx - cr; cx <= ccx + cr; cx++) {
      const V3 mid{(double(cx) + .5) * kCell, (double(cy) + .5) * kCell, 0};
      if (wanted(mid, kGrassRange)) order.push_back({len2(mid - V3{eye.x, eye.y, 0}), {cx, cy}});
    }
  std::sort(order.begin(), order.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  for (const auto& o : order) {
    const Cell& c = cellAt(o.second.first, o.second.second, budget, missing);
    if (c.grass.empty()) continue;
    // As far as any of it can be seen from here. The shader's range: R (0.18 + 0.82 thin^2).
    const double nearest = std::max(std::sqrt(o.first) - kCell * 0.75, 0.0);
    const double needed = std::sqrt(std::max(nearest / kGrassRange - 0.18, 0.0) / 0.82);
    size_t lo = 0, hi = c.grass.size();
    while (lo < hi) {
      const size_t mid = (lo + hi) / 2;
      const float f = c.grass[mid].seed * 7.13f;
      if (double(f - std::floor(f)) >= needed) lo = mid + 1;
      else hi = mid;
    }
    scratch_.insert(scratch_.end(), c.grass.begin(), c.grass.begin() + long(double(lo) * density_));
  }
  grassCount_ = int(scratch_.size());
  glBindBuffer(GL_ARRAY_BUFFER, grassInst_);
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(scratch_.size() * sizeof(Inst)), scratch_.data(), GL_STREAM_DRAW);
  scratch_.clear();
  for (size_t i = 0; i < bushCells_.size(); i++) {
    if (!wanted(bushCellMid_[i], kBushRange)) continue;
    const Inst* first = bushes_.data() + bushCells_[i].first;
    scratch_.insert(scratch_.end(), first, first + bushCells_[i].second);
  }
  bushCount_ = int(scratch_.size());
  glBindBuffer(GL_ARRAY_BUFFER, bushInst_);
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(scratch_.size() * sizeof(Inst)), scratch_.data(), GL_STREAM_DRAW);
  // What has gone far behind is built again if it is ever needed.
  if (cells_.size() > 150) {
    for (auto it = cells_.begin(); it != cells_.end();) {
      const int cx = int(int32_t(it->first >> 32)), cy = int(int32_t(it->first & 0xffffffffu));
      const V3 mid{(double(cx) + .5) * kCell, (double(cy) + .5) * kCell, 0};
      if (len(mid - V3{eye.x, eye.y, 0}) > kGrassRange * 1.8) it = cells_.erase(it);
      else ++it;
    }
  }
  builtEye_ = missing ? V3{1e9, 0, 0} : eye;
  builtDir_ = viewDir;
  rebuildMs_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void Vegetation::update(const Snapshot& s, V3 eye, V3 viewDir, double dt) {
  clock_ += dt;
  if (!world_ || !world_->isArena()) return;
  for (const Snapshot::GroundBox& b : s.groundBoxes) stamp(b.at, b.fwd, b.left, b.halfL, b.halfW);
  // A blast burns away what grew there.
  if (s.craters.size() < cratersSeen_) cratersSeen_ = 0;
  for (; cratersSeen_ < s.craters.size(); cratersSeen_++) {
    const World::Crater& c = s.craters[cratersSeen_];
    const double reach = c.r * 1.15;
    const int x0 = std::max(int(c.x - reach + 1024.0), 0), x1 = std::min(int(c.x + reach + 1024.0) + 1, kMapSize - 1);
    const int y0 = std::max(int(c.y - reach + 1024.0), 0), y1 = std::min(int(c.y + reach + 1024.0) + 1, kMapSize - 1);
    if (x0 > x1 || y0 > y1) continue;
    for (int y = y0; y <= y1; y++)
      for (int x = x0; x <= x1; x++)
        if (std::hypot(double(x) - 1024.0 - c.x, double(y) - 1024.0 - c.y) < reach) marks_[(size_t(y) * kMapSize + size_t(x)) * 2 + 1] = 255;
    glBindTexture(GL_TEXTURE_2D, marksTex_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, kMapSize);
    glTexSubImage2D(GL_TEXTURE_2D, 0, x0, y0, x1 - x0 + 1, y1 - y0 + 1, GL_RG, GL_UNSIGNED_BYTE, &marks_[(size_t(y0) * kMapSize + size_t(x0)) * 2]);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
  }
  if (len(V3{eye.x - builtEye_.x, eye.y - builtEye_.y, 0}) > 5.0 || dot(viewDir, builtDir_) < 0.985) rebuild(eye, viewDir);

}

void Vegetation::drawShadow(const float* lightVP) {
  if (!bushCount_) return;
  glUseProgram(progBushShadow_);
  glUniformMatrix4fv(glGetUniformLocation(progBushShadow_, "uVP"), 1, GL_FALSE, lightVP);
  glActiveTexture(GL_TEXTURE4);
  glBindTexture(GL_TEXTURE_2D, marksTex_);
  glUniform1i(glGetUniformLocation(progBushShadow_, "uMarks"), 4);
  glActiveTexture(GL_TEXTURE0);
  glBindVertexArray(bushVao_);
  glDrawArraysInstanced(GL_TRIANGLES, 0, bushVerts_, bushCount_);
}

void Vegetation::draw(const float* vp, const float* prevVP, const float* lightVP, V3 eye, V3 sun) {
  glActiveTexture(GL_TEXTURE4);
  glBindTexture(GL_TEXTURE_2D, marksTex_);
  glActiveTexture(GL_TEXTURE0);
  const auto use = [&](GLuint p, float range) {
    glUseProgram(p);
    glUniformMatrix4fv(glGetUniformLocation(p, "uVP"), 1, GL_FALSE, vp);
    glUniformMatrix4fv(glGetUniformLocation(p, "uPrevVP"), 1, GL_FALSE, prevVP);
    glUniformMatrix4fv(glGetUniformLocation(p, "uShadowVP"), 1, GL_FALSE, lightVP);
    glUniform1i(glGetUniformLocation(p, "uMarks"), 4);
    glUniform1i(glGetUniformLocation(p, "uShadow"), 0);
    glUniform3f(glGetUniformLocation(p, "uEye"), float(eye.x), float(eye.y), float(eye.z));
    glUniform3f(glGetUniformLocation(p, "uSun"), float(sun.x), float(sun.y), float(sun.z));
    glUniform1f(glGetUniformLocation(p, "uTime"), float(std::fmod(clock_, 1000.0)));
    glUniform1f(glGetUniformLocation(p, "uRange"), range);
  };
  if (grassCount_) {
    use(progGrass_, float(kGrassRange));
    glBindVertexArray(grassVao_);
    glDrawArraysInstanced(GL_TRIANGLES, 0, grassVerts_, grassCount_);
  }
  if (bushCount_) {
    use(progBush_, float(kBushRange));
    glBindVertexArray(bushVao_);
    glDrawArraysInstanced(GL_TRIANGLES, 0, bushVerts_, bushCount_);
  }
}
