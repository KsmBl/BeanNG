#include "render.h"
#include <epoxy/gl.h>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <random>
#include "props.h"

namespace {
struct Mat4 {
  float m[16] = {};
};
struct V3f {
  float x, y, z;
};
inline V3f toF(V3 v, V3 o) { return {float(v.x - o.x), float(v.y - o.y), float(v.z - o.z)}; }

Mat4 mul(const Mat4& a, const Mat4& b) {
  Mat4 r;
  for (int c = 0; c < 4; c++)
    for (int i = 0; i < 4; i++) {
      float s = 0;
      for (int k = 0; k < 4; k++) s += a.m[k * 4 + i] * b.m[c * 4 + k];
      r.m[c * 4 + i] = s;
    }
  return r;
}
Mat4 lookAt(V3 eye, V3 target, V3 up) {
  const V3 f = norm(target - eye), s = norm(cross(f, up)), u = cross(s, f);
  Mat4 r;
  r.m[0] = float(s.x); r.m[4] = float(s.y); r.m[8] = float(s.z);
  r.m[1] = float(u.x); r.m[5] = float(u.y); r.m[9] = float(u.z);
  r.m[2] = float(-f.x); r.m[6] = float(-f.y); r.m[10] = float(-f.z);
  r.m[12] = float(-dot(s, eye)); r.m[13] = float(-dot(u, eye)); r.m[14] = float(dot(f, eye));
  r.m[15] = 1;
  return r;
}
Mat4 perspective(double fovy, double aspect, double zn, double zf) {
  const double f = 1.0 / std::tan(fovy * 0.5);
  Mat4 r;
  r.m[0] = float(f / aspect);
  r.m[5] = float(f);
  r.m[10] = float((zf + zn) / (zn - zf));
  r.m[11] = -1;
  r.m[14] = float(2 * zf * zn / (zn - zf));
  return r;
}
Mat4 ortho(double l, double r_, double b, double t, double n, double f) {
  Mat4 r;
  r.m[0] = float(2 / (r_ - l));
  r.m[5] = float(2 / (t - b));
  r.m[10] = float(-2 / (f - n));
  r.m[12] = float(-(r_ + l) / (r_ - l));
  r.m[13] = float(-(t + b) / (t - b));
  r.m[14] = float(-(f + n) / (f - n));
  r.m[15] = 1;
  return r;
}

GLuint compile(GLenum type, const char* src) {
  GLuint s = glCreateShader(type);
  glShaderSource(s, 1, &src, nullptr);
  glCompileShader(s);
  GLint ok = 0;
  glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char log[2048];
    glGetShaderInfoLog(s, sizeof log, nullptr, log);
    fprintf(stderr, "shader error: %s\n", log);
  }
  return s;
}
GLuint program(const char* vs, const char* fs) {
  GLuint p = glCreateProgram();
  const GLuint a = compile(GL_VERTEX_SHADER, vs), b = compile(GL_FRAGMENT_SHADER, fs);
  glAttachShader(p, a);
  glAttachShader(p, b);
  glLinkProgram(p);
  GLint ok = 0;
  glGetProgramiv(p, GL_LINK_STATUS, &ok);
  if (!ok) {
    char log[2048];
    glGetProgramInfoLog(p, sizeof log, nullptr, log);
    fprintf(stderr, "link error: %s\n", log);
  }
  glDeleteShader(a);
  glDeleteShader(b);
  return p;
}

const char* kMeshVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec4 aCol;
uniform mat4 uVP;
uniform mat4 uModel;
uniform mat4 uShadowVP;
out vec3 vPos; out vec3 vNrm; out vec4 vCol; out vec4 vShadow;
void main(){
  vec4 w = uModel * vec4(aPos,1.0);
  vPos = w.xyz;
  vNrm = mat3(uModel) * aNrm;
  vCol = aCol;
  vShadow = uShadowVP * w;
  gl_Position = uVP * w;
})";

// aCol.a encodes the material: <0.5 matte, 0.5..0.9 paint with sky reflection, 1 = opaque glossy
const char* kMeshFS = R"(#version 330 core
in vec3 vPos; in vec3 vNrm; in vec4 vCol; in vec4 vShadow;
uniform vec3 uEye;
uniform vec3 uSun;
uniform sampler2DShadow uShadow;
uniform float uAlpha;
out vec4 frag;
vec3 sky(vec3 d){
  float t = clamp(d.z*0.5+0.5,0.0,1.0);
  vec3 c = mix(vec3(0.62,0.60,0.56), vec3(0.32,0.52,0.82), pow(t,0.7));
  if(d.z<0.0) c = mix(vec3(0.33,0.32,0.28), c, clamp(1.0+d.z*6.0,0.0,1.0));
  c += vec3(1.0,0.9,0.75)*pow(max(dot(d,uSun),0.0),180.0)*2.5;
  return c;
}
float shadow(){
  vec3 p = vShadow.xyz/vShadow.w*0.5+0.5;
  if(p.x<0.0||p.x>1.0||p.y<0.0||p.y>1.0||p.z>1.0) return 1.0;
  float s=0.0;
  for(int x=-1;x<=1;x++) for(int y=-1;y<=1;y++)
    s += texture(uShadow, vec3(p.xy+vec2(x,y)/2048.0, p.z-0.0009));
  return s/9.0;
}
void main(){
  vec3 N = normalize(vNrm);
  vec3 V = normalize(uEye - vPos);
  float gloss = vCol.a;
  if(dot(N,V)<0.0){ N=-N; if(gloss>0.05) gloss=0.0; }
  float diff = max(dot(N,uSun),0.0)*shadow();
  vec3 amb = mix(vec3(0.20,0.19,0.17), vec3(0.34,0.40,0.50), N.z*0.5+0.5);
  vec3 c = vCol.rgb*(amb + diff*vec3(1.05,1.0,0.92));
  if(gloss>0.05){
    vec3 R = reflect(-V,N);
    float fres = pow(1.0-max(dot(N,V),0.0),4.0);
    bool metal = gloss > 0.97;
    float k = metal ? mix(0.70,0.95,fres) : mix(0.04,0.70,fres)*gloss*1.25;
    vec3 env = sky(R);
    if(metal) env *= vCol.rgb*1.35;
    c = mix(c, env, clamp(k,0.0,1.0));
    c += vec3(1.0,0.95,0.85)*pow(max(dot(R,uSun),0.0),90.0)*gloss*diff;
  }
  float d = length(uEye-vPos);
  float fog = 1.0-exp(-d*0.0011);
  c = mix(c, vec3(0.66,0.70,0.74), clamp(fog,0.0,0.85));
  c = c/(c*0.22+0.85);
  frag = vec4(pow(c,vec3(1.0/1.6)), uAlpha);
})";

// Wheel: rim damage turns with the wheel, the contact patch and tyre bulge stay at the road.
const char* kWheelVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
layout(location=2) in vec4 aCol;
uniform mat4 uVP;
uniform mat4 uModel;
uniform mat4 uShadowVP;
uniform int uMode;        // 0 rigid, 1 rim, 2 tyre
uniform vec2 uSpin;       // cos, sin of the wheel angle
uniform float uDent[24];
uniform vec2 uRunout;
uniform vec3 uPlaneN;     // towards the road, wheel frame
uniform float uPlaneD;    // hub to road
uniform float uDeflate;
out vec3 vPos; out vec3 vNrm; out vec4 vCol; out vec4 vShadow;
const float PI = 3.14159265;
void main(){
  vec3 p = aPos;
  vec3 n = aNrm;
  if(uMode > 0){
    float r = length(p.xz);
    float phi = atan(p.z, p.x);
    float f = (phi + PI) / (2.0*PI) * 24.0;
    f = mod(f + 12.0, 24.0);   // bin 0 at phi = 0
    int i0 = int(f);
    float dent = mix(uDent[i0], uDent[(i0+1)%24], f - float(i0));
    float rimF = smoothstep(0.06, 0.18, r);
    float r2 = r - dent*rimF;
    if(uMode == 2){
      // The carcass above the bead: a flat tyre loses its height and its shape.
      float bead = 0.195;
      float wall = max(r - bead, 0.0);
      float wrinkle = uDeflate * 0.005 * sin(phi*11.0 + p.y*40.0);
      r2 = (bead - dent) + wall*(1.0 - 0.34*uDeflate) + wrinkle;
      float edge = clamp(wall/0.117, 0.0, 1.0);
      p.y *= 1.0 + uDeflate*0.10*sin(edge*PI);
    }
    if(r > 1e-4) p.xz *= r2/r;
    p.y += dot(uRunout, vec2(cos(phi), sin(phi))) * rimF;
  }
  // Turn the wheel.
  p.xz = vec2(p.x*uSpin.x - p.z*uSpin.y, p.x*uSpin.y + p.z*uSpin.x);
  n.xz = vec2(n.x*uSpin.x - n.z*uSpin.y, n.x*uSpin.y + n.z*uSpin.x);
  if(uMode == 2){
    // The road flattens the tread and pushes the sidewalls out. This happens at any load, and it
    // is simply far more of it on a flat tyre.
    float h = dot(p, uPlaneN);
    float band = 0.05 + 0.05*uDeflate;
    float t = clamp((h - (uPlaneD - band))/band, 0.0, 1.0);
    float bulge = t*t*(0.005 + 0.035*uDeflate);
    p.y += sign(p.y) * bulge * (1.0 - 0.5*abs(p.y)/0.1025);
    if(h > uPlaneD){
      p -= uPlaneN*(h - uPlaneD);
      n = normalize(mix(n, -uPlaneN, 0.85));
    }
  }
  vec4 w = uModel * vec4(p,1.0);
  vPos = w.xyz;
  vNrm = mat3(uModel) * n;
  vCol = aCol;
  vShadow = uShadowVP * w;
  gl_Position = uVP * w;
})";

const char* kDepthVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uVP; uniform mat4 uModel;
void main(){ gl_Position = uVP*uModel*vec4(aPos,1.0); })";
const char* kDepthFS = R"(#version 330 core
void main(){})";

const char* kSkyVS = R"(#version 330 core
out vec2 vUv;
void main(){ vec2 p=vec2((gl_VertexID<<1)&2, gl_VertexID&2); vUv=p*2.0-1.0; gl_Position=vec4(vUv,0.9999,1.0); })";
const char* kSkyFS = R"(#version 330 core
in vec2 vUv; out vec4 frag;
uniform mat3 uInvRot; uniform float uTanY; uniform float uAspect; uniform vec3 uSun;
void main(){
  vec3 d = normalize(uInvRot*vec3(vUv.x*uTanY*uAspect, vUv.y*uTanY, -1.0));
  float t = clamp(d.z,0.0,1.0);
  vec3 c = mix(vec3(0.74,0.79,0.84), vec3(0.28,0.47,0.76), pow(t,0.55));
  c += vec3(1.0,0.92,0.78)*pow(max(dot(d,uSun),0.0),600.0)*2.0;
  c += vec3(1.0,0.85,0.65)*pow(max(dot(d,uSun),0.0),12.0)*0.12;
  frag=vec4(c,1.0);
})";

const char* kPartVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec4 aCol;
layout(location=2) in float aSize;
uniform mat4 uVP; uniform float uScale;
out vec4 vCol;
void main(){ gl_Position=uVP*vec4(aPos,1.0); vCol=aCol; gl_PointSize=clamp(aSize*uScale/gl_Position.w,1.0,160.0); })";
const char* kPartFS = R"(#version 330 core
in vec4 vCol; out vec4 frag;
void main(){ vec2 c=gl_PointCoord*2.0-1.0; float a=1.0-dot(c,c); if(a<=0.0) discard; frag=vec4(vCol.rgb,vCol.a*a); })";

const char* kLineVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec4 aCol;
uniform mat4 uVP; out vec4 vCol;
void main(){ gl_Position=uVP*vec4(aPos,1.0); vCol=aCol; })";
const char* kLineFS = R"(#version 330 core
in vec4 vCol; out vec4 frag; void main(){ frag=vCol; })";

const char* kHudVS = R"(#version 330 core
layout(location=0) in vec2 aPos;
layout(location=1) in vec4 aCol;
uniform vec2 uRes; out vec4 vCol;
void main(){ gl_Position=vec4(aPos.x/uRes.x*2.0-1.0, 1.0-aPos.y/uRes.y*2.0, 0.0, 1.0); vCol=aCol; })";
const char* kHudFS = R"(#version 330 core
in vec4 vCol; out vec4 frag; void main(){ frag=vCol; })";

// Classic public-domain 5x7 column font for ASCII 32..90.
const u8 kFont[59][5] = {
    {0, 0, 0, 0, 0},          {0, 0, 0x5F, 0, 0},       {0, 7, 0, 7, 0},          {0x14, 0x7F, 0x14, 0x7F, 0x14},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, {0x23, 0x13, 8, 0x64, 0x62}, {0x36, 0x49, 0x56, 0x20, 0x50}, {0, 8, 7, 3, 0},
    {0, 0x1C, 0x22, 0x41, 0}, {0, 0x41, 0x22, 0x1C, 0}, {0x2A, 0x1C, 0x7F, 0x1C, 0x2A}, {8, 8, 0x3E, 8, 8},
    {0, 0x80, 0x70, 0x30, 0}, {8, 8, 8, 8, 8},          {0, 0, 0x60, 0x60, 0},    {0x20, 0x10, 8, 4, 2},
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0, 0x42, 0x7F, 0x40, 0}, {0x72, 0x49, 0x49, 0x49, 0x46}, {0x21, 0x41, 0x49, 0x4D, 0x33},
    {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39}, {0x3C, 0x4A, 0x49, 0x49, 0x31}, {0x41, 0x21, 0x11, 9, 7},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x46, 0x49, 0x49, 0x29, 0x1E}, {0, 0, 0x14, 0, 0}, {0, 0x40, 0x34, 0, 0},
    {0, 8, 0x14, 0x22, 0x41}, {0x14, 0x14, 0x14, 0x14, 0x14}, {0, 0x41, 0x22, 0x14, 8}, {2, 1, 0x59, 9, 6},
    {0x3E, 0x41, 0x5D, 0x59, 0x4E}, {0x7C, 0x12, 0x11, 0x12, 0x7C}, {0x7F, 0x49, 0x49, 0x49, 0x36}, {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 9, 9, 9, 1}, {0x3E, 0x41, 0x41, 0x51, 0x73},
    {0x7F, 8, 8, 8, 0x7F},    {0, 0x41, 0x7F, 0x41, 0}, {0x20, 0x40, 0x41, 0x3F, 1}, {0x7F, 8, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40}, {0x7F, 2, 0x1C, 2, 0x7F}, {0x7F, 4, 8, 0x10, 0x7F}, {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 9, 9, 9, 6},       {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 9, 0x19, 0x29, 0x46}, {0x26, 0x49, 0x49, 0x49, 0x32},
    {3, 1, 0x7F, 1, 3},       {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F}, {0x3F, 0x40, 0x38, 0x40, 0x3F},
    {0x63, 0x14, 8, 0x14, 0x63}, {3, 4, 0x78, 4, 3},    {0x61, 0x59, 0x49, 0x4D, 0x43}};

struct Vtx {
  float x, y, z, nx, ny, nz;
  u8 r, g, b, a;
};
struct PartV {
  float x, y, z;
  float r, g, b, a;
  float size;
};
struct LineV {
  float x, y, z;
  float r, g, b, a;
};
struct HudV {
  float x, y;
  float r, g, b, a;
};
struct Particle {
  V3 p, v;
  float life, maxLife, size;
  float r, g, b, a;
  float gravity, drag;
};
inline u8 U8(float v) { return u8(std::clamp(v, 0.f, 1.f) * 255.f); }

// Base colour, and in the last channel the finish: 0 matte, up to 0.9 paint or plastic with a clear
// highlight, 1.0 for metal, which mirrors its surroundings.
struct MatDef {
  float r, g, b, gloss;
};
const MatDef kMats[MM_COUNT] = {
    {0.24f, 0.014f, 0.026f, 0.92f},  // paint: Citroen dark metallic red
    {0.012f, 0.02f, 0.024f, 0.90f},  // glass
    {0.016f, 0.016f, 0.017f, 0.18f}, // black plastic
    {0.012f, 0.012f, 0.012f, 0.0f},  // liner
    {0.72f, 0.72f, 0.74f, 1.0f},     // chrome
    {0.30f, 0.32f, 0.33f, 0.90f},    // headlight lens (drawn translucent)
    {0.60f, 0.020f, 0.022f, 0.85f},  // rear light
    {0.75f, 0.33f, 0.03f, 0.85f},    // indicator
    {0.018f, 0.018f, 0.019f, 0.05f}, // tyre rubber
    {0.55f, 0.56f, 0.58f, 1.0f},     // alloy wheel
    {0.10f, 0.095f, 0.09f, 0.0f},    // seat fabric
    {0.04f, 0.04f, 0.04f, 0.0f},     // carpet
    {0.78f, 0.76f, 0.20f, 0.2f},     // number plate
    {0.25f, 0.25f, 0.26f, 0.45f},    // steel
    {0.48f, 0.49f, 0.50f, 0.55f},    // cast aluminium
    {0.09f, 0.33f, 0.10f, 0.70f},    // LHM green spheres
    {0.035f, 0.035f, 0.037f, 0.25f}, // dark grey
    {0.33f, 0.33f, 0.34f, 1.0f},     // brake disc
    {0.72f, 0.72f, 0.70f, 0.85f},    // reversing light
    {0.80f, 0.80f, 0.78f, 1.0f},     // lamp reflector
    {0.20f, 0.13f, 0.09f, 0.1f},     // exhaust steel
};

void propColor(u8 c, float& r, float& g, float& b, float& gloss) {
  gloss = 0;
  switch (c) {
    case COL_STEEL: r = .45f; g = .46f; b = .48f; gloss = .25f; break;
    case COL_SIGN_RED: r = .65f; g = .05f; b = .04f; gloss = .2f; break;
    case COL_SIGN_WHITE: r = .8f; g = .8f; b = .78f; gloss = .2f; break;
    case COL_SIGN_BLUE: r = .05f; g = .14f; b = .5f; gloss = .2f; break;
    case COL_WOOD: r = .36f; g = .22f; b = .11f; break;
    case COL_WOOD_DARK: r = .26f; g = .16f; b = .08f; break;
    case COL_STONE: r = .46f; g = .43f; b = .39f; break;
    case COL_STONE2: r = .38f; g = .36f; b = .33f; break;
    case COL_CONCRETE: r = .55f; g = .55f; b = .52f; break;
    case COL_GUARDRAIL: r = .55f; g = .57f; b = .6f; gloss = .35f; break;
    case COL_CONE: r = .9f; g = .27f; b = .03f; break;
    default: r = g = b = .5f;
  }
}
}  // namespace

struct Renderer::Impl {
  const World* world = nullptr;
  const Sim* sim = nullptr;

  GLuint progWheel = 0, progWheelDepth = 0;
  GLuint progMesh = 0, progDepth = 0, progSky = 0, progPart = 0, progLine = 0, progHud = 0;
  GLuint shadowFbo = 0, shadowTex = 0;
  static constexpr int kShadow = 2048;

  struct Chunk {
    GLuint vao = 0, vbo = 0, ibo = 0;
    GLsizei count = 0;
    V3 center;
  };
  std::vector<Chunk> chunks[2];  // near/far LOD
  GLuint staticVao = 0, staticVbo = 0;
  GLsizei staticCount = 0;  // roads, markings and static boxes
  V3 origin;                // static meshes are stored relative to the world origin

  // Flexbody mesh, deformed on the CPU from the node snapshot every frame.
  const CarRig* rig = nullptr;
  const CarMesh* mesh = nullptr;
  GLuint carVao = 0, carVbo = 0, carIbo = 0, carGlassVao = 0, carGlassIbo = 0;
  std::vector<Vtx> carVerts;
  std::vector<u32> carIdx, glassIdx, lensIdx;
  GLuint lensVao = 0, lensIbo = 0;
  std::vector<float> nrmAccum;
  std::vector<u8> partHidden;
  std::vector<const MeshPart*> wheelParts, wheelStaticParts;
  GLuint wheelVao2 = 0, wheelVbo2 = 0, wheelIbo2 = 0;
  std::vector<int> wheelDrawFirst, wheelDrawCount, wheelDrawMode, staticDrawFirst, staticDrawCount;
  // Unit rod for arms, shafts and struts.
  GLuint rodVao = 0, rodVbo = 0;
  GLsizei rodCount = 0;
  float rodColor[MM_COUNT][4];

  GLuint propVao = 0, propVbo = 0;
  std::vector<Vtx> propVerts;

  GLuint partVao = 0, partVbo = 0;
  std::vector<Particle> particles;
  std::vector<PartV> partVerts;

  GLuint lineVao = 0, lineVbo = 0;
  std::vector<LineV> lineVerts;

  // Skid marks: a ring buffer of triangles lying on the road. They stay until the world is reset.
  static constexpr size_t kMarkVerts = 6 * 60000;
  GLuint markVao = 0, markVbo = 0;
  size_t markNext = 0, markCount = 0;
  struct MarkTrail {
    bool valid = false;
    V3 l, r;
    float a = 0;
  } trail[4];

  void addSkidMarks(const Snapshot& s) {
    constexpr double kHalfTread = 0.085;
    constexpr double kMinStep = 0.06, kMaxStep = 4.0;
    LineV q[6];
    glBindBuffer(GL_ARRAY_BUFFER, markVbo);
    for (int i = 0; i < 4; i++) {
      const WheelView& w = s.wheels[i];
      MarkTrail& t = trail[i];
      if (w.skid < 0.03 || w.groundD > 5.0) {
        t.valid = false;
        continue;
      }
      // Above the road ribbon on roads, just above the ground elsewhere.
      V3 c = w.patch;
      const bool road = w.surface == SURF_ASPHALT;
      c.z += road ? 0.036 : 0.016;
      V3 side = w.axis - V3{0, 0, 1} * w.axis.z;
      side = norm(side) * kHalfTread;
      const V3 l = c + side, r = c - side;
      const float a = float(w.skid) * (road ? 0.72f : 0.8f);
      if (!t.valid) {
        t = {true, l, r, a};
        continue;
      }
      const double step = len((l + r) * 0.5 - (t.l + t.r) * 0.5);
      if (step < kMinStep) continue;
      if (step < kMaxStep) {
        const bool soft = w.surface == SURF_GRASS || w.surface == SURF_GRAVEL;
        const float cr = soft ? 0.13f : 0.012f, cg = soft ? 0.10f : 0.012f, cb = soft ? 0.06f : 0.014f;
        auto v = [&](V3 p, float al) { return LineV{float(p.x), float(p.y), float(p.z), cr, cg, cb, al}; };
        q[0] = v(t.l, t.a);
        q[1] = v(l, a);
        q[2] = v(r, a);
        q[3] = v(t.l, t.a);
        q[4] = v(r, a);
        q[5] = v(t.r, t.a);
        glBufferSubData(GL_ARRAY_BUFFER, GLintptr(markNext * sizeof(LineV)), sizeof q, q);
        markNext = (markNext + 6) % kMarkVerts;
        markCount = std::min(kMarkVerts, markCount + 6);
      }
      t = {true, l, r, a};
    }
  }

  GLuint hudVao = 0, hudVbo = 0;
  std::vector<HudV> hudVerts;

  std::mt19937 rng{1234};
  float rnd(float a, float b) { return a + (b - a) * float(rng() & 0xffff) / 65535.f; }

  V3 sun = norm({0.45, -0.30, 0.84});

  void setupVtxAttribs() {
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)12);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(Vtx), (void*)24);
  }

  void terrainColor(int ix, int iy, float& r, float& g, float& b) {
    const u8 s = world->gridSurface(ix, iy);
    const u32 h = u32(ix) * 374761393u + u32(iy) * 668265263u;
    const float n = float((h ^ (h >> 13)) * 1274126177u >> 24) / 255.f;
    switch (s) {
      case SURF_ASPHALT: r = .10f; g = .10f; b = .105f; break;
      case SURF_CONCRETE: r = .30f + n * .025f; g = .30f + n * .025f; b = .29f + n * .025f; break;
      case SURF_GRAVEL: r = .33f + n * .05f; g = .29f + n * .05f; b = .22f + n * .05f; break;
      case SURF_PAVE: r = .24f + n * .1f; g = .22f + n * .1f; b = .20f + n * .1f; break;
      default: r = .12f + n * .05f; g = .24f + n * .07f; b = .07f + n * .03f; break;
    }
  }

  void buildTerrain() {
    constexpr int kChunk = 64;
    const int N = World::N;
    for (int lod = 0; lod < 2; lod++) {
      const int step = lod == 0 ? 1 : 4;
      for (int cy = 0; cy + kChunk < N; cy += kChunk)
        for (int cx = 0; cx + kChunk < N; cx += kChunk) {
          std::vector<Vtx> v;
          std::vector<u32> idx;
          const int n = kChunk / step + 1;
          auto vert = [&](int ix, int iy, float zOff) {
            Vtx t{};
            t.x = float(ix - World::kHalf);
            t.y = float(iy - World::kHalf);
            t.z = float(world->gridHeight(ix, iy)) + zOff;
            const int xm = std::max(ix - 1, 0), xp = std::min(ix + 1, N - 1);
            const int ym = std::max(iy - 1, 0), yp = std::min(iy + 1, N - 1);
            const V3 nr = norm({world->gridHeight(xm, iy) - world->gridHeight(xp, iy),
                                world->gridHeight(ix, ym) - world->gridHeight(ix, yp), double(xp - xm)});
            t.nx = float(nr.x);
            t.ny = float(nr.y);
            t.nz = float(nr.z);
            float r, g, b;
            terrainColor(ix, iy, r, g, b);
            t.r = U8(r);
            t.g = U8(g);
            t.b = U8(b);
            t.a = 0;
            return t;
          };
          for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++) v.push_back(vert(cx + i * step, cy + j * step, 0));
          for (int j = 0; j + 1 < n; j++)
            for (int i = 0; i + 1 < n; i++) {
              const u32 a = u32(j * n + i), b = a + 1, c = a + u32(n), d = c + 1;
              // Same diagonal as World::height so that the wheels sit on what is drawn.
              idx.insert(idx.end(), {a, b, c, b, d, c});
            }
          // Skirt hides the cracks between LODs.
          auto skirt = [&](int ax, int ay, int dx, int dy) {
            const u32 base = u32(v.size());
            for (int k = 0; k < n; k++) v.push_back(vert(cx + (ax + dx * k) * step, cy + (ay + dy * k) * step, -2.5f));
            for (int k = 0; k + 1 < n; k++) {
              const u32 top0 = u32((ay + dy * k) * n + (ax + dx * k));
              const u32 top1 = u32((ay + dy * (k + 1)) * n + (ax + dx * (k + 1)));
              idx.insert(idx.end(), {top0, top1, base + u32(k), top1, base + u32(k) + 1, base + u32(k)});
            }
          };
          if (lod == 1) {
            skirt(0, 0, 1, 0);
            skirt(0, n - 1, 1, 0);
            skirt(0, 0, 0, 1);
            skirt(n - 1, 0, 0, 1);
          }
          Chunk ch;
          ch.count = GLsizei(idx.size());
          ch.center = {cx + kChunk * 0.5 - World::kHalf, cy + kChunk * 0.5 - World::kHalf, world->gridHeight(cx + kChunk / 2, cy + kChunk / 2)};
          glGenVertexArrays(1, &ch.vao);
          glGenBuffers(1, &ch.vbo);
          glGenBuffers(1, &ch.ibo);
          glBindVertexArray(ch.vao);
          glBindBuffer(GL_ARRAY_BUFFER, ch.vbo);
          glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(v.size() * sizeof(Vtx)), v.data(), GL_STATIC_DRAW);
          glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ch.ibo);
          glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(idx.size() * sizeof(u32)), idx.data(), GL_STATIC_DRAW);
          setupVtxAttribs();
          chunks[lod].push_back(ch);
        }
    }
  }

  void addQuad(std::vector<Vtx>& out, V3 a, V3 b, V3 c, V3 d, float r, float g, float bl, float gloss) {
    V3 n = norm(cross(b - a, c - a));
    auto vx = [&](V3 p) {
      Vtx t{};
      t.x = float(p.x); t.y = float(p.y); t.z = float(p.z);
      t.nx = float(n.x); t.ny = float(n.y); t.nz = float(n.z);
      t.r = U8(r); t.g = U8(g); t.b = U8(bl); t.a = U8(gloss);
      return t;
    };
    for (V3 p : {a, b, c, a, c, d}) out.push_back(vx(p));
  }

  void buildStatic() {
    std::vector<Vtx> v;
    // Road surfaces with edge lines and a dashed centre line.
    for (const Road& road : world->roads()) {
      const size_t cnt = road.pts.size();
      const size_t segs = road.closed ? cnt : cnt - 1;
      for (size_t i = 0; i < segs; i++) {
        const RoadPoint& a = road.pts[i];
        const RoadPoint& b = road.pts[(i + 1) % cnt];
        const double tx = b.x - a.x, ty = b.y - a.y, tl = std::hypot(tx, ty);
        const double nx = -ty / tl, ny = tx / tl;
        auto at = [&](const RoadPoint& p, double off) {
          const double x = p.x + nx * off, y = p.y + ny * off;
          return V3{x, y, world->height(x, y) + 0.025};
        };
        const double w = road.halfWidth;
        addQuad(v, at(a, -w), at(b, -w), at(b, w), at(a, w), .085f, .085f, .09f, 0);
        auto strip = [&](double o0, double o1, float c) {
          V3 p0 = at(a, o0), p1 = at(b, o0), p2 = at(b, o1), p3 = at(a, o1);
          p0.z += 0.006; p1.z += 0.006; p2.z += 0.006; p3.z += 0.006;
          addQuad(v, p0, p1, p2, p3, c, c, c * .97f, 0);
        };
        strip(-w + 0.15, -w + 0.3, .75f);
        strip(w - 0.3, w - 0.15, .75f);
        if ((i / 6) % 2 == 0) strip(-0.07, 0.07, .8f);
      }
    }
    // Test lanes painted on the pad.
    auto line = [&](double x0, double y0, double x1, double y1, double wd, float r, float g, float b) {
      const double dx = x1 - x0, dy = y1 - y0, l = std::hypot(dx, dy);
      const double nx = -dy / l * wd * 0.5, ny = dx / l * wd * 0.5;
      const double z = 0.012;
      addQuad(v, {x0 - nx, y0 - ny, z}, {x1 - nx, y1 - ny, z}, {x1 + nx, y1 + ny, z}, {x0 + nx, y0 + ny, z}, r, g, b, 0);
    };
    for (double yy : {-9.0, 9.0}) line(-110, yy, 148, yy, 0.18, .75f, .72f, .35f);
    for (double yy : {-36.0, -24.0}) line(40, yy, 109, yy, 0.18, .75f, .75f, .75f);
    for (double x0 = -110; x0 < 140; x0 += 8) line(x0, 0, x0 + 3.5, 0, 0.12, .7f, .7f, .7f);

    // Static boxes.
    for (const StaticBox& b : world->boxes()) {
      for (int f = 0; f < 3; f++)
        for (int sgn = -1; sgn <= 1; sgn += 2) {
          const V3 ax = b.ax[f] * (sgn * (f == 0 ? b.h.x : (f == 1 ? b.h.y : b.h.z)));
          const V3 ua = b.ax[(f + 1) % 3] * ((f + 1) % 3 == 0 ? b.h.x : ((f + 1) % 3 == 1 ? b.h.y : b.h.z));
          const V3 va = b.ax[(f + 2) % 3] * ((f + 2) % 3 == 0 ? b.h.x : ((f + 2) % 3 == 1 ? b.h.y : b.h.z));
          const V3 c = b.c + ax;
          if (sgn > 0)
            addQuad(v, c - ua - va, c + ua - va, c + ua + va, c - ua + va, b.r, b.g, b.b, 0);
          else
            addQuad(v, c - ua + va, c + ua + va, c + ua - va, c - ua - va, b.r, b.g, b.b, 0);
        }
    }
    staticCount = GLsizei(v.size());
    glGenVertexArrays(1, &staticVao);
    glGenBuffers(1, &staticVbo);
    glBindVertexArray(staticVao);
    glBindBuffer(GL_ARRAY_BUFFER, staticVbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(v.size() * sizeof(Vtx)), v.data(), GL_STATIC_DRAW);
    setupVtxAttribs();
  }

  bool xrayNow = false;
  static bool isMechanical(const MeshPart& p) {
    return p.group == "engine" || p.group == "gearbox" || p.group == "radiator" || p.group == "battery" || p.group == "tank" ||
           p.name.rfind("spheres", 0) == 0 || p.name.rfind("exhaust", 0) == 0 || p.name.rfind("bay_parts", 0) == 0 || p.name.rfind("steering", 0) == 0;
  }

  // Deform the flexbody from the node snapshot.
  void updateCar(const Snapshot& s) {
    const CarMesh& M = *mesh;
    const size_t nv = M.restPos.size() / 3;
    if (carVerts.size() != nv) {
      carVerts.resize(nv);
      nrmAccum.assign(nv * 3, 0.f);
      for (const MeshPart& p : M.parts) {
        const MatDef& md = kMats[p.mat];
        for (u32 v = p.firstVert; v < p.firstVert + p.vertCount; v++) {
          carVerts[v].r = U8(md.r);
          carVerts[v].g = U8(md.g);
          carVerts[v].b = U8(md.b);
          carVerts[v].a = U8(md.gloss);
        }
      }
    }
    constexpr int K = CarMesh::kK;
    const V3* pos = s.pos.data();
    for (const MeshPart& p : M.parts) {
      if (!p.bound) continue;
      for (u32 v = p.firstVert; v < p.firstVert + p.vertCount; v++) {
        const u32* idx = &M.bindNode[size_t(v) * K];
        const float* wg = &M.bindWeight[size_t(v) * K];
        double x = 0, y = 0, z = 0;
        for (int i = 0; i < K; i++) {
          const V3& q = pos[idx[i]];
          const double w = wg[i];
          x += q.x * w;
          y += q.y * w;
          z += q.z * w;
        }
        carVerts[v].x = float(x);
        carVerts[v].y = float(y);
        carVerts[v].z = float(z);
      }
    }

    // Area-weighted normals from the deformed surface, so that every dent catches the light.
    // Torn triangles are dropped rather than stretched into long rubber sheets. Glass shatters at a
    // strain far below what sheet steel accepts.
    carIdx.clear();
    glassIdx.clear();
    lensIdx.clear();
    std::fill(nrmAccum.begin(), nrmAccum.end(), 0.f);
    constexpr float kTear = 1.45f * 1.45f;
    constexpr float kShatter = 1.04f * 1.04f;
    for (size_t pi = 0; pi < M.parts.size(); pi++) {
      const MeshPart& p = M.parts[pi];
      if (!p.bound) continue;
      const bool isGlass = p.mat == MM_GLASS;
      if (partHidden[pi] || (xrayNow && !isMechanical(p))) continue;
      auto& list = isGlass ? glassIdx : (p.mat == MM_LENS ? lensIdx : carIdx);
      u32 overstrained = 0;
      for (u32 t = p.firstTri; t < p.firstTri + p.triCount; t++) {
        const u32 a = M.tris[t * 3], b = M.tris[t * 3 + 1], c = M.tris[t * 3 + 2];
        const Vtx& A = carVerts[a];
        const Vtx& B = carVerts[b];
        const Vtx& C = carVerts[c];
        const float e1x = B.x - A.x, e1y = B.y - A.y, e1z = B.z - A.z;
        const float e2x = C.x - A.x, e2y = C.y - A.y, e2z = C.z - A.z;
        const float e3x = C.x - B.x, e3y = C.y - B.y, e3z = C.z - B.z;
        const float l1 = e1x * e1x + e1y * e1y + e1z * e1z;
        const float l2 = e3x * e3x + e3y * e3y + e3z * e3z;
        const float l3 = e2x * e2x + e2y * e2y + e2z * e2z;
        const float* rest = &M.restEdge2[size_t(t) * 3];
        if (isGlass) {
          // A pane breaks when a real share of it is stretched, not because of one sliver triangle.
          if (l1 > kShatter * rest[0] + 1e-5f || l2 > kShatter * rest[1] + 1e-5f || l3 > kShatter * rest[2] + 1e-5f) {
            if (++overstrained > std::max(4u, p.triCount / 25)) {
              partHidden[pi] = 1;
              spawnGlass(V3{A.x, A.y, A.z}, s.carVel);
              break;
            }
          }
        } else if (l1 > kTear * rest[0] + 1e-4f || l2 > kTear * rest[1] + 1e-4f || l3 > kTear * rest[2] + 1e-4f) {
          continue;
        }
        const float nx = e1y * e2z - e1z * e2y, ny = e1z * e2x - e1x * e2z, nz = e1x * e2y - e1y * e2x;
        for (u32 q : {a, b, c}) {
          nrmAccum[q * 3] += nx;
          nrmAccum[q * 3 + 1] += ny;
          nrmAccum[q * 3 + 2] += nz;
        }
        list.push_back(a);
        list.push_back(b);
        list.push_back(c);
      }
    }
    for (size_t v = 0; v < nv; v++) {
      const float nx = nrmAccum[v * 3], ny = nrmAccum[v * 3 + 1], nz = nrmAccum[v * 3 + 2];
      const float l = std::sqrt(nx * nx + ny * ny + nz * nz) + 1e-12f;
      carVerts[v].nx = nx / l;
      carVerts[v].ny = ny / l;
      carVerts[v].nz = nz / l;
    }
    glBindBuffer(GL_ARRAY_BUFFER, carVbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(carVerts.size() * sizeof(Vtx)), carVerts.data(), GL_STREAM_DRAW);
    glBindVertexArray(carVao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, carIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(carIdx.size() * sizeof(u32)), carIdx.data(), GL_STREAM_DRAW);
    glBindVertexArray(carGlassVao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, carGlassIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(glassIdx.size() * sizeof(u32)), glassIdx.data(), GL_STREAM_DRAW);
    glBindVertexArray(lensVao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, lensIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(lensIdx.size() * sizeof(u32)), lensIdx.data(), GL_STREAM_DRAW);
  }

  void buildWheelParts() {
    // Wheels are rigid: their parts live in one static buffer and are drawn with a matrix.
    const CarMesh& M = *mesh;
    std::vector<Vtx> v;
    std::vector<u32> idx;
    for (const MeshPart& p : M.parts) {
      if (p.bound) continue;
      const bool isStatic = p.group == "wheel_static";
      if (p.group.rfind("wheel", 0) != 0) continue;
      const u32 base = u32(v.size());
      const MatDef& md = kMats[p.mat];
      for (u32 q = p.firstVert; q < p.firstVert + p.vertCount; q++) {
        Vtx t{};
        t.x = M.restPos[q * 3];
        t.y = M.restPos[q * 3 + 1];
        t.z = M.restPos[q * 3 + 2];
        t.nx = M.restNormal[q * 3];
        t.ny = M.restNormal[q * 3 + 1];
        t.nz = M.restNormal[q * 3 + 2];
        t.r = U8(md.r);
        t.g = U8(md.g);
        t.b = U8(md.b);
        t.a = U8(md.gloss);
        v.push_back(t);
      }
      (isStatic ? staticDrawFirst : wheelDrawFirst).push_back(int(idx.size()));
      for (u32 t = p.firstTri; t < p.firstTri + p.triCount; t++)
        for (int k = 0; k < 3; k++) idx.push_back(M.tris[t * 3 + k] - p.firstVert + base);
      (isStatic ? staticDrawCount : wheelDrawCount).push_back(int(p.triCount * 3));
      if (!isStatic) {
        // The disc and hub stay true; rim and tyre take the damage.
        int mode = 1;
        if (p.name == "wheel_tyre") mode = 2;
        if (p.name.rfind("wheel_disc", 0) == 0 || p.name.rfind("wheel_hub", 0) == 0) mode = 0;
        wheelDrawMode.push_back(mode);
      }
    }
    glGenVertexArrays(1, &wheelVao2);
    glGenBuffers(1, &wheelVbo2);
    glGenBuffers(1, &wheelIbo2);
    glBindVertexArray(wheelVao2);
    glBindBuffer(GL_ARRAY_BUFFER, wheelVbo2);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(v.size() * sizeof(Vtx)), v.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, wheelIbo2);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(idx.size() * sizeof(u32)), idx.data(), GL_STATIC_DRAW);
    setupVtxAttribs();
  }

  void buildRod() {
    // Unit cylinder along +z, radius 1, length 1, with end caps. Colour comes from a uniform-free
    // trick: one copy per material, stored consecutively.
    std::vector<Vtx> v;
    constexpr int kSeg = 12;
    for (int m = 0; m < MM_COUNT; m++) {
      const MatDef& md = kMats[m];
      for (int i = 0; i < kSeg; i++) {
        const float a0 = 2.f * float(PI) * i / kSeg, a1 = 2.f * float(PI) * (i + 1) / kSeg;
        const float c0 = std::cos(a0), s0 = std::sin(a0), c1 = std::cos(a1), s1 = std::sin(a1);
        auto push = [&](float x, float y, float z, float nx, float ny, float nz) {
          Vtx t{};
          t.x = x; t.y = y; t.z = z; t.nx = nx; t.ny = ny; t.nz = nz;
          t.r = U8(md.r); t.g = U8(md.g); t.b = U8(md.b); t.a = U8(md.gloss);
          v.push_back(t);
        };
        push(c0, s0, 0, c0, s0, 0); push(c1, s1, 0, c1, s1, 0); push(c1, s1, 1, c1, s1, 0);
        push(c0, s0, 0, c0, s0, 0); push(c1, s1, 1, c1, s1, 0); push(c0, s0, 1, c0, s0, 0);
        push(0, 0, 1, 0, 0, 1); push(c0, s0, 1, 0, 0, 1); push(c1, s1, 1, 0, 0, 1);
        push(0, 0, 0, 0, 0, -1); push(c1, s1, 0, 0, 0, -1); push(c0, s0, 0, 0, 0, -1);
      }
    }
    rodCount = GLsizei(v.size() / MM_COUNT);
    glGenVertexArrays(1, &rodVao);
    glGenBuffers(1, &rodVbo);
    glBindVertexArray(rodVao);
    glBindBuffer(GL_ARRAY_BUFFER, rodVbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(v.size() * sizeof(Vtx)), v.data(), GL_STATIC_DRAW);
    setupVtxAttribs();
  }

  Mat4 rodMatrix(V3 a, V3 b, float radius) {
    const V3 ax = b - a;
    const V3 z = norm(ax);
    V3 x = norm(cross(z, std::fabs(z.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0}));
    const V3 y = cross(z, x);
    Mat4 m;
    m.m[0] = float(x.x * radius); m.m[1] = float(x.y * radius); m.m[2] = float(x.z * radius);
    m.m[4] = float(y.x * radius); m.m[5] = float(y.y * radius); m.m[6] = float(y.z * radius);
    m.m[8] = float(ax.x); m.m[9] = float(ax.y); m.m[10] = float(ax.z);
    m.m[12] = float(a.x); m.m[13] = float(a.y); m.m[14] = float(a.z);
    m.m[15] = 1;
    return m;
  }

  void updateProps(const Snapshot& s, const V3& camPos) {
    propVerts.clear();
    constexpr double kDraw2 = 500.0 * 500.0;
    const u32 firstProp = sim->objs[0].nodeCount;
    for (const Tri& t : sim->tris) {
      if ((t.flags & TF_NOVIS) || t.a < firstProp) continue;
      const V3& a = s.pos[t.a];
      if (len2(a - camPos) > kDraw2) continue;
      const V3& b = s.pos[t.b];
      const V3& c = s.pos[t.c];
      // Hide triangles whose beams have torn.
      constexpr double kTear2 = 1.5 * 1.5;
      if (len2(b - a) > kTear2 * len2(sim->rest[t.b] - sim->rest[t.a]) ||
          len2(c - b) > kTear2 * len2(sim->rest[t.c] - sim->rest[t.b]) ||
          len2(a - c) > kTear2 * len2(sim->rest[t.a] - sim->rest[t.c]))
        continue;
      const V3 n = norm(cross(b - a, c - a));
      float cr, cg, cb, gloss;
      propColor(t.color, cr, cg, cb, gloss);
      for (const V3* p : {&a, &b, &c}) {
        Vtx v{};
        v.x = float(p->x);
        v.y = float(p->y);
        v.z = float(p->z);
        v.nx = float(n.x);
        v.ny = float(n.y);
        v.nz = float(n.z);
        v.r = U8(cr);
        v.g = U8(cg);
        v.b = U8(cb);
        v.a = U8(gloss);
        propVerts.push_back(v);
      }
    }
    glBindBuffer(GL_ARRAY_BUFFER, propVbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(propVerts.size() * sizeof(Vtx)), propVerts.data(), GL_STREAM_DRAW);
  }

  void spawnGlass(V3 p, V3 v) {
    for (int i = 0; i < 90; i++) {
      Particle q;
      q.p = p + V3{rnd(-.3f, .3f), rnd(-.3f, .3f), rnd(-.2f, .2f)};
      q.v = v * 0.75 + V3{rnd(-2.5f, 2.5f), rnd(-2.5f, 2.5f), rnd(-.5f, 2.5f)};
      q.maxLife = q.life = rnd(0.7f, 1.8f);
      q.size = rnd(.012f, .03f);
      q.r = .75f; q.g = .9f; q.b = .95f; q.a = .85f;
      q.gravity = 9.81f;
      q.drag = 0.2f;
      particles.push_back(q);
    }
  }

  void handleEvents(const Snapshot& s) {
    for (const SimEvent& e : s.events) {
      if (particles.size() > 6000) break;
      switch (e.kind) {
        case EV_SPARK: {
          const int n = 2 + int(rng() % 3);
          for (int i = 0; i < n; i++) {
            Particle q;
            q.p = e.p;
            q.v = e.v * 0.45 + V3{rnd(-2, 2), rnd(-2, 2), rnd(0, 3)};
            q.maxLife = q.life = rnd(.2f, .55f);
            q.size = rnd(.012f, .028f);
            q.r = 1.f; q.g = rnd(.55f, .8f); q.b = .2f; q.a = 1.f;
            q.gravity = 9.81f;
            q.drag = .5f;
            particles.push_back(q);
          }
          break;
        }
        case EV_DUST:
        case EV_TIRE_SMOKE: {
          const bool dust = e.kind == EV_DUST;
          Particle q;
          q.p = e.p + V3{0, 0, 0.05};
          q.v = e.v * 0.15 + V3{rnd(-.4f, .4f), rnd(-.4f, .4f), rnd(.2f, .9f)};
          q.maxLife = q.life = rnd(1.0f, 2.2f);
          q.size = rnd(.25f, .5f);
          if (dust) {
            q.r = .42f; q.g = .36f; q.b = .25f;
          } else {
            q.r = q.g = q.b = .72f;
          }
          q.a = .22f;
          q.gravity = -0.25f;
          q.drag = 1.1f;
          particles.push_back(q);
          break;
        }
        case EV_CRUNCH: {
          for (int i = 0; i < 3; i++) {
            Particle q;
            q.p = e.p;
            q.v = e.v * 0.6 + V3{rnd(-3, 3), rnd(-3, 3), rnd(0, 3)};
            q.maxLife = q.life = rnd(.6f, 1.3f);
            q.size = rnd(.02f, .045f);
            q.r = .2f; q.g = .2f; q.b = .22f; q.a = 1.f;
            q.gravity = 9.81f;
            q.drag = .2f;
            particles.push_back(q);
          }
          break;
        }
        case EV_GLASS:
          spawnGlass(e.p, e.v);
          break;
      }
    }
  }

  void stepParticles(double dt) {
    partVerts.clear();
    for (size_t i = 0; i < particles.size();) {
      Particle& q = particles[i];
      q.life -= float(dt);
      if (q.life <= 0) {
        particles[i] = particles.back();
        particles.pop_back();
        continue;
      }
      q.v.z -= q.gravity * dt;
      q.v *= std::max(0.0, 1.0 - q.drag * dt);
      q.p += q.v * dt;
      const double gh = world->height(q.p.x, q.p.y);
      if (q.p.z < gh) {
        q.p.z = gh;
        q.v.z *= -0.3;
        q.v.x *= 0.6;
        q.v.y *= 0.6;
      }
      const float t = q.life / q.maxLife;
      const float grow = q.gravity < 0 ? 1.f + (1.f - t) * 3.f : 1.f;
      partVerts.push_back({float(q.p.x), float(q.p.y), float(q.p.z), q.r, q.g, q.b, q.a * std::min(1.f, t * 2.f), q.size * grow});
      i++;
    }
    glBindBuffer(GL_ARRAY_BUFFER, partVbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(partVerts.size() * sizeof(PartV)), partVerts.data(), GL_STREAM_DRAW);
  }

  // ---- HUD helpers.
  void rect(float x, float y, float w, float h, float r, float g, float b, float a) {
    const HudV v[6] = {{x, y, r, g, b, a},         {x + w, y, r, g, b, a},     {x + w, y + h, r, g, b, a},
                       {x, y, r, g, b, a},         {x + w, y + h, r, g, b, a}, {x, y + h, r, g, b, a}};
    hudVerts.insert(hudVerts.end(), v, v + 6);
  }
  void text(float x, float y, const char* s, float px, float r, float g, float b, float a = 1.f) {
    float cx = x;
    for (; *s; s++) {
      int c = u8(*s);
      if (c >= 'a' && c <= 'z') c -= 32;
      if (c >= 32 && c <= 90) {
        const u8* gl = kFont[c - 32];
        for (int col = 0; col < 5; col++)
          for (int row = 0; row < 8; row++)
            if (gl[col] & (1 << row)) rect(cx + col * px, y + row * px, px, px, r, g, b, a);
      }
      cx += 6 * px;
    }
  }
  void bar(float x, float y, float w, float h, float v, float r, float g, float b) {
    rect(x, y, w, h, 0, 0, 0, .45f);
    rect(x + 1, y + 1, (w - 2) * float(std::clamp(v, 0.0f, 1.0f)), h - 2, r, g, b, .95f);
  }

  void buildHud(const Snapshot& s, const HudOptions& o, int w, int h) {
    hudVerts.clear();
    const float W = float(w), H = float(h);
    const float scale = std::max(1.f, std::floor(H / 720.f + 0.5f));
    const float px2 = 2 * scale, px3 = 3 * scale, px1 = scale;
    char buf[160];

    // Lower right: speed, gear, rev bar.
    const float bx = W - 300 * scale, by = H - 130 * scale;
    rect(bx - 10 * scale, by - 10 * scale, 300 * scale, 130 * scale, 0, 0, 0, .35f);
    snprintf(buf, sizeof buf, "%3.0f", std::fabs(s.speedKmh));
    text(bx, by, buf, 8 * scale, 1, 1, 1);
    text(bx + 150 * scale, by + 35 * scale, "KM/H", px2, .8f, .8f, .8f);
    snprintf(buf, sizeof buf, "%s%s", s.gear < 0 ? "R" : (s.gear == 0 ? "N" : ""), "");
    if (s.gear > 0) snprintf(buf, sizeof buf, "%d", s.gear);
    text(bx + 220 * scale, by, buf, 7 * scale, s.autoBox ? .6f : 1.f, 1.f, s.autoBox ? 1.f : .4f);
    text(bx + 222 * scale, by + 58 * scale, s.autoBox ? "AUTO" : "MAN", px1 * 2, .7f, .7f, .7f);
    const float rpmN = float(s.rpm / 7000.0);
    bar(bx, by + 76 * scale, 270 * scale, 14 * scale, rpmN, s.rpm > 6100 ? 1.f : .35f, s.rpm > 6100 ? .15f : .8f, .25f);
    snprintf(buf, sizeof buf, "%4.0f U/MIN %2.0FL %s", s.rpm, s.fuel, s.running ? "" : "MOTOR AUS");
    text(bx, by + 94 * scale, buf, px1 * 2, .85f, .85f, .85f);

    // Lower left: Hydractive/Activa state.
    const float lx = 14 * scale;
    float ly = H - 190 * scale;
    rect(lx - 6 * scale, ly - 8 * scale, 290 * scale, 186 * scale, 0, 0, 0, .35f);
    text(lx, ly, "HYDRACTIVE", px2, .75f, .85f, 1);
    text(lx + 130 * scale, ly, s.soft ? "SOFT" : "FIRM", px2, s.soft ? .5f : 1.f, 1.f, s.soft ? .7f : .3f);
    if (s.sport) text(lx + 200 * scale, ly, "SPORT", px2, 1, .6f, .2f);
    ly += 22 * scale;
    text(lx, ly, "ACTIVA", px2, .75f, .85f, 1);
    text(lx + 130 * scale, ly, s.activa ? "AN" : "AUS", px2, s.activa ? .4f : 1.f, s.activa ? 1.f : .3f, .4f);
    ly += 22 * scale;
    static const char* heights[3] = {"NIEDRIG", "STRASSE", "HOCH"};
    snprintf(buf, sizeof buf, "NIVEAU %s", heights[s.heightMode]);
    text(lx, ly, buf, px2, .85f, .85f, .85f);
    ly += 22 * scale;
    snprintf(buf, sizeof buf, "ROLL %+5.1f QUER %+4.2fG", s.roll * 57.3, s.latG);
    text(lx, ly, buf, px2, .85f, .85f, .85f);
    ly += 22 * scale;
    snprintf(buf, sizeof buf, "LHM %3.0F BAR %.1FL", s.accum / 1e5, s.lhm * 1000.0);
    text(lx, ly, buf, px2, s.accum > 90e5 ? .55f : 1.f, s.accum > 90e5 ? .95f : .3f, .55f);
    ly += 22 * scale;
    text(lx, ly, "WASSER", px2, .85f, .85f, .85f);
    bar(lx + 70 * scale, ly, 200 * scale, 14 * scale, float((s.coolant - 40.0) / 95.0), s.coolant > 112 ? 1.f : .4f, s.coolant > 112 ? .2f : .7f, .3f);
    ly += 22 * scale;
    text(lx, ly, "GAS", px1 * 2, .8f, .8f, .8f);
    bar(lx + 40 * scale, ly, 90 * scale, 12 * scale, float(s.throttle), .3f, .9f, .3f);
    text(lx + 140 * scale, ly, "BR", px1 * 2, .8f, .8f, .8f);
    bar(lx + 170 * scale, ly, 100 * scale, 12 * scale, float(s.brake), .95f, .25f, .2f);

    // Upper left: crash telemetry and warnings.
    float ty = 10 * scale;
    snprintf(buf, sizeof buf, "CRASH %5.1f KJ  PEAK %4.1f G", s.plastic / 1e3, s.peakG);
    text(10 * scale, ty, buf, px2, 1, 1, 1);
    ty += 22 * scale;
    auto warn = [&](bool on, const char* label) {
      if (!on) return;
      rect(8 * scale, ty - 3 * scale, float(strlen(label)) * 12 * scale + 8 * scale, 20 * scale, .7f, .05f, .03f, .85f);
      text(12 * scale, ty, label, px2, 1, 1, 1);
      ty += 24 * scale;
    };
    warn(s.hydraulics, "STOP HYDRAULIK");
    warn(s.seized, "MOTOR DEFEKT");
    warn(s.radiator && !s.seized, "KUEHLER LECK");
    warn(s.mount, "MOTORLAGER");
    warn(s.steering, "LENKUNG DEFEKT");
    warn(s.flat[0] || s.flat[1] || s.flat[2] || s.flat[3], "REIFENSCHADEN");
    warn(s.gearboxBad, "GETRIEBE DEFEKT");
    warn(s.tankBad, "TANK LECK");
    warn(s.oilBad, "OELVERLUST");

    // Upper right: status.
    float ry = 10 * scale;
    auto rtext = [&](const char* t, float r, float g, float b) {
      text(W - float(strlen(t)) * 12 * scale - 10 * scale, ry, t, px2, r, g, b);
      ry += 20 * scale;
    };
    snprintf(buf, sizeof buf, "%3.0F FPS  %s", o.fps, o.gpuName.c_str());
    rtext(buf, .75f, .85f, .75f);
    snprintf(buf, sizeof buf, "PHYSIK %3.0F%%  WACH %d", s.physLoad * 100.0, s.awake);
    rtext(buf, .75f, .85f, .75f);
    rtext(o.gamepad ? "CONTROLLER AKTIV" : "TASTATUR/MAUS", .75f, .75f, .75f);
    snprintf(buf, sizeof buf, "KAMERA %s", o.cameraName.c_str());
    rtext(buf, .75f, .75f, .75f);
    if (o.mouseSteer) rtext("MAUSLENKUNG AN", 1, .85f, .35f);
    if (s.timeScale < 0.99) {
      snprintf(buf, sizeof buf, "ZEITLUPE %.2fX", s.timeScale);
      rtext(buf, 1, .85f, .35f);
    }
    if (s.paused) rtext("PAUSE", 1, .85f, .35f);
    if (!o.help) rtext("F1 HILFE", .6f, .6f, .6f);

    // Latest damage message, fading out.
    if (s.damageAge < 4.0 && !s.damageMsg.empty()) {
      const float a = float(std::min(1.0, 4.0 - s.damageAge));
      const float tw = float(s.damageMsg.size()) * 18 * scale;
      text((W - tw) * 0.5f, 60 * scale, s.damageMsg.c_str(), px3, 1, .55f, .25f, a);
    }

    if (o.help) {
      static const char* lines[] = {
          "BEANNG - CITROEN XANTIA ACTIVA",
          "",
          "W/UP GAS   S/DOWN BREMSE (STEHEND: RUECKWAERTS)",
          "A D / LEFT RIGHT LENKEN    SPACE HANDBREMSE",
          "Q E GANG RUNTER/HOCH      T AUTO/MANUELL",
          "G HYDRACTIVE SPORT   H NIVEAU   V ACTIVA AN/AUS",
          "I ZUENDUNG   R ZURUECK ZUM START   F5 REPARIEREN",
          "C KAMERA   M MAUSLENKUNG   P PAUSE   [ ] ZEITLUPE",
          "F2 BEAM-ANSICHT   F3 ROENTGEN   F12 SCREENSHOT   ESC BEENDEN",
          "MAUS: RECHTS ZIEHEN KAMERA, RAD ZOOM, LINKS ZIEHEN PACKT DAS AUTO",
          "",
          "CONTROLLER: L-STICK LENKEN  RT GAS  LT BREMSE",
          "R-STICK KAMERA  A HANDBREMSE  LB/RB GANG  Y KAMERA",
          "X SPORT  B REPARIEREN  BACK START  START PAUSE",
      };
      const float hw = 720 * scale, hh = float(sizeof(lines) / sizeof(lines[0])) * 22 * scale + 24 * scale;
      const float hx = (W - hw) * 0.5f, hy = (H - hh) * 0.5f;
      rect(hx, hy, hw, hh, 0, 0, 0, .72f);
      float yy = hy + 12 * scale;
      for (const char* l : lines) {
        text(hx + 14 * scale, yy, l, px2, .95f, .95f, .95f);
        yy += 22 * scale;
      }
    }
    glBindBuffer(GL_ARRAY_BUFFER, hudVbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(hudVerts.size() * sizeof(HudV)), hudVerts.data(), GL_STREAM_DRAW);
  }

  void setMat(GLuint p, const char* n, const Mat4& m) {
    glUniformMatrix4fv(glGetUniformLocation(p, n), 1, GL_FALSE, m.m);
  }
  // Wheel frame without its rotation: x and z span the wheel plane, y is the axle pointing outboard.
  void wheelAxes(const WheelView& wv, V3& ex, V3& ey, V3& ez) {
    ey = wv.axis * (wv.left ? 1.0 : -1.0);
    ex = norm(wv.up - wv.axis * dot(wv.up, wv.axis));
    ez = cross(ex, ey);
  }
  Mat4 wheelMatrix(const WheelView& wv) {
    V3 ex, ey, ez;
    wheelAxes(wv, ex, ey, ez);
    Mat4 m;
    m.m[0] = float(ex.x); m.m[1] = float(ex.y); m.m[2] = float(ex.z);
    m.m[4] = float(ey.x); m.m[5] = float(ey.y); m.m[6] = float(ey.z);
    m.m[8] = float(ez.x); m.m[9] = float(ez.y); m.m[10] = float(ez.z);
    m.m[12] = float(wv.hub.x); m.m[13] = float(wv.hub.y); m.m[14] = float(wv.hub.z);
    m.m[15] = 1;
    return m;
  }
  void drawWheels(const Snapshot& s, GLuint prog, const Mat4& vp, const Mat4& shadowVP) {
    glUseProgram(prog);
    setMat(prog, "uVP", vp);
    setMat(prog, "uShadowVP", shadowVP);
    glBindVertexArray(wheelVao2);
    const GLint uMode = glGetUniformLocation(prog, "uMode"), uSpin = glGetUniformLocation(prog, "uSpin");
    for (const WheelView& wv : s.wheels) {
      V3 ex, ey, ez;
      wheelAxes(wv, ex, ey, ez);
      setMat(prog, "uModel", wheelMatrix(wv));
      glUniform1fv(glGetUniformLocation(prog, "uDent"), 24, wv.dent);
      glUniform2f(glGetUniformLocation(prog, "uRunout"), wv.runoutX, wv.runoutZ);
      glUniform3f(glGetUniformLocation(prog, "uPlaneN"), float(dot(wv.groundN, ex)), float(dot(wv.groundN, ey)), float(dot(wv.groundN, ez)));
      glUniform1f(glGetUniformLocation(prog, "uPlaneD"), float(wv.groundD));
      glUniform1f(glGetUniformLocation(prog, "uDeflate"), float(wv.deflate));
      const float sn = float(std::sin(wv.theta) * (wv.left ? 1.0 : -1.0));
      glUniform2f(uSpin, float(std::cos(wv.theta)), sn);
      for (size_t i = 0; i < wheelDrawFirst.size(); i++) {
        glUniform1i(uMode, wheelDrawMode[i]);
        glDrawElements(GL_TRIANGLES, wheelDrawCount[i], GL_UNSIGNED_INT, (void*)(size_t(wheelDrawFirst[i]) * sizeof(u32)));
      }
      // Caliper: fixed to the hub, it does not turn.
      glUniform1i(uMode, 0);
      glUniform2f(uSpin, 1.f, 0.f);
      for (size_t i = 0; i < staticDrawFirst.size(); i++)
        glDrawElements(GL_TRIANGLES, staticDrawCount[i], GL_UNSIGNED_INT, (void*)(size_t(staticDrawFirst[i]) * sizeof(u32)));
    }
  }

  Mat4 identity() {
    Mat4 m;
    m.m[0] = m.m[5] = m.m[10] = m.m[15] = 1;
    return m;
  }
};

bool Renderer::init(const World& world, const Sim& sim, const CarRig& rig, const CarMesh& mesh) {
  d = new Impl();
  d->world = &world;
  d->sim = &sim;
  d->rig = &rig;
  d->mesh = &mesh;
  d->partHidden.assign(mesh.parts.size(), 0);

  d->progMesh = program(kMeshVS, kMeshFS);
  d->progDepth = program(kDepthVS, kDepthFS);
  d->progWheel = program(kWheelVS, kMeshFS);
  d->progWheelDepth = program(kWheelVS, kDepthFS);
  d->progSky = program(kSkyVS, kSkyFS);
  d->progPart = program(kPartVS, kPartFS);
  d->progLine = program(kLineVS, kLineFS);
  d->progHud = program(kHudVS, kHudFS);

  glGenFramebuffers(1, &d->shadowFbo);
  glGenTextures(1, &d->shadowTex);
  glBindTexture(GL_TEXTURE_2D, d->shadowTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, Impl::kShadow, Impl::kShadow, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  glBindFramebuffer(GL_FRAMEBUFFER, d->shadowFbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, d->shadowTex, 0);
  glDrawBuffer(GL_NONE);
  glReadBuffer(GL_NONE);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  d->buildTerrain();
  d->buildStatic();
  d->buildWheelParts();
  d->buildRod();

  glGenVertexArrays(1, &d->carVao);
  glGenVertexArrays(1, &d->carGlassVao);
  glGenBuffers(1, &d->carVbo);
  glGenBuffers(1, &d->carIbo);
  glGenBuffers(1, &d->carGlassIbo);
  glBindBuffer(GL_ARRAY_BUFFER, d->carVbo);
  glBindVertexArray(d->carVao);
  d->setupVtxAttribs();
  glBindVertexArray(d->carGlassVao);
  d->setupVtxAttribs();
  glGenVertexArrays(1, &d->lensVao);
  glGenBuffers(1, &d->lensIbo);
  glBindVertexArray(d->lensVao);
  d->setupVtxAttribs();

  glGenVertexArrays(1, &d->propVao);
  glGenBuffers(1, &d->propVbo);
  glBindVertexArray(d->propVao);
  glBindBuffer(GL_ARRAY_BUFFER, d->propVbo);
  d->setupVtxAttribs();

  glGenVertexArrays(1, &d->partVao);
  glGenBuffers(1, &d->partVbo);
  glBindVertexArray(d->partVao);
  glBindBuffer(GL_ARRAY_BUFFER, d->partVbo);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PartV), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(PartV), (void*)12);
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(PartV), (void*)28);

  glGenVertexArrays(1, &d->lineVao);
  glGenBuffers(1, &d->lineVbo);
  glBindVertexArray(d->lineVao);
  glBindBuffer(GL_ARRAY_BUFFER, d->lineVbo);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(LineV), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(LineV), (void*)12);

  glGenVertexArrays(1, &d->markVao);
  glGenBuffers(1, &d->markVbo);
  glBindVertexArray(d->markVao);
  glBindBuffer(GL_ARRAY_BUFFER, d->markVbo);
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(Impl::kMarkVerts * sizeof(LineV)), nullptr, GL_DYNAMIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(LineV), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(LineV), (void*)12);

  glGenVertexArrays(1, &d->hudVao);
  glGenBuffers(1, &d->hudVbo);
  glBindVertexArray(d->hudVao);
  glBindBuffer(GL_ARRAY_BUFFER, d->hudVbo);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(HudV), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(HudV), (void*)8);
  glBindVertexArray(0);

  glEnable(GL_PROGRAM_POINT_SIZE);
  return true;
}

void Renderer::clearSkidMarks() {
  d->markNext = d->markCount = 0;
  for (auto& t : d->trail) t.valid = false;
}

void Renderer::resetDamage() {
  std::fill(d->partHidden.begin(), d->partHidden.end(), u8(0));
  d->particles.clear();
}

void Renderer::draw(const Snapshot& s, const Camera& cam, const HudOptions& hud, int w, int h, double dt) {
  Impl& D = *d;
  D.origin = {};  // positions stay within +-512 m, which is fine for float vertex data
  D.handleEvents(s);
  D.xrayNow = hud.xray;
  D.updateCar(s);
  D.updateProps(s, cam.eye);
  D.stepParticles(dt);
  D.addSkidMarks(s);
  D.buildHud(s, hud, w, h);

  // Eye-relative view would be more precise, but +-500 m with 1 mm float steps is good enough.
  const Mat4 view = lookAt(cam.eye, cam.target, cam.up);
  const Mat4 proj = perspective(cam.fovY, double(w) / double(h), 0.12, 2500.0);
  const Mat4 vp = mul(proj, view);

  // Shadow map: a tight box following the car, for sharp contact shadows where it matters.
  constexpr double kRange = 42.0;
  // Snap the shadow camera to whole texels, or the shadow edges would crawl with every movement.
  V3 focus = s.carCog;
  {
    const V3 lr = norm(cross(D.sun, V3{0, 0, 1}));
    const V3 lu = cross(lr, D.sun);
    const double texel = 2.0 * kRange / Impl::kShadow;
    const double a = std::round(dot(focus, lr) / texel) * texel, b = std::round(dot(focus, lu) / texel) * texel, c = dot(focus, D.sun);
    focus = lr * a + lu * b + D.sun * c;
  }
  const V3 lightEye = focus + D.sun * 120.0;
  const Mat4 lightView = lookAt(lightEye, focus, V3{0, 0, 1});
  const Mat4 lightVP = mul(ortho(-kRange, kRange, -kRange, kRange, 1.0, 260.0), lightView);
  const Mat4 I = D.identity();

  auto drawDyn = [&](GLuint prog) {
    D.setMat(prog, "uModel", I);
    glBindVertexArray(D.carVao);
    glDrawElements(GL_TRIANGLES, GLsizei(D.carIdx.size()), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(D.propVao);
    glDrawArrays(GL_TRIANGLES, 0, GLsizei(D.propVerts.size()));
    // Running gear: arms, shafts and struts between their real nodes.
    glBindVertexArray(D.rodVao);
    for (const VisLink& l : D.rig->links) {
      const V3 a = s.pos[l.a], b = s.pos[l.b];
      const double restL = len(D.sim->rest[l.b] - D.sim->rest[l.a]);
      if (len(b - a) > restL * 1.6) continue;  // torn off
      if (l.kind == 1) {
        // Strut: slim chrome rod from the top, body over the lower part.
        const V3 mid = lerpv(a, b, 0.42);
        D.setMat(prog, "uModel", D.rodMatrix(a, mid, l.radius * 0.45f));
        glDrawArrays(GL_TRIANGLES, MM_CHROME * D.rodCount, D.rodCount);
        D.setMat(prog, "uModel", D.rodMatrix(mid, b, l.radius));
      } else {
        D.setMat(prog, "uModel", D.rodMatrix(a, b, l.radius));
      }
      glDrawArrays(GL_TRIANGLES, l.mat * D.rodCount, D.rodCount);
    }
    D.setMat(prog, "uModel", I);
  };

  glEnable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glDisable(GL_CULL_FACE);

  glBindFramebuffer(GL_FRAMEBUFFER, D.shadowFbo);
  glViewport(0, 0, Impl::kShadow, Impl::kShadow);
  glClear(GL_DEPTH_BUFFER_BIT);
  glUseProgram(D.progDepth);
  D.setMat(D.progDepth, "uVP", lightVP);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(2.0f, 3.0f);
  drawDyn(D.progDepth);
  D.setMat(D.progDepth, "uModel", I);
  glBindVertexArray(D.staticVao);
  glDrawArrays(GL_TRIANGLES, 0, D.staticCount);
  D.drawWheels(s, D.progWheelDepth, lightVP, lightVP);
  glDisable(GL_POLYGON_OFFSET_FILL);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  glViewport(0, 0, w, h);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  // Sky.
  glDisable(GL_DEPTH_TEST);
  glUseProgram(D.progSky);
  {
    const V3 f = norm(cam.target - cam.eye), sv = norm(cross(f, cam.up)), u = cross(sv, f);
    const float rot[9] = {float(sv.x), float(sv.y), float(sv.z), float(u.x), float(u.y), float(u.z), float(-f.x), float(-f.y), float(-f.z)};
    glUniformMatrix3fv(glGetUniformLocation(D.progSky, "uInvRot"), 1, GL_FALSE, rot);
    glUniform1f(glGetUniformLocation(D.progSky, "uTanY"), float(std::tan(cam.fovY * 0.5)));
    glUniform1f(glGetUniformLocation(D.progSky, "uAspect"), float(w) / float(h));
    glUniform3f(glGetUniformLocation(D.progSky, "uSun"), float(D.sun.x), float(D.sun.y), float(D.sun.z));
    glBindVertexArray(D.hudVao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
  }
  glEnable(GL_DEPTH_TEST);

  // Main pass.
  glUseProgram(D.progMesh);
  D.setMat(D.progMesh, "uVP", vp);
  D.setMat(D.progMesh, "uShadowVP", lightVP);
  D.setMat(D.progMesh, "uModel", I);
  glUniform3f(glGetUniformLocation(D.progMesh, "uEye"), float(cam.eye.x), float(cam.eye.y), float(cam.eye.z));
  glUniform3f(glGetUniformLocation(D.progMesh, "uSun"), float(D.sun.x), float(D.sun.y), float(D.sun.z));
  glUniform1f(glGetUniformLocation(D.progMesh, "uAlpha"), 1.f);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, D.shadowTex);
  glUniform1i(glGetUniformLocation(D.progMesh, "uShadow"), 0);

  // Terrain chunks by distance, with the far LOD beyond 220 m.
  const V3 viewDir = norm(cam.target - cam.eye);
  for (size_t i = 0; i < D.chunks[0].size(); i++) {
    const V3 c = D.chunks[0][i].center;
    const V3 toC = c - cam.eye;
    const double dist = len(toC);
    if (dist > 90.0 && dot(toC, viewDir) / dist < 0.2) continue;  // cheap view culling
    const Impl::Chunk& ch = dist < 220.0 ? D.chunks[0][i] : D.chunks[1][i];
    glBindVertexArray(ch.vao);
    glDrawElements(GL_TRIANGLES, ch.count, GL_UNSIGNED_INT, nullptr);
  }
  glBindVertexArray(D.staticVao);
  glDrawArrays(GL_TRIANGLES, 0, D.staticCount);

  drawDyn(D.progMesh);
  glUseProgram(D.progWheel);
  glUniform3f(glGetUniformLocation(D.progWheel, "uEye"), float(cam.eye.x), float(cam.eye.y), float(cam.eye.z));
  glUniform3f(glGetUniformLocation(D.progWheel, "uSun"), float(D.sun.x), float(D.sun.y), float(D.sun.z));
  glUniform1f(glGetUniformLocation(D.progWheel, "uAlpha"), 1.f);
  glUniform1i(glGetUniformLocation(D.progWheel, "uShadow"), 0);
  D.drawWheels(s, D.progWheel, vp, lightVP);

  // Debug overlay: every beam, coloured by load.
  if (hud.debugBeams) {
    D.lineVerts.clear();
    const u32 carBeams = D.sim->objs[0].beamCount;
    for (u32 i = 0; i < carBeams && i < s.beamForce.size(); i++) {
      const float f = s.beamForce[i];
      if (f == 0.f) continue;
      const Beam& b = D.sim->beams[i];
      const float t = std::clamp(std::fabs(f) / 12000.f, 0.f, 1.f);
      const float r = f > 0 ? t : 0.15f, bl = f < 0 ? t : 0.15f, g = 0.9f - t * 0.75f;
      const V3& A = s.pos[b.a];
      const V3& B = s.pos[b.b];
      D.lineVerts.push_back({float(A.x), float(A.y), float(A.z), r, g, bl, .9f});
      D.lineVerts.push_back({float(B.x), float(B.y), float(B.z), r, g, bl, .9f});
    }
    glBindBuffer(GL_ARRAY_BUFFER, D.lineVbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(D.lineVerts.size() * sizeof(LineV)), D.lineVerts.data(), GL_STREAM_DRAW);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(D.progLine);
    D.setMat(D.progLine, "uVP", vp);
    glBindVertexArray(D.lineVao);
    glDrawArrays(GL_LINES, 0, GLsizei(D.lineVerts.size()));
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
  }

  // Transparent things last: skid marks, glass and particles.
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthMask(GL_FALSE);
  if (D.markCount > 0) {
    glUseProgram(D.progLine);
    D.setMat(D.progLine, "uVP", vp);
    glBindVertexArray(D.markVao);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-1.0f, -2.0f);
    glDrawArrays(GL_TRIANGLES, 0, GLsizei(D.markCount));
    glDisable(GL_POLYGON_OFFSET_FILL);
  }
  glUseProgram(D.progMesh);
  D.setMat(D.progMesh, "uModel", I);
  glUniform1f(glGetUniformLocation(D.progMesh, "uAlpha"), 0.30f);
  glBindVertexArray(D.lensVao);
  glDrawElements(GL_TRIANGLES, GLsizei(D.lensIdx.size()), GL_UNSIGNED_INT, nullptr);
  glUniform1f(glGetUniformLocation(D.progMesh, "uAlpha"), 0.80f);
  glBindVertexArray(D.carGlassVao);
  glDrawElements(GL_TRIANGLES, GLsizei(D.glassIdx.size()), GL_UNSIGNED_INT, nullptr);

  glUseProgram(D.progPart);
  D.setMat(D.progPart, "uVP", vp);
  glUniform1f(glGetUniformLocation(D.progPart, "uScale"), float(h) * 1.2f);
  glBindVertexArray(D.partVao);
  glDrawArrays(GL_POINTS, 0, GLsizei(D.partVerts.size()));
  glDepthMask(GL_TRUE);

  // HUD.
  glDisable(GL_DEPTH_TEST);
  glUseProgram(D.progHud);
  glUniform2f(glGetUniformLocation(D.progHud, "uRes"), float(w), float(h));
  glBindVertexArray(D.hudVao);
  glDrawArrays(GL_TRIANGLES, 0, GLsizei(D.hudVerts.size()));
  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  glBindVertexArray(0);
}

int Renderer::pickNode(const Snapshot& s, const Camera& cam, int w, int h, double mx, double my, double maxPx) const {
  const Mat4 vp = mul(perspective(cam.fovY, double(w) / double(h), 0.12, 2500.0), lookAt(cam.eye, cam.target, cam.up));
  const u32 n = d->sim->objs[0].nodeCount;
  int best = -1;
  double bestD = maxPx * maxPx;
  for (u32 i = 0; i < n; i++) {
    const V3& p = s.pos[i];
    const double cx = vp.m[0] * p.x + vp.m[4] * p.y + vp.m[8] * p.z + vp.m[12];
    const double cy = vp.m[1] * p.x + vp.m[5] * p.y + vp.m[9] * p.z + vp.m[13];
    const double cw = vp.m[3] * p.x + vp.m[7] * p.y + vp.m[11] * p.z + vp.m[15];
    if (cw <= 0.01) continue;
    const double sx = (cx / cw * 0.5 + 0.5) * w, sy = (0.5 - cy / cw * 0.5) * h;
    const double dd = (sx - mx) * (sx - mx) + (sy - my) * (sy - my);
    if (dd < bestD) {
      bestD = dd;
      best = int(i);
    }
  }
  return best;
}

V3 Renderer::unprojectOnPlane(const Camera& cam, int w, int h, double mx, double my, V3 planePoint) const {
  const V3 f = norm(cam.target - cam.eye), sv = norm(cross(f, cam.up)), u = cross(sv, f);
  const double t = std::tan(cam.fovY * 0.5);
  const double nx = (mx / w * 2.0 - 1.0) * t * double(w) / h, ny = (1.0 - my / h * 2.0) * t;
  const V3 ray = norm(f + sv * nx + u * ny);
  const double dn = dot(ray, f);
  const double dist = dn > 1e-3 ? dot(planePoint - cam.eye, f) / dn : 10.0;
  return cam.eye + ray * dist;
}

bool Renderer::savePng(const std::string& path, int w, int h) const {
  std::vector<u8> px(size_t(w) * h * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadBuffer(GL_BACK);
  glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());

  // Uncompressed PNG: stored deflate blocks. Needs no dependency, and screenshots are rare.
  static u32 crcTable[256];
  static bool crcReady = false;
  if (!crcReady) {
    for (u32 n = 0; n < 256; n++) {
      u32 c = n;
      for (int k = 0; k < 8; k++) c = (c & 1) ? 0xedb88320u ^ (c >> 1) : c >> 1;
      crcTable[n] = c;
    }
    crcReady = true;
  }
  FILE* f = fopen(path.c_str(), "wb");
  if (!f) return false;
  auto be32 = [](u32 v, u8* o) { o[0] = u8(v >> 24); o[1] = u8(v >> 16); o[2] = u8(v >> 8); o[3] = u8(v); };
  auto chunk = [&](const char* type, const std::vector<u8>& data) {
    u8 b[4];
    be32(u32(data.size()), b);
    fwrite(b, 1, 4, f);
    fwrite(type, 1, 4, f);
    if (!data.empty()) fwrite(data.data(), 1, data.size(), f);
    u32 c = 0xffffffffu;
    for (int i = 0; i < 4; i++) c = crcTable[(c ^ u8(type[i])) & 0xff] ^ (c >> 8);
    for (u8 v : data) c = crcTable[(c ^ v) & 0xff] ^ (c >> 8);
    be32(c ^ 0xffffffffu, b);
    fwrite(b, 1, 4, f);
  };
  const u8 sig[8] = {0x89, 'P', 'N', 'G', 13, 10, 26, 10};
  fwrite(sig, 1, 8, f);
  std::vector<u8> ihdr(13);
  be32(u32(w), &ihdr[0]);
  be32(u32(h), &ihdr[4]);
  ihdr[8] = 8;
  ihdr[9] = 2;
  chunk("IHDR", ihdr);

  std::vector<u8> raw;
  raw.reserve(size_t(w * 3 + 1) * h);
  for (int y = h - 1; y >= 0; y--) {
    raw.push_back(0);
    raw.insert(raw.end(), px.begin() + std::ptrdiff_t(y) * w * 3, px.begin() + std::ptrdiff_t(y + 1) * w * 3);
  }
  std::vector<u8> z = {0x78, 0x01};
  u32 a1 = 1, a2 = 0;
  for (u8 v : raw) {
    a1 = (a1 + v) % 65521;
    a2 = (a2 + a1) % 65521;
  }
  for (size_t pos = 0; pos < raw.size();) {
    const size_t n = std::min<size_t>(65535, raw.size() - pos);
    z.push_back(pos + n == raw.size() ? 1 : 0);
    z.push_back(u8(n));
    z.push_back(u8(n >> 8));
    z.push_back(u8(~n));
    z.push_back(u8((~n) >> 8));
    z.insert(z.end(), raw.begin() + std::ptrdiff_t(pos), raw.begin() + std::ptrdiff_t(pos + n));
    pos += n;
  }
  const u32 ad = (a2 << 16) | a1;
  z.push_back(u8(ad >> 24));
  z.push_back(u8(ad >> 16));
  z.push_back(u8(ad >> 8));
  z.push_back(u8(ad));
  chunk("IDAT", z);
  chunk("IEND", {});
  fclose(f);
  return true;
}
