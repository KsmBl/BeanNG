#include "render.h"
#include <epoxy/gl.h>
#include <algorithm>
#include <chrono>
#include <cstdlib>
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
layout(location=3) in vec3 aRest;
layout(location=4) in vec3 aRestN;
uniform mat4 uVP;
uniform mat4 uModel;
uniform mat4 uShadowVP;
uniform mat4 uPrevVP;
uniform mat4 uPrevWorld;  // where this surface was one shutter time ago, relative to where it is now
out vec3 vPos; out vec3 vNrm; out vec4 vCol; out vec4 vShadow; out vec3 vRest; out vec3 vRestN;
out vec4 vClip; out vec4 vPrevClip;
void main(){
  vRest = aRest;
  vRestN = aRestN;
  vec4 w = uModel * vec4(aPos,1.0);
  vPos = w.xyz;
  vNrm = mat3(uModel) * aNrm;
  vCol = aCol;
  vShadow = uShadowVP * w;
  gl_Position = uVP * w;
  vClip = gl_Position;
  vPrevClip = uPrevVP * uPrevWorld * w;
})";

// aCol.a encodes the material: <0.5 matte, 0.5..0.9 paint with sky reflection, 1 = opaque glossy
const char* kMeshFS = R"(#version 330 core
in vec3 vPos; in vec3 vNrm; in vec4 vCol; in vec4 vShadow; in vec3 vRest; in vec3 vRestN; in vec4 vClip; in vec4 vPrevClip;
uniform vec3 uEye;
uniform vec3 uSun;
uniform sampler2DShadow uShadow;
uniform sampler2DShadow uShadowFar;
uniform mat4 uShadowFarVP;
uniform float uAlpha;
uniform sampler2DArray uScratchTex;
uniform sampler2D uDetailTex;
uniform float uDetail;
uniform float uScorch;
layout(location=0) out vec4 frag;
layout(location=1) out vec2 velOut;
vec3 sky(vec3 d){
  float t = clamp(d.z*0.5+0.5,0.0,1.0);
  vec3 c = mix(vec3(0.62,0.60,0.56), vec3(0.32,0.52,0.82), pow(t,0.7));
  if(d.z<0.0) c = mix(vec3(0.33,0.32,0.28), c, clamp(1.0+d.z*6.0,0.0,1.0));
  c += vec3(1.0,0.9,0.75)*pow(max(dot(d,uSun),0.0),180.0)*2.5;
  return c;
}
// Scratches live in five flat maps of the car: left, right, top, nose and tail. They are real painted
// lines with mipmaps, so that from afar they fade out smoothly instead of turning into noise.
vec2 scratchAt(){
  if(dot(vRestN,vRestN) < 0.25) return vec2(0.0);
  vec3 n = abs(vRestN);
  vec3 dx = dFdx(vRest), dy = dFdy(vRest);
  vec2 uv, gx, gy; float layer;
  const vec3 org = vec3(0.1, 1.0, 0.0);
  const vec3 size = vec3(4.8, 2.0, 1.6);
  if(n.y >= n.x && n.y >= n.z*0.9){
    layer = vRestN.y > 0.0 ? 0.0 : 1.0;
    uv = vec2(vRest.x+org.x, vRest.z)/size.xz; gx = dx.xz/size.xz; gy = dy.xz/size.xz;
  } else if(n.z >= n.x){
    if(vRestN.z < 0.0) return vec2(0.0);
    layer = 2.0;
    uv = vec2(vRest.x+org.x, vRest.y+org.y)/size.xy; gx = dx.xy/size.xy; gy = dy.xy/size.xy;
  } else {
    layer = vRestN.x < 0.0 ? 3.0 : 4.0;
    uv = vec2(vRest.y+org.y, vRest.z)/size.yz; gx = dx.yz/size.yz; gy = dy.yz/size.yz;
  }
  return textureGrad(uScratchTex, vec3(uv, layer), gx, gy).rg;
}
float shadowIn(sampler2DShadow map, vec3 p, float texel, float bias){
  float s=0.0;
  for(int x=-1;x<=1;x++) for(int y=-1;y<=1;y++)
    s += texture(map, vec3(p.xy+vec2(x,y)*texel, p.z-bias));
  return s/9.0;
}
// Two maps: a sharp one around the car, and a coarse one reaching far out over the landscape.
float shadow(){
  vec3 p = vShadow.xyz/vShadow.w*0.5+0.5;
  float near = 1.0;
  float edge = 2.0*max(abs(p.x-0.5), abs(p.y-0.5));
  bool inNear = edge < 1.0 && p.z <= 1.0;
  if(inNear) near = shadowIn(uShadow, p, 1.0/2048.0, 0.0009);
  if(inNear && edge < 0.85) return near;
  vec4 f = uShadowFarVP*vec4(vPos,1.0);
  vec3 q = f.xyz/f.w*0.5+0.5;
  float edgeF = 2.0*max(abs(q.x-0.5), abs(q.y-0.5));
  float far = 1.0;
  if(edgeF < 1.0 && q.z <= 1.0) far = mix(shadowIn(uShadowFar, q, 1.0/4096.0, 0.0007), 1.0, smoothstep(0.85, 1.0, edgeF));
  return inNear ? mix(near, far, smoothstep(0.85, 1.0, edge)) : far;
}
void main(){
  vec3 N = normalize(vNrm);
  vec3 V = normalize(uEye - vPos);
  float gloss = vCol.a;
  vec3 albedo = vCol.rgb;
  float flash = 0.0;
  if(dot(N,V)<0.0){ N=-N; if(gloss>0.05) gloss=0.0; }
  else {
    vec2 sc = scratchAt();
    if(sc.g > 0.002){
      // Scuffed clear coat: duller and a little lighter, with some of the obstacle rubbed on.
      gloss *= 1.0 - 0.75*sc.g;
      albedo = mix(albedo, albedo*0.72 + vec3(0.10), sc.g);
    }
    if(sc.r > 0.004){
      // Through the paint to grey primer, and in the deep gouges to bare steel.
      float g = clamp(sc.r*1.6, 0.0, 1.0);
      float steel = smoothstep(0.42, 0.72, sc.r);
      vec3 bare = mix(vec3(0.42,0.41,0.39), vec3(0.66,0.67,0.69), steel);
      albedo = mix(albedo, bare, g);
      gloss *= 1.0 - g;
      flash = 0.25*steel*g;
    }
  }
  if(uDetail > 0.0 && N.z > 0.5){
    // Patches and grit on the ground. Without them the road gives the eye nothing to measure speed by.
    float fine = texture(uDetailTex, vPos.xy/3.7).r;
    float wide = texture(uDetailTex, vPos.xy/41.0 + 0.37).g;
    albedo *= uDetail*(0.62 + 0.55*fine + 0.45*wide) + (1.0 - uDetail);
  }
  if(uScorch > 0.0){
    // Burnt out: paint gone to soot and scale, with some patches lasting longer than others.
    float n = texture(uDetailTex, vRest.xy/1.3 + vRest.zz*0.37).g;
    float burn = clamp(uScorch*(0.75 + 1.1*n), 0.0, 1.0);
    albedo = mix(albedo, vec3(0.018,0.016,0.015) + 0.05*vec3(0.5,0.32,0.2)*n, burn);
    gloss *= 1.0 - burn;
  }
  float diff = max(dot(N,uSun),0.0)*shadow();
  vec3 amb = mix(vec3(0.20,0.19,0.17), vec3(0.34,0.40,0.50), N.z*0.5+0.5);
  vec3 c = albedo*(amb + diff*vec3(1.05,1.0,0.92)) + flash*diff*vec3(1.0);
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
  // How far this point moves across the screen during the shutter time, for the motion blur.
  velOut = vPrevClip.w > 0.05 ? (vClip.xy/vClip.w - vPrevClip.xy/vPrevClip.w)*0.5 : vec2(0.0);
})";

// Motion blur: smears every pixel along its own motion on screen. A blurred background must not pick
// up the sharp car in front of it, so samples nearer than this pixel only count if they really move
// that far themselves.
const char* kPostVS = R"(#version 330 core
uniform mat4 uSkyVP; uniform mat4 uSkyPrevVP;
out vec2 vUv; flat out mat4 vSkyReproject;
void main(){
  vec2 p = vec2((gl_VertexID==1)?3.0:-1.0, (gl_VertexID==2)?3.0:-1.0);
  vUv = p*0.5+0.5;
  vSkyReproject = uSkyPrevVP*inverse(uSkyVP);
  gl_Position = vec4(p,0.0,1.0);
})";
const char* kPostFS = R"(#version 330 core
in vec2 vUv; flat in mat4 vSkyReproject;
uniform sampler2D uColor; uniform sampler2D uVel; uniform sampler2D uDepth;
uniform vec2 uRes; uniform float uBlur; uniform float uNear; uniform float uFar; uniform float uVignette; uniform float uFlash;
out vec4 frag;
float linZ(float d){ float z=d*2.0-1.0; return 2.0*uNear*uFar/(uFar+uNear-z*(uFar-uNear)); }
vec2 motionAt(vec2 uv, float depth){
  if(depth >= 0.99999){
    vec4 q = vSkyReproject*vec4(uv*2.0-1.0, 1.0, 1.0);
    return uv - (q.xy/q.w*0.5+0.5);
  }
  return texture(uVel, uv).xy;
}
void main(){
  vec3 base = texture(uColor, vUv).rgb;
  vec3 c = base;
  float d0 = texture(uDepth, vUv).r;
  vec2 vel = motionAt(vUv, d0)*uBlur;
  // Limited to a fifteenth of the screen width: beyond that it only hides the scenery.
  float px = length(vel*uRes);
  float maxPx = uRes.x/15.0;
  if(px > maxPx){ vel *= maxPx/px; px = maxPx; }
  if(px > 1.0){
    const int N = 16;
    float z0 = linZ(d0);
    float noise = fract(52.9829189*fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
    vec3 sum = base; float total = 1.0;
    for(int i = 0; i < N; i++){
      float t = (float(i)+noise)/float(N) - 0.5;
      vec2 uv = vUv + vel*t;
      float ds = texture(uDepth, uv).r;
      float zs = linZ(ds);
      if(zs < z0 - 0.25){
        vec2 vs = motionAt(uv, ds)*uBlur;
        if(length(vs*uRes) < 2.0*abs(t)*px) continue;
      }
      sum += texture(uColor, uv).rgb;
      total += 1.0;
    }
    c = sum/total;
  }
  // At speed the edges of the view fall away a little, as they do in the eye.
  vec2 e = vUv*2.0-1.0;
  c *= 1.0 - uVignette*smoothstep(0.35, 1.5, dot(e,e));
  c += uFlash*vec3(1.0, 0.85, 0.6);
  frag = vec4(c, 1.0);
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
uniform vec2 uPrevSpin;
uniform mat4 uPrevVP;
uniform mat4 uPrevWorld;
out vec3 vPos; out vec3 vNrm; out vec4 vCol; out vec4 vShadow; out vec3 vRest; out vec3 vRestN;
out vec4 vClip; out vec4 vPrevClip;
const float PI = 3.14159265;
void main(){
  vRest = vec3(0.0);
  vRestN = vec3(0.0);
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
  vec3 pPrev = vec3(p.x*uPrevSpin.x - p.z*uPrevSpin.y, p.y, p.x*uPrevSpin.y + p.z*uPrevSpin.x);
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
  vClip = gl_Position;
  vPrevClip = uPrevVP * uPrevWorld * uModel * vec4(pPrev,1.0);
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
void main(){ gl_Position=uVP*vec4(aPos,1.0); vCol=aCol; gl_PointSize=clamp(aSize*uScale/gl_Position.w,1.0,420.0); })";
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
    {0.26f, 0.012f, 0.022f, 0.70f},  // paint: Citroen dark metallic red
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
    {0.135f, 0.17f, 0.075f, 0.12f},  // NATO olive
    {0.105f, 0.135f, 0.06f, 0.08f},  // darker fittings, weathered
    {0.055f, 0.052f, 0.048f, 0.25f}, // track steel and rubber
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
    case COL_BARK: r = .22f; g = .155f; b = .10f; break;
    case COL_NEEDLES: r = .10f; g = .25f; b = .08f; break;
    case COL_NEEDLES2: r = .13f; g = .30f; b = .10f; break;
    case COL_NEEDLES3: r = .08f; g = .21f; b = .09f; break;
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
  GLuint shadowFarFbo = 0, shadowFarTex = 0;
  static constexpr int kShadow = 2048, kShadowFar = 4096;

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
  CarMesh* mesh = nullptr;
  V3 spawn;
  u32 seenSplit = 0;
  CarMesh pristine;
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

  // Scene targets: the frame is rendered with MSAA and a motion vector per pixel, then blurred.
  GLuint progPost = 0;
  Mat4 camVP;
  GLuint detailTex = 0;
  void buildDetailTexture() {
    // Two tiling layers of soft noise: R is grit with dark flecks, G is broad patches.
    constexpr int N = 512;
    std::vector<u8> px(size_t(N) * N * 2);
    std::mt19937 rng(99);
    auto layer = [&](int cells, std::vector<float>& out) {
      std::vector<float> g(size_t(cells) * cells);
      for (float& f : g) f = float(rng() & 0xffff) / 65535.f;
      out.resize(size_t(N) * N);
      for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++) {
          const float fx = float(x) * cells / N, fy = float(y) * cells / N;
          const int ix = int(fx), iy = int(fy);
          float tx = fx - ix, ty = fy - iy;
          tx = tx * tx * (3 - 2 * tx);
          ty = ty * ty * (3 - 2 * ty);
          const int x1 = (ix + 1) % cells, y1 = (iy + 1) % cells;
          const float a = g[size_t(iy) * cells + ix], b = g[size_t(iy) * cells + x1], c = g[size_t(y1) * cells + ix], d = g[size_t(y1) * cells + x1];
          out[size_t(y) * N + x] = a + (b - a) * tx + (c - a) * ty + (a - b - c + d) * tx * ty;
        }
    };
    std::vector<float> l0, l1, l2, l3;
    layer(128, l0);
    layer(32, l1);
    layer(8, l2);
    layer(3, l3);
    for (size_t i = 0; i < size_t(N) * N; i++) {
      px[i * 2] = u8(std::clamp((l0[i] * 0.6f + l1[i] * 0.4f) * 255.f, 0.f, 255.f));
      px[i * 2 + 1] = u8(std::clamp((l2[i] * 0.6f + l3[i] * 0.4f) * 255.f, 0.f, 255.f));
    }
    glGenTextures(1, &detailTex);
    glBindTexture(GL_TEXTURE_2D, detailTex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, N, N, 0, GL_RG, GL_UNSIGNED_BYTE, px.data());
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    if (epoxy_has_gl_extension("GL_EXT_texture_filter_anisotropic"))
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 8.f);
  }
  GLuint sceneFbo = 0, sceneColor = 0, sceneVel = 0, sceneDepth = 0;
  GLuint resolveFbo = 0, resolveColor = 0, resolveVel = 0, resolveDepth = 0;
  int targetW = 0, targetH = 0;
  void ensureTargets(int w, int h) {
    if (w == targetW && h == targetH) return;
    targetW = w;
    targetH = h;
    if (!sceneFbo) {
      glGenFramebuffers(1, &sceneFbo);
      glGenFramebuffers(1, &resolveFbo);
      glGenRenderbuffers(1, &sceneColor);
      glGenRenderbuffers(1, &sceneVel);
      glGenRenderbuffers(1, &sceneDepth);
      glGenTextures(1, &resolveColor);
      glGenTextures(1, &resolveVel);
      glGenTextures(1, &resolveDepth);
    }
    constexpr int kSamples = 4;
    glBindRenderbuffer(GL_RENDERBUFFER, sceneColor);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, kSamples, GL_RGBA8, w, h);
    glBindRenderbuffer(GL_RENDERBUFFER, sceneVel);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, kSamples, GL_RG16F, w, h);
    glBindRenderbuffer(GL_RENDERBUFFER, sceneDepth);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, kSamples, GL_DEPTH_COMPONENT24, w, h);
    glBindFramebuffer(GL_FRAMEBUFFER, sceneFbo);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, sceneColor);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_RENDERBUFFER, sceneVel);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, sceneDepth);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) fprintf(stderr, "Szene-Framebuffer unvollstaendig\n");
    auto tex = [&](GLuint t, GLenum internal, GLenum format, GLenum type) {
      glBindTexture(GL_TEXTURE_2D, t);
      glTexImage2D(GL_TEXTURE_2D, 0, GLint(internal), w, h, 0, format, type, nullptr);
      const GLint filter = format == GL_DEPTH_COMPONENT ? GL_NEAREST : GL_LINEAR;
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    };
    tex(resolveColor, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    tex(resolveVel, GL_RG16F, GL_RG, GL_FLOAT);
    tex(resolveDepth, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT, GL_FLOAT);
    glBindFramebuffer(GL_FRAMEBUFFER, resolveFbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, resolveColor, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, resolveVel, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, resolveDepth, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) fprintf(stderr, "Resolve-Framebuffer unvollstaendig\n");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }
  void useBuffers(int n) {
    static const GLenum both[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
    glDrawBuffers(n, both);
  }

  // Motion for the blur. The physics delivers in steps of 4 ms and frames do not line up with them, so
  // raw differences between frames would make the blur length pump. These rates are smoothed.
  static constexpr double kShutter = 1.0 / 55.0;  // seconds of motion smeared into one frame
  V3 lastCog, lastFwd, lastUp, lastEye, lastTarget;
  V3 rateCog, rateFwd, rateUp, rateEye, rateTarget;
  bool motionValid = false;
  Mat4 prevVP, carPrevWorld, skyVP, skyPrevVP;
  void updateMotion(const Snapshot& s, const Camera& cam, const Mat4& proj, double dt) {
    const double a = dt > 0 ? clampd(dt / 0.05, 0, 1) : 0;
    auto track = [&](V3 now, V3& last, V3& rate, double limit) {
      if (motionValid && dt > 1e-4) {
        const V3 v = (now - last) / dt;
        if (len(v) > limit) rate = {};  // reset or repair: a jump, not motion
        else rate += (v - rate) * a;
      } else {
        rate = {};
      }
      last = now;
    };
    track(s.carCog, lastCog, rateCog, 200);
    track(s.carFwd, lastFwd, rateFwd, 20);
    track(s.carUp, lastUp, rateUp, 20);
    track(cam.eye, lastEye, rateEye, 200);
    track(cam.target, lastTarget, rateTarget, 200);
    motionValid = true;
    const double T = kShutter;
    prevVP = mul(proj, lookAt(cam.eye - rateEye * T, cam.target - rateTarget * T, cam.up));
    const V3 f = norm(cam.target - cam.eye);
    const V3 fp = norm(cam.target - rateTarget * T - cam.eye + rateEye * T);
    skyVP = mul(proj, lookAt(V3{}, f, cam.up));
    skyPrevVP = mul(proj, lookAt(V3{}, fp, cam.up));
    // The car as a rigid body, one shutter time ago: p_prev = c_prev + R_prev R_now^T (p - c_now).
    const V3 f0 = s.carFwd, u0 = norm(s.carUp - s.carFwd * dot(s.carUp, s.carFwd)), l0 = cross(u0, f0);
    const V3 f1 = norm(f0 - rateFwd * T);
    const V3 up1 = s.carUp - rateUp * T;
    const V3 u1 = norm(up1 - f1 * dot(up1, f1)), l1 = cross(u1, f1);
    const V3 c1 = s.carCog - rateCog * T;
    auto rot = [&](V3 p) { return f1 * dot(p, f0) + l1 * dot(p, l0) + u1 * dot(p, u0); };
    const V3 cx = rot(V3{1, 0, 0}), cy = rot(V3{0, 1, 0}), cz = rot(V3{0, 0, 1});
    const V3 t = c1 - rot(s.carCog);
    Mat4 m;
    m.m[0] = float(cx.x); m.m[1] = float(cx.y); m.m[2] = float(cx.z);
    m.m[4] = float(cy.x); m.m[5] = float(cy.y); m.m[6] = float(cy.z);
    m.m[8] = float(cz.x); m.m[9] = float(cz.y); m.m[10] = float(cz.z);
    m.m[12] = float(t.x); m.m[13] = float(t.y); m.m[14] = float(t.z);
    m.m[15] = 1;
    carPrevWorld = m;
  }

  // Scratches: five maps of the car in its own frame. R is groove depth, G is scuffed clear coat.
  static constexpr int kScrW = 2400, kScrH = 800, kScrLayers = 5;
  static constexpr double kScrSizeX = 4.8, kScrSizeY = 2.0, kScrSizeZ = 1.6, kScrOrgX = 0.1, kScrOrgY = 1.0;
  GLuint carRestVbo = 0, carRestNVbo = 0, scratchTex = 0;
  std::vector<u8> scratchPx;
  struct DirtyRect { int x0 = 1 << 30, y0 = 1 << 30, x1 = -1, y1 = -1; } scratchDirty[kScrLayers];
  double mipClock = 0;
  bool mipStale = false;
  std::vector<std::vector<u32>> scratchGrid;
  static constexpr double kScratchCell = 0.30;
  static constexpr int kGridX = 18, kGridY = 8, kGridZ = 6;
  size_t scrapeTotal = 0;
  std::mt19937 scratchRng{12345};
  static int gridIndex(double x, double y, double z) {
    const int ix = std::clamp(int(x / kScratchCell) + 1, 0, kGridX - 1);
    const int iy = std::clamp(int((y + 1.2) / kScratchCell), 0, kGridY - 1);
    const int iz = std::clamp(int(z / kScratchCell), 0, kGridZ - 1);
    return (iz * kGridY + iy) * kGridX + ix;
  }
  static int chartOf(float nx, float ny, float nz) {
    const float ax = std::fabs(nx), ay = std::fabs(ny), az = std::fabs(nz);
    if (ay >= ax && ay >= az * 0.9f) return ny > 0 ? 0 : 1;
    if (az >= ax) return nz < 0 ? -1 : 2;
    return nx < 0 ? 3 : 4;
  }
  void buildScratchData() {
    const CarMesh& M = *mesh;
    const size_t nv = M.restPos.size() / 3;
    std::vector<float> restN(nv * 3, 0.f);
    scratchGrid.assign(size_t(kGridX) * kGridY * kGridZ, {});
    for (const MeshPart& p : M.parts) {
      if (!p.bound) continue;
      // Only the outer skin takes scratches. Glass, lenses and everything inside stay as they are.
      const bool skin = p.mat == MM_PAINT || p.mat == MM_BLACK || p.mat == MM_DARK || p.mat == MM_CHROME;
      const bool outer = p.group != "frame" && p.group != "engine" && p.group != "gearbox" && p.group != "radiator" && p.group != "battery" && p.group != "tank";
      if (!skin || !outer) continue;
      for (u32 v = p.firstVert; v < p.firstVert + p.vertCount; v++) {
        for (int k = 0; k < 3; k++) restN[size_t(v) * 3 + k] = M.restNormal[size_t(v) * 3 + k];
        scratchGrid[size_t(gridIndex(M.restPos[v * 3], M.restPos[v * 3 + 1], M.restPos[v * 3 + 2]))].push_back(v);
      }
    }
    glGenBuffers(1, &carRestVbo);
    glBindBuffer(GL_ARRAY_BUFFER, carRestVbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(M.restPos.size() * sizeof(float)), M.restPos.data(), GL_STATIC_DRAW);
    glGenBuffers(1, &carRestNVbo);
    glBindBuffer(GL_ARRAY_BUFFER, carRestNVbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(restN.size() * sizeof(float)), restN.data(), GL_STATIC_DRAW);
    for (GLuint vao : {carVao, carGlassVao, lensVao}) {
      glBindVertexArray(vao);
      glBindBuffer(GL_ARRAY_BUFFER, carRestVbo);
      glEnableVertexAttribArray(3);
      glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
      glBindBuffer(GL_ARRAY_BUFFER, carRestNVbo);
      glEnableVertexAttribArray(4);
      glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    }
    // Everything else in the world reads the attribute's constant value: no scratches.
    glBindVertexArray(0);
    glVertexAttrib3f(4, 0.f, 0.f, 0.f);
    glVertexAttrib3f(3, 0.f, 0.f, 0.f);

    scratchPx.assign(size_t(kScrW) * kScrH * kScrLayers * 2, 0);
    glGenTextures(1, &scratchTex);
    glBindTexture(GL_TEXTURE_2D_ARRAY, scratchTex);
    int levels = 1;
    for (int m = std::max(kScrW, kScrH); m > 1; m >>= 1) levels++;
    glTexStorage3D(GL_TEXTURE_2D_ARRAY, levels, GL_RG8, kScrW, kScrH, kScrLayers);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    if (epoxy_has_gl_extension("GL_EXT_texture_filter_anisotropic"))
      glTexParameterf(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAX_ANISOTROPY_EXT, 8.f);
    uploadScratches(true);
  }
  void uploadScratches(bool all) {
    glBindTexture(GL_TEXTURE_2D_ARRAY, scratchTex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, kScrW);
    bool any = false;
    for (int l = 0; l < kScrLayers; l++) {
      DirtyRect& r = scratchDirty[l];
      if (all) r = DirtyRect{0, 0, kScrW - 1, kScrH - 1};
      if (r.x1 < r.x0) continue;
      const u8* src = &scratchPx[(size_t(l) * kScrH * kScrW + size_t(r.y0) * kScrW + r.x0) * 2];
      glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, r.x0, r.y0, l, r.x1 - r.x0 + 1, r.y1 - r.y0 + 1, 1, GL_RG, GL_UNSIGNED_BYTE, src);
      r = DirtyRect{};
      any = true;
    }
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    if (any) mipStale = true;
  }
  // Paints a capsule into one map. Position in metres on the map's two axes.
  void paintLine(int layer, double ax, double ay, double bx, double by, double halfWidth, float groove, float scuff) {
    const double sx = layer <= 2 ? kScrW / kScrSizeX : kScrW / kScrSizeY;
    const double sy = layer == 2 ? kScrH / kScrSizeY : kScrH / kScrSizeZ;
    const double feather = 1.5 / std::min(sx, sy);
    const double reach = halfWidth + feather;
    const int x0 = std::max(0, int((std::min(ax, bx) - reach) * sx)), x1 = std::min(kScrW - 1, int((std::max(ax, bx) + reach) * sx) + 1);
    const int y0 = std::max(0, int((std::min(ay, by) - reach) * sy)), y1 = std::min(kScrH - 1, int((std::max(ay, by) + reach) * sy) + 1);
    if (x1 < x0 || y1 < y0) return;
    const double dx = bx - ax, dy = by - ay, l2 = std::max(dx * dx + dy * dy, 1e-9);
    u8* base = &scratchPx[size_t(layer) * kScrH * kScrW * 2];
    for (int y = y0; y <= y1; y++) {
      const double py = (y + 0.5) / sy;
      u8* row = base + size_t(y) * kScrW * 2;
      for (int x = x0; x <= x1; x++) {
        const double px = (x + 0.5) / sx;
        const double t = std::clamp(((px - ax) * dx + (py - ay) * dy) / l2, 0.0, 1.0);
        const double ex = px - ax - dx * t, ey = py - ay - dy * t;
        const double d = std::sqrt(ex * ex + ey * ey);
        if (d >= reach) continue;
        const double cover = std::clamp((reach - d) / (2.0 * feather), 0.0, 1.0);
        // The ends taper out, a stylus lifting off the paint.
        const double taper = std::min(1.0, std::min(t, 1.0 - t) * 6.0 + 0.15);
        u8* o = row + size_t(x) * 2;
        o[0] = std::max(o[0], u8(255.0 * groove * cover * taper));
        // Scuffing builds up with repeated passes.
        o[1] = u8(std::min(255.0, o[1] + 255.0 * scuff * cover * taper));
      }
    }
    DirtyRect& r = scratchDirty[layer];
    r.x0 = std::min(r.x0, x0);
    r.y0 = std::min(r.y0, y0);
    r.x1 = std::max(r.x1, x1);
    r.y1 = std::max(r.y1, y1);
  }
  // One bundle of marks left by something dragging over the panel: a broad scuffed band, a few clear
  // scratches, and sometimes a gouge down to the steel. Always a few millimetres wide, so they read
  // as individual lines from the chase camera.
  void addStroke(int layer, double u, double v, double du, double dv, double strength) {
    auto rnd = [&](double a, double b) { return a + (b - a) * double(scratchRng() & 0xffff) / 65535.0; };
    const double dl = std::hypot(du, dv);
    if (dl < 0.2) return;  // sliding into the surface, not along it
    du /= dl;
    dv /= dl;
    const double nu = -dv, nv = du;
    const double len = rnd(0.12, 0.30 + 0.45 * strength);
    if (rnd(0, 1) < 0.5)
      paintLine(layer, u - du * len, v - dv * len, u + du * len, v + dv * len, rnd(0.012, 0.03), 0.f, float(0.015 + 0.03 * strength));
    const double off = rnd(-0.035, 0.035);
    const double a = rnd(-0.3, 1.0) * len, b = a - rnd(0.5, 1.6) * len;
    const double width = rnd(0.0016, 0.0034) * (1.0 + 1.2 * strength);
    const float depth = float(rnd(0.22, 0.42));
    // A slightly wavering line: the panel bends and the contact point jumps.
    double px = u + nu * off + du * b, py = v + nv * off + dv * b;
    const double wob = rnd(-0.012, 0.012);
    for (int i = 1; i <= 4; i++) {
      const double s = b + (a - b) * i / 4.0;
      const double w = off + wob * std::sin(i * 1.7) + rnd(-0.0015, 0.0015);
      const double qx = u + nu * w + du * s, qy = v + nv * w + dv * s;
      paintLine(layer, px, py, qx, qy, width, depth, 0.f);
      px = qx;
      py = qy;
    }
    if (strength > 0.35 && rnd(0, 1) < 0.10 + 0.25 * strength) {
      const double g = off + rnd(-0.02, 0.02), gl = len * rnd(0.3, 0.8), gw = rnd(0.004, 0.009);
      paintLine(layer, u + nu * g - du * gl, v + nv * g - dv * gl, u + nu * g + du * gl, v + nv * g + dv * gl, gw, float(rnd(0.75, 1.0)), 0.f);
    }
  }
  void applyScrapes(const Snapshot& s, double dt) {
    if (!rig->scratches) return;
    const CarMesh& M = *mesh;
    scrapeTotal += s.scrapes.size();
    for (const Scrape& sc : s.scrapes) {
      // A contact reports hundreds of times a second: thin that out to the scratches it really leaves.
      if ((scratchRng() & 1023) > 40 + 110 * sc.strength) continue;
      // To the design frame the mesh was modelled in.
      const double px = rig->noseX - (sc.restPoint.x - spawn.x), py = sc.restPoint.y - spawn.y, pz = sc.restPoint.z - spawn.z;
      const V3 d = norm(V3{-dot(sc.dir, s.carFwd), dot(sc.dir, s.carLeft), dot(sc.dir, s.carUp)});
      // The node is only where the structure touches. Which faces of the skin are around it?
      int count[kScrLayers] = {};
      constexpr double kReach2 = 0.16 * 0.16;
      for (int dz = -1; dz <= 1; dz++)
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++) {
            const double qx = px + dx * kScratchCell, qy = py + dy * kScratchCell, qz = pz + dz * kScratchCell;
            if (qx < -kScratchCell || qy < -1.2 || qz < 0) continue;
            for (u32 v : scratchGrid[size_t(gridIndex(qx, qy, qz))]) {
              const V3 r{M.restPos[v * 3] - px, M.restPos[v * 3 + 1] - py, M.restPos[v * 3 + 2] - pz};
              if (len2(r) > kReach2) continue;
              const int c = chartOf(M.restNormal[size_t(v) * 3], M.restNormal[size_t(v) * 3 + 1], M.restNormal[size_t(v) * 3 + 2]);
              if (c >= 0) count[c]++;
            }
          }
      int total = 0;
      for (int c : count) total += c;
      if (total == 0) continue;
      for (int l = 0; l < kScrLayers; l++) {
        if (count[l] * 5 < total) continue;
        const double strength = sc.strength;
        if (l <= 1) addStroke(l, px + kScrOrgX, pz, d.x, d.z, strength);
        else if (l == 2) addStroke(l, px + kScrOrgX, py + kScrOrgY, d.x, d.y, strength);
        else addStroke(l, py + kScrOrgY, pz, d.y, d.z, strength);
      }
    }
    uploadScratches(false);
    // Rebuilding the mipmaps touches the whole texture, so it happens a few times a second at most.
    mipClock += dt;
    if (mipStale && mipClock > 0.12) {
      glBindTexture(GL_TEXTURE_2D_ARRAY, scratchTex);
      glGenerateMipmap(GL_TEXTURE_2D_ARRAY);
      mipStale = false;
      mipClock = 0;
    }
  }

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
        // Each point has one cross direction, shared by the two pieces that meet there. With one per
        // piece the edges of neighbouring pieces do not line up, and the bends fall into segments.
        auto across = [&](size_t k, double& nx, double& ny) {
          const RoadPoint& p0 = road.pts[road.closed ? (k + cnt - 1) % cnt : (k > 0 ? k - 1 : 0)];
          const RoadPoint& p1 = road.pts[road.closed ? (k + 1) % cnt : std::min(k + 1, cnt - 1)];
          const double tx = p1.x - p0.x, ty = p1.y - p0.y, tl = std::hypot(tx, ty);
          nx = -ty / tl;
          ny = tx / tl;
        };
        double anx, any, bnx, bny;
        across(i, anx, any);
        across((i + 1) % cnt, bnx, bny);
        auto at = [&](const RoadPoint& p, double off) {
          const bool first = &p == &a;
          const double x = p.x + (first ? anx : bnx) * off, y = p.y + (first ? any : bny) * off;
          // Height from the road's own plane, not the 1 m terrain grid, so that it never saws up and down.
          return V3{x, y, p.z + off * p.bank + 0.035};
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
    // Reflector posts every 25 m along the roads, the classic cue for speed.
    for (const Road& road : world->roads()) {
      double run = 0;
      const size_t cnt = road.pts.size();
      const size_t segs = road.closed ? cnt : cnt - 1;
      bool flip = false;
      for (size_t i = 0; i < segs; i++) {
        const RoadPoint& a = road.pts[i];
        const RoadPoint& b = road.pts[(i + 1) % cnt];
        const double tl = std::hypot(b.x - a.x, b.y - a.y);
        run += tl;
        if (run < 25.0) continue;
        run = 0;
        flip = !flip;
        const double tx = (b.x - a.x) / tl, ty = (b.y - a.y) / tl, nx = -ty, ny = tx;
        for (double sd : {-1.0, 1.0}) {
          const double px = a.x + nx * sd * (road.halfWidth + 1.1), py = a.y + ny * sd * (road.halfWidth + 1.1);
          const double z0 = world->height(px, py);
          const double hw = 0.06, hd = 0.04;
          auto box = [&](double zb, double zt, float cr, float cg, float cb) {
            const V3 c0{px - tx * hw - nx * hd, py - ty * hw - ny * hd, zb}, c1{px + tx * hw - nx * hd, py + ty * hw - ny * hd, zb};
            const V3 c2{px + tx * hw + nx * hd, py + ty * hw + ny * hd, zb}, c3{px - tx * hw + nx * hd, py - ty * hw + ny * hd, zb};
            const V3 up{0, 0, zt - zb};
            addQuad(v, c0, c1, c1 + up, c0 + up, cr, cg, cb, 0);
            addQuad(v, c1, c2, c2 + up, c1 + up, cr, cg, cb, 0);
            addQuad(v, c2, c3, c3 + up, c2 + up, cr, cg, cb, 0);
            addQuad(v, c3, c0, c0 + up, c3 + up, cr, cg, cb, 0);
            addQuad(v, c0 + up, c1 + up, c2 + up, c3 + up, cr, cg, cb, 0);
          };
          box(z0, z0 + 0.75, .85f, .85f, .83f);
          box(z0 + 0.75, z0 + 0.95, .05f, .05f, .05f);
          box(z0 + 0.80, z0 + 0.88, .95f, .55f, .15f);
        }
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
  // Track links: boxes spaced along the track's line, moving with the track.
  GLuint trackVao = 0, trackVbo = 0;
  std::vector<Vtx> trackVerts;
  void updateTrack(const Snapshot& s) {
    trackVerts.clear();
    if (!s.tank) return;
    if (!trackVao) {
      glGenVertexArrays(1, &trackVao);
      glGenBuffers(1, &trackVbo);
      glBindVertexArray(trackVao);
      glBindBuffer(GL_ARRAY_BUFFER, trackVbo);
      setupVtxAttribs();
    }
    const MatDef& md = kMats[MM_TRACK];
    constexpr double kPitch = 0.19, kThick = 0.05, kLen = 0.165, kHalfW = 0.275;
    for (int side = 0; side < 2; side++) {
      const std::vector<V3>& path = s.trackPath[side];
      const size_t n = path.size();
      if (n < 3) continue;
      double total = 0;
      for (size_t i = 0; i < n; i++) total += len(path[(i + 1) % n] - path[i]);
      const int links = int(total / kPitch);
      const double step = total / links;
      double along = std::fmod(s.trackPhase[side], step);
      if (along < 0) along += step;
      size_t seg = 0;
      double segStart = 0;
      for (int k = 0; k < links; k++) {
        const double d = along + k * step;
        while (seg < n) {
          const double L = len(path[(seg + 1) % n] - path[seg]);
          if (d <= segStart + L || seg + 1 == n) break;
          segStart += L;
          seg++;
        }
        const V3 a = path[seg], b = path[(seg + 1) % n];
        const double L = std::max(len(b - a), 1e-6);
        const V3 t = (b - a) / L;
        const V3 c = a + t * (d - segStart);
        const V3 w = s.carLeft;
        const V3 o = norm(cross(t, w));  // out of the track's plane
        const V3 lf = t * (kLen * 0.5), wd = w * kHalfW, up = o * (kThick * 0.5);
        const V3 corner[8] = {c - lf - wd - up, c + lf - wd - up, c + lf + wd - up, c - lf + wd - up, c - lf - wd + up, c + lf - wd + up, c + lf + wd + up, c - lf + wd + up};
        static const int faces[5][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4}, {2, 3, 7, 6}, {1, 2, 6, 5}};
        for (const auto& f : faces) addQuad(trackVerts, corner[f[0]], corner[f[1]], corner[f[2]], corner[f[3]], md.r, md.g, md.b, md.gloss);
      }
    }
    glBindBuffer(GL_ARRAY_BUFFER, trackVbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(trackVerts.size() * sizeof(Vtx)), trackVerts.data(), GL_STREAM_DRAW);
  }

  std::vector<size_t> steerParts;
  int steerRim = -1;
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

    // The steering wheel turns about its column. Lock to lock is just under three turns.
    if (steerParts.empty()) {
      for (size_t i = 0; i < M.parts.size(); i++) {
        if (M.parts[i].name.rfind("steerwheel_", 0) == 0) steerParts.push_back(i);
        if (M.parts[i].name.rfind("steerwheel_rim", 0) == 0) steerRim = int(i);
      }
    }
    if (steerRim >= 0) {
      constexpr double kLockAngle = 2.9 * PI;
      constexpr double kColumnTilt = 24.0 * PI / 180.0;
      const MeshPart& rim = M.parts[size_t(steerRim)];
      double cx = 0, cy = 0, cz = 0;
      for (u32 v = rim.firstVert; v < rim.firstVert + rim.vertCount; v++) {
        cx += carVerts[v].x;
        cy += carVerts[v].y;
        cz += carVerts[v].z;
      }
      const double n = std::max(double(rim.vertCount), 1.0);
      cx /= n;
      cy /= n;
      cz /= n;
      const V3 axis = norm(s.carFwd * std::cos(kColumnTilt) - s.carUp * std::sin(kColumnTilt));
      // The driver looks along the axis, so turning left is negative about it.
      const double ang = -s.steer * kLockAngle, ca = std::cos(ang), sa = std::sin(ang);
      for (size_t pi : steerParts) {
        const MeshPart& p = M.parts[pi];
        for (u32 v = p.firstVert; v < p.firstVert + p.vertCount; v++) {
          const V3 r{carVerts[v].x - cx, carVerts[v].y - cy, carVerts[v].z - cz};
          const V3 q = r * ca + cross(axis, r) * sa + axis * (dot(axis, r) * (1.0 - ca));
          carVerts[v].x = float(cx + q.x);
          carVerts[v].y = float(cy + q.y);
          carVerts[v].z = float(cz + q.z);
        }
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
      if (s.scorch > 0 && (p.mat == MM_GLASS || p.mat == MM_LENS)) continue;
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
        // The mesh is modelled with x running rearwards, the world with x forwards: that mirror turns
        // the winding around, so the outward normal is the negative of this cross product.
        const float nx = e1z * e2y - e1y * e2z, ny = e1x * e2z - e1z * e2x, nz = e1y * e2x - e1x * e2y;
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
    constexpr int kSeg = 28;
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
        case EV_WOOD:
        case EV_BRANCH: {
          // Splinters off a trunk, a puff of needles off a branch.
          const bool trunk = e.kind == EV_WOOD;
          for (int i = 0; i < (trunk ? 7 : 3); i++) {
            Particle q;
            q.p = e.p;
            q.v = e.v * .5 + V3{rnd(-4, 4), rnd(-4, 4), rnd(0, 5)};
            q.maxLife = q.life = rnd(.8f, 2.f);
            q.size = trunk ? rnd(.02f, .06f) : rnd(.03f, .08f);
            if (trunk) { q.r = .78f; q.g = .62f; q.b = .36f; }
            else { q.r = .14f; q.g = .32f; q.b = .10f; }
            q.a = 1.f;
            q.gravity = trunk ? 9.81f : 3.f;
            q.drag = trunk ? .5f : 3.f;
            particles.push_back(q);
          }
          break;
        }
        case EV_BLAST: {
          const float R = e.strength;  // the blast's radius in metres
          flash = std::max(flash, 1.f);
          // Fireball.
          for (int i = 0; i < 90; i++) {
            Particle q;
            const V3 d = norm(V3{rnd(-1, 1), rnd(-1, 1), rnd(-.2f, 1.2f)});
            q.p = e.p + d * rnd(.1f, .6f);
            q.v = d * rnd(.3f, 1.3f) * R * .75;
            q.maxLife = q.life = rnd(.25f, .7f);
            q.size = rnd(.5f, 1.2f) * (.3f + R * .04f);
            q.r = 1.f; q.g = rnd(.45f, .85f); q.b = rnd(.08f, .25f); q.a = 1.f;
            q.gravity = -3.f;
            q.drag = 4.f;
            particles.push_back(q);
          }
          // A column of black smoke, slow to clear.
          for (int i = 0; i < 70; i++) {
            Particle q;
            const V3 d = norm(V3{rnd(-1, 1), rnd(-1, 1), rnd(0, 1.5f)});
            q.p = e.p + d * rnd(.1f, 1.f);
            q.v = d * rnd(.3f, 1.f) * R * .35 + V3{0, 0, rnd(1.f, 5.f)};
            q.maxLife = q.life = rnd(2.f, 6.f);
            q.size = rnd(.5f, 1.3f) * (.4f + R * .05f);
            const float g = rnd(.04f, .13f);
            q.r = g * 1.6f; q.g = g * 1.6f; q.b = g * 1.6f; q.a = rnd(.16f, .34f);
            q.gravity = -.8f;
            q.drag = 1.6f;
            particles.push_back(q);
          }
          // Shrapnel and sparks.
          for (int i = 0; i < 130; i++) {
            Particle q;
            const V3 d = norm(V3{rnd(-1, 1), rnd(-1, 1), rnd(-.1f, 1.f)});
            q.p = e.p;
            q.v = d * rnd(.5f, 2.4f) * R;
            q.maxLife = q.life = rnd(.5f, 1.8f);
            q.size = rnd(.02f, .06f);
            q.r = 1.f; q.g = rnd(.5f, .8f); q.b = .2f; q.a = 1.f;
            q.gravity = 9.81f;
            q.drag = .25f;
            particles.push_back(q);
          }
          break;
        }
        case EV_FIRE: {
          Particle q;
          q.p = e.p + V3{rnd(-.5f, .5f), rnd(-.5f, .5f), rnd(-.1f, .3f)};
          q.v = V3{rnd(-.3f, .3f), rnd(-.3f, .3f), rnd(1.f, 3.f)};
          q.maxLife = q.life = rnd(.3f, .8f);
          q.size = rnd(.22f, .5f) * (.4f + e.strength);
          q.r = 1.f; q.g = rnd(.35f, .7f); q.b = .08f; q.a = .9f;
          q.gravity = -1.f;
          q.drag = 1.f;
          particles.push_back(q);
          if (rng() % 2 == 0) {
            Particle k = q;
            k.p.z += .4;
            k.v = V3{rnd(-.4f, .4f), rnd(-.4f, .4f), rnd(2.f, 5.f)};
            k.maxLife = k.life = rnd(2.f, 5.f);
            k.size = rnd(.35f, .8f);
            const float g = rnd(.03f, .10f);
            k.r = g; k.g = g; k.b = g; k.a = .55f;
            k.gravity = -.5f;
            k.drag = .6f;
            particles.push_back(k);
          }
          break;
        }
        case EV_MUZZLE: {
          flash = std::max(flash, .35f);
          for (int i = 0; i < 26; i++) {
            Particle q;
            const V3 d = norm(e.v + V3{rnd(-.35f, .35f), rnd(-.35f, .35f), rnd(-.35f, .35f)});
            q.p = e.p + d * rnd(0.f, 1.2f);
            q.v = d * rnd(4.f, 22.f);
            q.maxLife = q.life = rnd(.05f, .16f);
            q.size = rnd(.3f, .8f);
            q.r = 1.f; q.g = rnd(.6f, .9f); q.b = .3f; q.a = 1.f;
            q.gravity = 0.f;
            q.drag = 5.f;
            particles.push_back(q);
          }
          for (int i = 0; i < 26; i++) {
            Particle q;
            const V3 d = norm(e.v * (rnd(0, 1) < .5f ? 1.0 : 0.15) + V3{rnd(-.6f, .6f), rnd(-.6f, .6f), rnd(-.3f, .6f)});
            q.p = e.p;
            q.v = d * rnd(2.f, 12.f);
            q.maxLife = q.life = rnd(.6f, 2.4f);
            q.size = rnd(.3f, .8f);
            const float g = rnd(.55f, .72f);
            q.r = g; q.g = g; q.b = g * .96f; q.a = .5f;
            q.gravity = -.4f;
            q.drag = 2.5f;
            particles.push_back(q);
          }
          break;
        }
        case EV_TRACER: {
          Particle q;
          q.p = e.p;
          q.v = e.v * .02;
          q.maxLife = q.life = .16f;
          q.size = .16f;
          q.r = 1.f; q.g = .75f; q.b = .35f; q.a = 1.f;
          q.gravity = 0.f;
          q.drag = 0.f;
          particles.push_back(q);
          break;
        }
      }
    }
  }
  float flash = 0;  // lights up the whole picture for a moment after a detonation

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
    const float rpmN = float(s.rpm / s.rpmMax);
    bar(bx, by + 76 * scale, 270 * scale, 14 * scale, rpmN, s.rpm > s.rpmRed ? 1.f : .35f, s.rpm > s.rpmRed ? .15f : .8f, .25f);
    snprintf(buf, sizeof buf, "%4.0f U/MIN %2.0FL %s", s.rpm, s.fuel, s.running ? "" : "MOTOR AUS");
    text(bx, by + 94 * scale, buf, px1 * 2, .85f, .85f, .85f);

    // Lower left: the vehicle's systems.
    const float lx = 14 * scale;
    if (s.tank) {
      float ly = H - 148 * scale;
      rect(lx - 6 * scale, ly - 8 * scale, 290 * scale, 144 * scale, 0, 0, 0, .35f);
      text(lx, ly, "LEOPARD 1", px2, .75f, .95f, .55f);
      ly += 22 * scale;
      snprintf(buf, sizeof buf, "TURM %+4.0F ROHR %+3.0F", s.turretYaw * 57.3, s.gunPitch * 57.3);
      text(lx, ly, buf, px2, .85f, .85f, .85f);
      ly += 22 * scale;
      if (s.reload > 0) {
        text(lx, ly, "LADEN", px2, 1, .7f, .3f);
        bar(lx + 70 * scale, ly, 200 * scale, 14 * scale, float(1.0 - s.reload / std::max(s.reloadTime, 0.1)), 1.f, .7f, .25f);
      } else {
        text(lx, ly, "FEUERBEREIT", px2, .5f, 1.f, .5f);
      }
      ly += 22 * scale;
      snprintf(buf, sizeof buf, "QUER %+4.2FG KRAFTSTOFF %4.0FL", s.latG, s.fuel);
      text(lx, ly, buf, px2 * 0.85f, .85f, .85f, .85f);
      ly += 22 * scale;
      text(lx, ly, "GAS", px1 * 2, .8f, .8f, .8f);
      bar(lx + 40 * scale, ly, 90 * scale, 12 * scale, float(s.throttle), .3f, .9f, .3f);
      text(lx + 140 * scale, ly, "BR", px1 * 2, .8f, .8f, .8f);
      bar(lx + 170 * scale, ly, 100 * scale, 12 * scale, float(s.brake), .95f, .25f, .2f);
    } else {
    float ly = H - 234 * scale;
    rect(lx - 6 * scale, ly - 8 * scale, 290 * scale, 230 * scale, 0, 0, 0, .35f);
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
    snprintf(buf, sizeof buf, "WASSER %3.0F C", s.coolant);
    text(lx, ly, buf, px2, s.coolant > 112 ? 1.f : .85f, s.coolant > 112 ? .3f : .85f, s.coolant > 112 ? .25f : .85f);
    bar(lx + 150 * scale, ly, 120 * scale, 14 * scale, float((s.coolant - 40.0) / 95.0), s.coolant > 112 ? 1.f : .4f, s.coolant > 112 ? .2f : .7f, .3f);
    ly += 22 * scale;
    snprintf(buf, sizeof buf, "OEL    %3.0F C", s.oilTemp);
    text(lx, ly, buf, px2, s.oilTemp > 135 ? 1.f : .85f, s.oilTemp > 135 ? .3f : .85f, s.oilTemp > 135 ? .25f : .85f);
    bar(lx + 150 * scale, ly, 120 * scale, 14 * scale, float((s.oilTemp - 40.0) / 150.0), s.oilTemp > 135 ? 1.f : .9f, s.oilTemp > 135 ? .2f : .65f, .2f);
    ly += 22 * scale;
    snprintf(buf, sizeof buf, "OELSTAND %3.0F%%", s.oilLevel * 100.0);
    text(lx, ly, buf, px2, s.oilLevel < .35 ? 1.f : .85f, s.oilLevel < .35 ? .3f : .85f, s.oilLevel < .35 ? .25f : .85f);
    ly += 22 * scale;
    text(lx, ly, "GAS", px1 * 2, .8f, .8f, .8f);
    bar(lx + 40 * scale, ly, 90 * scale, 12 * scale, float(s.throttle), .3f, .9f, .3f);
    text(lx + 140 * scale, ly, "BR", px1 * 2, .8f, .8f, .8f);
    bar(lx + 170 * scale, ly, 100 * scale, 12 * scale, float(s.brake), .95f, .25f, .2f);

    }

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
    snprintf(buf, sizeof buf, "PHYSIK %3.0F%%  ECHTZEIT %3.0F%%  WACH %d/%d", s.physLoad * 100.0, s.realtime * 100.0, s.awake, s.objectCount);
    rtext(buf, s.realtime < s.timeScale * 0.9 ? 1.f : .75f, s.realtime < s.timeScale * 0.9 ? .35f : .85f, .75f);
    rtext(o.gamepad ? "CONTROLLER AKTIV" : "TASTATUR/MAUS", .75f, .75f, .75f);
    snprintf(buf, sizeof buf, "KAMERA %s", o.cameraName.c_str());
    rtext(buf, .75f, .75f, .75f);
    if (o.mouseSteer) rtext("MAUSLENKUNG AN", 1, .85f, .35f);
    if (!s.tank) {
      // Driver aid lamps: amber while working, dark when off.
      if (s.absActive) rtext("ABS", 1, .75f, .2f);
      if (s.espActive) rtext("ESP", 1, .75f, .2f);
      if (s.tcsActive) rtext("ASR", 1, .75f, .2f);
      if (!s.absOn) rtext("ABS AUS", .75f, .4f, .3f);
      if (!s.espOn) rtext("ESP AUS", .75f, .4f, .3f);
      if (!s.tcsOn) rtext("ASR AUS", .75f, .4f, .3f);
    }
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

    if (o.damageView && !s.parts.empty()) {
      // The vehicle from above, every part coloured by its state, with the inside parts drawn over.
      const float dw = 92 * scale, dh = 220 * scale;
      const float dx = W - dw - 14 * scale, dy = H - 130 * scale - dh - 26 * scale;
      rect(dx - 6 * scale, dy - 6 * scale, dw + 12 * scale, dh + 12 * scale, 0, 0, 0, .42f);
      const float ux = dw / 100.f, uy = dh / 240.f;
      for (const PartState& p : s.parts) {
        const float d = std::clamp(p.damage, 0.f, 1.f);
        // Green through amber to red, and dark when it is gone.
        float cr = d < .5f ? .2f + d * 1.6f : 1.f, cg = d < .5f ? .8f : .8f - (d - .5f) * 1.4f, cb = .22f;
        if (d >= .98f) { cr = .35f; cg = .05f; cb = .05f; }
        rect(dx + p.x * ux, dy + p.y * uy, p.w * ux - 1, p.h * uy - 1, cr, cg, cb, .9f);
        if (p.label) text(dx + p.x * ux + 2, dy + p.y * uy + 2, p.label, scale * 1.2f, 0, 0, 0, .9f);
      }
    }
    if (o.crosshair) {
      // The crosshair is where the gunner looks. The ring shows where the gun really points just now.
      const float cx = W * .5f, cy = H * .5f, a = 2 * scale, l = 11 * scale;
      rect(cx - l - 6 * scale, cy - a * .5f, l, a, 1, 1, 1, .8f);
      rect(cx + 6 * scale, cy - a * .5f, l, a, 1, 1, 1, .8f);
      rect(cx - a * .5f, cy + 6 * scale, a, l, 1, 1, 1, .8f);
      const V3 p = s.muzzle + s.gunDir * 400.0;
      const Mat4& m = camVP;
      const double qx = m.m[0] * p.x + m.m[4] * p.y + m.m[8] * p.z + m.m[12];
      const double qy = m.m[1] * p.x + m.m[5] * p.y + m.m[9] * p.z + m.m[13];
      const double qw = m.m[3] * p.x + m.m[7] * p.y + m.m[11] * p.z + m.m[15];
      if (qw > 0.1) {
        const float sx = float((qx / qw * .5 + .5) * W), sy = float((.5 - qy / qw * .5) * H), k = 9 * scale;
        const float col = s.reload > 0 ? .55f : 1.f;
        rect(sx - k, sy - k, 2 * k, a, 1, col, col * .3f, .9f);
        rect(sx - k, sy + k, 2 * k, a, 1, col, col * .3f, .9f);
        rect(sx - k, sy - k, a, 2 * k, 1, col, col * .3f, .9f);
        rect(sx + k, sy - k, a, 2 * k, 1, col, col * .3f, .9f);
      }
    }
    if (o.panel.open) {
      // A list with a header: vehicle menu or settings overlay.
      const UiPanel& P = o.panel;
      float x, y, pw, rowH;
      panelLayout(w, h, P.rows.size(), x, y, pw, rowH);
      rect(x, y - 2.0f * rowH, pw, (float(P.rows.size()) + 3.6f) * rowH, 0, 0, 0, .8f);
      text(x + 16 * scale, y - 1.5f * rowH, P.title.c_str(), px3, 1, .9f, .55f);
      for (size_t i = 0; i < P.rows.size(); i++) {
        const float ry2 = y + float(i) * rowH;
        const bool sel = int(i) == P.sel;
        if (sel) rect(x + 6 * scale, ry2 - 2 * scale, pw - 12 * scale, rowH - 2 * scale, .85f, .55f, .15f, .35f);
        const float c = P.rows[i].empty() ? .5f : (sel ? 1.f : .85f);
        text(x + 18 * scale, ry2 + 2 * scale, P.rows[i].c_str(), px2, c, c, c);
        if (i < P.fill.size() && P.fill[i] >= 0) bar(x + pw * .52f, ry2 + 3 * scale, pw * .26f, 12 * scale, P.fill[i], .85f, .65f, .2f);
        if (i < P.values.size() && !P.values[i].empty()) {
          const char* val = P.values[i].c_str();
          text(x + pw - float(strlen(val)) * 12 * scale - 18 * scale, ry2 + 2 * scale, val, px2, sel ? 1.f : .8f, sel ? .85f : .8f, sel ? .4f : .8f);
        }
      }
      text(x + 16 * scale, y + (float(P.rows.size()) + .35f) * rowH, P.foot.c_str(), px1 * 2, .65f, .65f, .65f);
    }
    if (o.help) {
      static const char* lines[] = {
          "BEANNG - CITROEN XANTIA ACTIVA",
          "",
          "W/UP GAS   S/DOWN BREMSE (STEHEND: RUECKWAERTS)",
          "A D / LEFT RIGHT LENKEN    SPACE HANDBREMSE",
          "Q E GANG RUNTER/HOCH      T AUTO/MANUELL",
          "G HYDRACTIVE SPORT   H NIVEAU   V ACTIVA AN/AUS",
          "I ZUENDUNG   R ZURUECK ZUM START   F5 REPARIEREN   F BOMBE/FEUER",
          "C KAMERA   M MAUSLENKUNG   P PAUSE   [ ] ZEITLUPE",
          "O EINSTELLUNGEN   TAB FAHRZEUG WAEHLEN   B BLUR   J WACKELN   N TON   + -",
          "F2 BEAM-ANSICHT   F3 ROENTGEN   F12 SCREENSHOT   ESC BEENDEN",
          "MAUS: RECHTS ZIEHEN KAMERA, RAD ZOOM, LINKS ZIEHEN PACKT DAS AUTO",
          "",
          "CONTROLLER: L-STICK LENKEN  RT GAS  LT BREMSE",
          "R-STICK KAMERA  A HANDBREMSE  LB/RB GANG  Y KAMERA",
          "X BOMBE/FEUER  R3 SPORT  B REPARIEREN  BACK START  START PAUSE",
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
    if (!rig->carWheels) return;
    glUseProgram(prog);
    setMat(prog, "uVP", vp);
    setMat(prog, "uShadowVP", shadowVP);
    setMat(prog, "uPrevVP", prevVP);
    setMat(prog, "uPrevWorld", carPrevWorld);
    const GLint uPrevSpin = glGetUniformLocation(prog, "uPrevSpin");
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
      {
        // Limited to about a third of a turn: past that a straight smear no longer looks like rotation.
        const double back = clampd(wv.omega * kShutter, -0.9, 0.9);
        const double tp = wv.theta - back;
        glUniform2f(uPrevSpin, float(std::cos(tp)), float(std::sin(tp) * (wv.left ? 1.0 : -1.0)));
      }
      for (size_t i = 0; i < wheelDrawFirst.size(); i++) {
        glUniform1i(uMode, wheelDrawMode[i]);
        glDrawElements(GL_TRIANGLES, wheelDrawCount[i], GL_UNSIGNED_INT, (void*)(size_t(wheelDrawFirst[i]) * sizeof(u32)));
      }
      // Caliper: fixed to the hub, it does not turn.
      glUniform1i(uMode, 0);
      glUniform2f(uSpin, 1.f, 0.f);
      glUniform2f(uPrevSpin, 1.f, 0.f);
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

Renderer::~Renderer() { delete d; }

bool Renderer::init(const World& world, const Sim& sim, const CarRig& rig, CarMesh& mesh, V3 spawn) {
  d = new Impl();
  d->world = &world;
  d->sim = &sim;
  d->rig = &rig;
  d->mesh = &mesh;
  d->spawn = spawn;
  d->pristine = mesh;
  d->partHidden.assign(mesh.parts.size(), 0);

  d->progMesh = program(kMeshVS, kMeshFS);
  d->progDepth = program(kDepthVS, kDepthFS);
  d->progWheel = program(kWheelVS, kMeshFS);
  d->progWheelDepth = program(kWheelVS, kDepthFS);
  d->progSky = program(kSkyVS, kSkyFS);
  d->progPart = program(kPartVS, kPartFS);
  d->progLine = program(kLineVS, kLineFS);
  d->progHud = program(kHudVS, kHudFS);
  d->progPost = program(kPostVS, kPostFS);

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
  glGenFramebuffers(1, &d->shadowFarFbo);
  glGenTextures(1, &d->shadowFarTex);
  glBindTexture(GL_TEXTURE_2D, d->shadowFarTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, Impl::kShadowFar, Impl::kShadowFar, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  glBindFramebuffer(GL_FRAMEBUFFER, d->shadowFarFbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, d->shadowFarTex, 0);
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

  d->buildScratchData();
  d->buildDetailTexture();

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
  std::fill(d->scratchPx.begin(), d->scratchPx.end(), u8(0));
  d->uploadScratches(true);
  d->particles.clear();
}

void Renderer::draw(const Snapshot& s, const Camera& cam, const HudOptions& hud, int w, int h, double dt) {
  Impl& D = *d;
  D.origin = {};  // positions stay within +-512 m, which is fine for float vertex data
  auto tp0 = std::chrono::steady_clock::now();
  D.handleEvents(s);
  D.applyScrapes(s, dt);
  D.xrayNow = hud.xray;
  if (s.splitGeneration != D.seenSplit) {
    if (s.splitGeneration < D.seenSplit) {
      // The world was reset: back to the original binding.
      D.mesh->bindNode = D.pristine.bindNode;
      D.mesh->bindWeight = D.pristine.bindWeight;
    } else {
      D.mesh->rebindAfterTear(D.sim->rest, D.rig->groups, D.spawn, s.nodeObj);
    }
    D.seenSplit = s.splitGeneration;
  }
  auto tp1 = std::chrono::steady_clock::now();
  D.updateCar(s);
  auto tp2 = std::chrono::steady_clock::now();
  D.updateProps(s, cam.eye);
  D.updateTrack(s);
  D.camVP = mul(perspective(cam.fovY, double(w) / double(h), 0.12, 2500.0), lookAt(cam.eye, cam.target, cam.up));
  auto tp3 = std::chrono::steady_clock::now();
  D.stepParticles(dt);
  D.addSkidMarks(s);
  D.buildHud(s, hud, w, h);
  auto tp4 = std::chrono::steady_clock::now();

  // Eye-relative view would be more precise, but +-500 m with 1 mm float steps is good enough.
  const Mat4 view = lookAt(cam.eye, cam.target, cam.up);
  const Mat4 proj = perspective(cam.fovY, double(w) / double(h), 0.12, 2500.0);
  const Mat4 vp = mul(proj, view);
  D.updateMotion(s, cam, proj, dt);
  D.ensureTargets(w, h);

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
  // The far map looks over the car's shoulder, covering what the camera actually sees.
  constexpr double kRangeFar = 420.0;
  V3 focusFar = cam.eye + norm(cam.target - cam.eye) * 260.0;
  {
    const V3 lr = norm(cross(D.sun, V3{0, 0, 1}));
    const V3 lu = cross(lr, D.sun);
    const double texel = 2.0 * kRangeFar / Impl::kShadowFar;
    const double a = std::round(dot(focusFar, lr) / texel) * texel, b = std::round(dot(focusFar, lu) / texel) * texel;
    focusFar = lr * a + lu * b + D.sun * dot(focusFar, D.sun);
  }
  const Mat4 lightFarVP = mul(ortho(-kRangeFar, kRangeFar, -kRangeFar, kRangeFar, 1.0, 900.0), lookAt(focusFar + D.sun * 450.0, focusFar, V3{0, 0, 1}));

  auto drawDyn = [&](GLuint prog) {
    D.setMat(prog, "uModel", I);
    D.setMat(prog, "uPrevWorld", D.carPrevWorld);
    glBindVertexArray(D.carVao);
    glDrawElements(GL_TRIANGLES, GLsizei(D.carIdx.size()), GL_UNSIGNED_INT, nullptr);
    D.setMat(prog, "uPrevWorld", I);
    const GLint uScorchLoc = glGetUniformLocation(prog, "uScorch");
    if (uScorchLoc >= 0) glUniform1f(uScorchLoc, 0.f);
    glBindVertexArray(D.propVao);
    glDrawArrays(GL_TRIANGLES, 0, GLsizei(D.propVerts.size()));
    if (uScorchLoc >= 0) glUniform1f(uScorchLoc, float(s.scorch));
    D.setMat(prog, "uPrevWorld", D.carPrevWorld);
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
      } else if (l.kind == 2) {
        const V3 ax = norm(b - a) * 0.02;
        D.setMat(prog, "uModel", D.rodMatrix(a - ax, b + ax, l.radius));
      } else {
        D.setMat(prog, "uModel", D.rodMatrix(a, b, l.radius));
      }
      glDrawArrays(GL_TRIANGLES, l.mat * D.rodCount, D.rodCount);
    }
    D.setMat(prog, "uModel", I);
    if (s.tank && !D.trackVerts.empty()) {
      glBindVertexArray(D.trackVao);
      glDrawArrays(GL_TRIANGLES, 0, GLsizei(D.trackVerts.size()));
    }
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

  glBindFramebuffer(GL_FRAMEBUFFER, D.shadowFarFbo);
  glViewport(0, 0, Impl::kShadowFar, Impl::kShadowFar);
  glClear(GL_DEPTH_BUFFER_BIT);
  glUseProgram(D.progDepth);
  D.setMat(D.progDepth, "uVP", lightFarVP);
  drawDyn(D.progDepth);
  D.setMat(D.progDepth, "uModel", I);
  glBindVertexArray(D.staticVao);
  glDrawArrays(GL_TRIANGLES, 0, D.staticCount);
  glDisable(GL_POLYGON_OFFSET_FILL);
  glBindFramebuffer(GL_FRAMEBUFFER, D.sceneFbo);
  glViewport(0, 0, w, h);
  D.useBuffers(2);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
  {
    const GLfloat none[4] = {0, 0, 0, 0};
    glClearBufferfv(GL_COLOR, 1, none);
  }

  // Sky.
  D.useBuffers(1);
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
  D.useBuffers(2);
  glUseProgram(D.progMesh);
  D.setMat(D.progMesh, "uVP", vp);
  D.setMat(D.progMesh, "uPrevVP", D.prevVP);
  D.setMat(D.progMesh, "uPrevWorld", I);
  D.setMat(D.progMesh, "uShadowVP", lightVP);
  D.setMat(D.progMesh, "uModel", I);
  glUniform3f(glGetUniformLocation(D.progMesh, "uEye"), float(cam.eye.x), float(cam.eye.y), float(cam.eye.z));
  glUniform3f(glGetUniformLocation(D.progMesh, "uSun"), float(D.sun.x), float(D.sun.y), float(D.sun.z));
  glUniform1f(glGetUniformLocation(D.progMesh, "uAlpha"), 1.f);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, D.shadowTex);
  glUniform1i(glGetUniformLocation(D.progMesh, "uShadow"), 0);
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D_ARRAY, D.scratchTex);
  glActiveTexture(GL_TEXTURE2);
  glBindTexture(GL_TEXTURE_2D, D.detailTex);
  glActiveTexture(GL_TEXTURE3);
  glBindTexture(GL_TEXTURE_2D, D.shadowFarTex);
  glActiveTexture(GL_TEXTURE0);
  glUniform1i(glGetUniformLocation(D.progMesh, "uShadowFar"), 3);
  D.setMat(D.progMesh, "uShadowFarVP", lightFarVP);
  glUniform1i(glGetUniformLocation(D.progMesh, "uScratchTex"), 1);
  glUniform1i(glGetUniformLocation(D.progMesh, "uDetailTex"), 2);
  glUniform1f(glGetUniformLocation(D.progMesh, "uDetail"), 1.f);

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

  glUniform1f(glGetUniformLocation(D.progMesh, "uDetail"), 0.f);
  glUniform1f(glGetUniformLocation(D.progMesh, "uScorch"), float(s.scorch));
  drawDyn(D.progMesh);
  glUniform1f(glGetUniformLocation(D.progMesh, "uScorch"), 0.f);
  glUseProgram(D.progWheel);
  glUniform3f(glGetUniformLocation(D.progWheel, "uEye"), float(cam.eye.x), float(cam.eye.y), float(cam.eye.z));
  glUniform3f(glGetUniformLocation(D.progWheel, "uSun"), float(D.sun.x), float(D.sun.y), float(D.sun.z));
  glUniform1f(glGetUniformLocation(D.progWheel, "uAlpha"), 1.f);
  glUniform1i(glGetUniformLocation(D.progWheel, "uShadow"), 0);
  glUniform1i(glGetUniformLocation(D.progWheel, "uScratchTex"), 1);
  glUniform1i(glGetUniformLocation(D.progWheel, "uDetailTex"), 2);
  glUniform1i(glGetUniformLocation(D.progWheel, "uShadowFar"), 3);
  D.setMat(D.progWheel, "uShadowFarVP", lightFarVP);
  glUniform1f(glGetUniformLocation(D.progWheel, "uDetail"), 0.f);
  glUniform1f(glGetUniformLocation(D.progWheel, "uScorch"), float(s.scorch));
  D.drawWheels(s, D.progWheel, vp, lightVP);

  // From here on nothing writes motion: glass and sparks take on the motion of what is behind them.
  D.useBuffers(1);

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
  glUniform1f(glGetUniformLocation(D.progMesh, "uAlpha"), cam.inside ? 0.14f : 0.80f);
  glBindVertexArray(D.carGlassVao);
  glDrawElements(GL_TRIANGLES, GLsizei(D.glassIdx.size()), GL_UNSIGNED_INT, nullptr);

  glUseProgram(D.progPart);
  D.setMat(D.progPart, "uVP", vp);
  glUniform1f(glGetUniformLocation(D.progPart, "uScale"), float(h) * 1.2f);
  glBindVertexArray(D.partVao);
  glDrawArrays(GL_POINTS, 0, GLsizei(D.partVerts.size()));
  glDepthMask(GL_TRUE);
  glDisable(GL_BLEND);

  // Resolve MSAA, then the motion blur onto the screen.
  glBindFramebuffer(GL_READ_FRAMEBUFFER, D.sceneFbo);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, D.resolveFbo);
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glDrawBuffer(GL_COLOR_ATTACHMENT0);
  glBlitFramebuffer(0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
  glReadBuffer(GL_COLOR_ATTACHMENT1);
  glDrawBuffer(GL_COLOR_ATTACHMENT1);
  glBlitFramebuffer(0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glDrawBuffer(GL_BACK);
  glReadBuffer(GL_BACK);
  glDisable(GL_DEPTH_TEST);
  glUseProgram(D.progPost);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, D.resolveColor);
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, D.resolveVel);
  glActiveTexture(GL_TEXTURE2);
  glBindTexture(GL_TEXTURE_2D, D.resolveDepth);
  glActiveTexture(GL_TEXTURE0);
  glUniform1i(glGetUniformLocation(D.progPost, "uColor"), 0);
  glUniform1i(glGetUniformLocation(D.progPost, "uVel"), 1);
  glUniform1i(glGetUniformLocation(D.progPost, "uDepth"), 2);
  glUniform2f(glGetUniformLocation(D.progPost, "uRes"), float(w), float(h));
  glUniform1f(glGetUniformLocation(D.progPost, "uBlur"), hud.motionBlur ? 1.f : 0.f);
  glUniform1f(glGetUniformLocation(D.progPost, "uNear"), 0.12f);
  glUniform1f(glGetUniformLocation(D.progPost, "uFar"), 2500.f);
  glUniform1f(glGetUniformLocation(D.progPost, "uVignette"), hud.motionBlur ? float(0.28 * clampd((std::fabs(s.speedKmh) - 60.0) / 170.0, 0, 1)) : 0.f);
  D.flash *= float(std::exp(-dt * 9.0));
  glUniform1f(glGetUniformLocation(D.progPost, "uFlash"), D.flash * 0.75f);
  D.setMat(D.progPost, "uSkyVP", D.skyVP);
  D.setMat(D.progPost, "uSkyPrevVP", D.skyPrevVP);
  glBindVertexArray(D.hudVao);
  glDrawArrays(GL_TRIANGLES, 0, 3);

  // HUD.
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glUseProgram(D.progHud);
  glUniform2f(glGetUniformLocation(D.progHud, "uRes"), float(w), float(h));
  glBindVertexArray(D.hudVao);
  glDrawArrays(GL_TRIANGLES, 0, GLsizei(D.hudVerts.size()));
  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  glBindVertexArray(0);
  if (getenv("BEANNG_PROFILE")) {
    auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
    static int n = 0;
    if (++n % 30 == 0)
      fprintf(stderr, "frame: scrapes %.1f car %.1f props %.1f misc %.1f gl %.1f ms\n", ms(tp0, tp1), ms(tp1, tp2), ms(tp2, tp3), ms(tp3, tp4), ms(tp4, std::chrono::steady_clock::now()));
  }
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
