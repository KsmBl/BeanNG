#include "render.h"
#include <array>
#include <epoxy/gl.h>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <random>
#include <unordered_map>
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
uniform int uCraterCount;
uniform vec3 uCraters[24];   // x, y, radius
uniform float uRoadCut;
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
  if(uDetail > 0.0 && uCraterCount > 0){
    // What a blast leaves: broken-up road around the hole, and burnt ground further out.
    float rough = texture(uDetailTex, vPos.xy/1.7).r;
    for(int i = 0; i < uCraterCount; i++){
      float dc = length(vPos.xy - uCraters[i].xy), rc = uCraters[i].z;
      if(dc > rc*2.4) continue;
      if(uRoadCut > 0.5 && N.z > 0.6 && dc < rc*(0.82 + 0.5*rough)) discard;
      float burn = 1.0 - smoothstep(rc*0.55, rc*(1.4 + 1.2*rough), dc);
      albedo = mix(albedo, vec3(0.018, 0.015, 0.012) + 0.06*vec3(0.5, 0.36, 0.2)*rough, burn*0.92);
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
  float fog = 1.0-exp(-d*0.0006);
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
uniform float uBead;
uniform float uWall;
uniform float uHalfW;
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
      float bead = uBead;
      float wall = max(r - bead, 0.0);
      float wrinkle = uDeflate * 0.005 * sin(phi*11.0 + p.y*40.0);
      r2 = (bead - dent) + wall*(1.0 - 0.34*uDeflate) + wrinkle;
      float edge = clamp(wall/uWall, 0.0, 1.0);
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
    p.y += sign(p.y) * bulge * (1.0 - 0.5*abs(p.y)/uHalfW);
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
layout(location=2) in vec2 aUv;
uniform vec2 uRes; out vec4 vCol; out vec2 vUv;
void main(){ gl_Position=vec4(aPos.x/uRes.x*2.0-1.0, 1.0-aPos.y/uRes.y*2.0, 0.0, 1.0); vCol=aCol; vUv=aUv; })";
// Shapes come as plain coloured triangles with a soft fringe. Letters and icons are distance fields.
const char* kHudFS = R"(#version 330 core
in vec4 vCol; in vec2 vUv; out vec4 frag; uniform sampler2D uAtlas;
void main(){
  if(vUv.x < 0.0){ frag = vCol; return; }
  float d = texture(uAtlas, vUv).r;
  float w = max(fwidth(d)*0.75, 0.02);
  frag = vec4(vCol.rgb, vCol.a*smoothstep(0.5-w, 0.5+w, d));
})";


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
  float u = -1.f, v = 0.f;
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
    {0.70f, 0.71f, 0.72f, 0.72f},    // pearl white
    {0.030f, 0.031f, 0.034f, 0.60f}, // exposed carbon, lacquered
    {0.92f, 0.30f, 0.03f, 0.55f},    // orange accents
    {0.40f, 0.33f, 0.21f, 0.22f},    // desert sand paint, matt
    {0.52f, 0.34f, 0.24f, 0.10f},    // skin
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

// Skid marks belong to the road, not to the renderer of the moment: they outlast a change of vehicle.
static std::vector<LineV> gMarks;
static size_t gMarkNext = 0, gMarkCount = 0;

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
    int cx = 0, cy = 0, lod = 0;
  };
  std::vector<Chunk> chunks[2];  // near/far LOD
  GLuint staticVao = 0, staticVbo = 0;
  GLsizei staticCount = 0;  // roads, markings and static boxes
  u32 builtBoxGeneration = 0;
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
      if (!p.bound || p.instance >= 0) continue;
      // Only the outer skin takes scratches. Glass, lenses and everything inside stay as they are.
      const bool skin = p.mat == MM_PAINT || p.mat == MM_BLACK || p.mat == MM_DARK || p.mat == MM_CHROME || p.mat == MM_PEARL || p.mat == MM_CARBON || p.mat == MM_ACCENT || p.mat == MM_SAND;
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
        gMarks.resize(kMarkVerts);
        std::copy(q, q + 6, gMarks.begin() + std::ptrdiff_t(markNext));
        markNext = (markNext + 6) % kMarkVerts;
        markCount = std::min(kMarkVerts, markCount + 6);
        gMarkNext = markNext;
        gMarkCount = markCount;
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
      case SURF_ROCK: r = .26f + n * .10f; g = .25f + n * .09f; b = .23f + n * .08f; break;
      case SURF_SNOW: r = .82f + n * .06f; g = .84f + n * .06f; b = .88f + n * .05f; break;
      default: r = .12f + n * .05f; g = .24f + n * .07f; b = .07f + n * .03f; break;
    }
  }

  void buildTerrain() {
    constexpr int kChunk = 64;
    const int N = World::N;
    for (int lod = 0; lod < 2; lod++)
      for (int cy = 0; cy + kChunk < N; cy += kChunk)
        for (int cx = 0; cx + kChunk < N; cx += kChunk) {
          Chunk ch;
          ch.lod = lod;
          ch.cx = cx;
          ch.cy = cy;
          buildChunk(ch);
          chunks[lod].push_back(ch);
        }
  }
  // The land beyond the map: the same hills as the physics, on a coarser grid, slipping just under the edge
  // of the detailed terrain so that no gap can open between the two.
  GLuint outerVao = 0, outerVbo = 0, outerIbo = 0;
  GLsizei outerCount = 0;
  void buildOuter() {
    constexpr double kStep = 16.0, kFar = 3300.0;
    const int n = int(2.0 * kFar / kStep) + 1;
    std::vector<Vtx> v(size_t(n) * n);
    std::vector<u32> idx;
    for (int j = 0; j < n; j++)
      for (int i = 0; i < n; i++) {
        const double x = -kFar + i * kStep, y = -kFar + j * kStep;
        const bool under = std::fabs(x) < World::kHalf && std::fabs(y) < World::kHalf;
        const double z = world->outerHeight(x, y) - (under ? 0.3 : 0.0);
        const double e = 8.0;
        const V3 nr = norm({world->outerHeight(x - e, y) - world->outerHeight(x + e, y), world->outerHeight(x, y - e) - world->outerHeight(x, y + e), 2.0 * e});
        const u32 h = u32(i) * 374761393u + u32(j) * 668265263u;
        const float t = float((h ^ (h >> 13)) * 1274126177u >> 24) / 255.f;
        Vtx& q = v[size_t(j) * n + i];
        q.x = float(x);
        q.y = float(y);
        q.z = float(z);
        q.nx = float(nr.x);
        q.ny = float(nr.y);
        q.nz = float(nr.z);
        q.r = U8(.12f + t * .05f);
        q.g = U8(.24f + t * .07f);
        q.b = U8(.07f + t * .03f);
        q.a = 0;
      }
    for (int j = 0; j + 1 < n; j++)
      for (int i = 0; i + 1 < n; i++) {
        const double x0 = -kFar + i * kStep, y0 = -kFar + j * kStep;
        // Nothing where the detailed terrain covers it completely.
        if (x0 > -World::kHalf + kStep && x0 + kStep < World::kHalf - kStep && y0 > -World::kHalf + kStep && y0 + kStep < World::kHalf - kStep) continue;
        const u32 a = u32(j * n + i), b = a + 1, c = a + u32(n), d = c + 1;
        idx.insert(idx.end(), {a, b, c, b, d, c});
      }
    outerCount = GLsizei(idx.size());
    glGenVertexArrays(1, &outerVao);
    glGenBuffers(1, &outerVbo);
    glGenBuffers(1, &outerIbo);
    glBindVertexArray(outerVao);
    glBindBuffer(GL_ARRAY_BUFFER, outerVbo);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(v.size() * sizeof(Vtx)), v.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, outerIbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(idx.size() * sizeof(u32)), idx.data(), GL_STATIC_DRAW);
    setupVtxAttribs();
  }
  void buildChunk(Chunk& ch) {
    constexpr int kChunk = 64;
    const int N = World::N;
    const int lod = ch.lod, cx = ch.cx, cy = ch.cy;
    const int step = lod == 0 ? 1 : 4;
    {
      {
        {
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
          ch.count = GLsizei(idx.size());
          ch.center = {cx + kChunk * 0.5 - World::kHalf, cy + kChunk * 0.5 - World::kHalf, world->gridHeight(cx + kChunk / 2, cy + kChunk / 2)};
          if (!ch.vao) {
            glGenVertexArrays(1, &ch.vao);
            glGenBuffers(1, &ch.vbo);
            glGenBuffers(1, &ch.ibo);
          }
          glBindVertexArray(ch.vao);
          glBindBuffer(GL_ARRAY_BUFFER, ch.vbo);
          glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(v.size() * sizeof(Vtx)), v.data(), GL_STATIC_DRAW);
          glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ch.ibo);
          glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(idx.size() * sizeof(u32)), idx.data(), GL_STATIC_DRAW);
          setupVtxAttribs();
        }
      }
    }
  }
  // New craters change the ground, and with it the piece of terrain they lie in.
  u32 seenCarResets = 0;
  size_t cratersDrawn = 0;
  std::vector<World::Crater> cratersKnown;
  void rebuildAround(const World::Crater& c) {
    const double reach = c.r * 1.5 + 2.0;
    for (auto& list : chunks)
      for (Chunk& ch : list) {
        const double x0 = ch.cx - World::kHalf, y0 = ch.cy - World::kHalf;
        if (c.x + reach < x0 - 1 || c.x - reach > x0 + 65 || c.y + reach < y0 - 1 || c.y - reach > y0 + 65) continue;
        buildChunk(ch);
      }
  }
  void updateCraters(const Snapshot& s) {
    if (s.craters.size() == cratersDrawn) return;
    if (s.craters.size() < cratersDrawn) {
      // The world was reset: put the ground back everywhere it was hit.
      for (const World::Crater& c : cratersKnown) rebuildAround(c);
      cratersKnown.clear();
      cratersDrawn = 0;
    }
    for (size_t i = cratersDrawn; i < s.craters.size(); i++) {
      rebuildAround(s.craters[i]);
      cratersKnown.push_back(s.craters[i]);
    }
    cratersDrawn = s.craters.size();
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
      if (road.trail) continue;  // dirt tracks are part of the ground itself
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
      if (road.trail) continue;
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
    builtBoxGeneration = world->boxGeneration();
    if (!staticVao) {
      glGenVertexArrays(1, &staticVao);
      glGenBuffers(1, &staticVbo);
    }
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
  size_t trackDriven = 0;
  void updateTrack(const Snapshot& s) {
    trackVerts.clear();
    trackDriven = 0;
    if (s.tankTracks.empty()) return;
    if (!trackVao) {
      glGenVertexArrays(1, &trackVao);
      glGenBuffers(1, &trackVbo);
      glBindVertexArray(trackVao);
      glBindBuffer(GL_ARRAY_BUFFER, trackVbo);
      setupVtxAttribs();
    }
    const MatDef& md = kMats[MM_TRACK];
    constexpr double kPitch = 0.19, kThick = 0.05, kHalfW = 0.275;
    // The one the camera follows first: it is drawn with its own motion.
    const size_t own = s.selfTrack >= 0 ? size_t(s.selfTrack) : s.tankTracks.size();
    for (size_t t2 = 0; t2 < s.tankTracks.size() * 2 + 2; t2++) {
      if (t2 == 2) trackDriven = trackVerts.size();
      const size_t which = t2 < 2 ? own : t2 / 2 - 1;
      if (which >= s.tankTracks.size() || (t2 >= 2 && which == own)) continue;
      const Snapshot::TankTrack& tt = s.tankTracks[which];
      const int side = int(t2 & 1);
      const std::vector<V3>& path = tt.path[side];
      const size_t n = path.size();
      if (n < 3) continue;
      double total = 0;
      for (size_t i = 0; i < n; i++) total += len(path[(i + 1) % n] - path[i]);
      constexpr int links = 76;  // a track has the number of links it has, whatever it is doing
      const double step = total / links;
      // Above a few m/s single links only strobe at 60 frames a second, as a wagon wheel does. To the
      // eye a fast track is a smooth blur, so here the links close up into one belt.
      const double closeUp = clampd((std::fabs(tt.speed[side]) - 2.5) / 3.0, 0, 1);
      const double kLen = lerpd(step * 0.86, step * 1.04, closeUp);
      // In the blur the pattern stands still instead of crawling or running backwards.
      double along = closeUp >= 1.0 ? 0.0 : std::fmod(tt.phase[side], step);
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
        const V3 w = tt.left[side];
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

  std::vector<u8> parkedDirty;
  std::vector<V3> parkedPrint;
  std::vector<std::array<std::vector<u32>, 3>> parkedLists;  // body, glass, lenses
  std::vector<std::pair<u32, u32>> parkedRange;              // first vertex, one past the last
  std::vector<u32> parkedIdx;
  size_t parkedIdxCount[3] = {};
  bool carBuffersMade = false;
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
    // A parked vehicle costs nothing until something has actually moved it: most of the mesh is theirs.
    const size_t parkedCount = rig->parkedNodes.size();
    if (parkedDirty.size() != parkedCount) {
      parkedDirty.assign(parkedCount, 1);
      parkedPrint.assign(parkedCount, V3{1e30, 0, 0});
      parkedLists.assign(parkedCount, {});
      parkedRange.assign(parkedCount, {u32(nv), 0u});
      for (const MeshPart& p : M.parts)
        if (p.instance >= 0) {
          auto& r = parkedRange[size_t(p.instance)];
          r.first = std::min(r.first, p.firstVert);
          r.second = std::max(r.second, p.firstVert + p.vertCount);
        }
    }
    bool anyParkedDirty = false;
    for (size_t k = 0; k < parkedCount; k++) {
      const auto& range = rig->parkedNodes[k];
      V3 sum{};
      for (u32 n = range.first; n < range.first + range.count; n++) sum += pos[n];
      parkedDirty[k] = !(sum.x == parkedPrint[k].x && sum.y == parkedPrint[k].y && sum.z == parkedPrint[k].z);
      parkedPrint[k] = sum;
      if (parkedDirty[k]) anyParkedDirty = true;
    }
    const auto skip = [&](const MeshPart& p) { return p.instance >= 0 && !parkedDirty[size_t(p.instance)]; };
    const size_t drivenVerts = parkedCount ? size_t(parkedRange[0].first) : nv;
    for (const MeshPart& p : M.parts) {
      if (!p.bound || skip(p)) continue;
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
    std::fill(nrmAccum.begin(), nrmAccum.begin() + long(drivenVerts) * 3, 0.f);
    for (size_t k = 0; k < parkedCount; k++)
      if (parkedDirty[k]) {
        std::fill(nrmAccum.begin() + long(parkedRange[k].first) * 3, nrmAccum.begin() + long(parkedRange[k].second) * 3, 0.f);
        for (auto& l : parkedLists[k]) l.clear();
      }
    constexpr float kTear = 1.45f * 1.45f;
    constexpr float kShatter = 1.04f * 1.04f;
    for (size_t pi = 0; pi < M.parts.size(); pi++) {
      const MeshPart& p = M.parts[pi];
      if (!p.bound || skip(p)) continue;
      const bool isGlass = p.mat == MM_GLASS;
      const bool parked = p.instance >= 0;
      if (partHidden[pi] || (!parked && xrayNow && !isMechanical(p))) continue;
      if (!parked && s.scorch > 0 && (p.mat == MM_GLASS || p.mat == MM_LENS)) continue;
      auto& list = parked ? parkedLists[size_t(p.instance)][isGlass ? 1 : (p.mat == MM_LENS ? 2 : 0)] : (isGlass ? glassIdx : (p.mat == MM_LENS ? lensIdx : carIdx));
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
              spawnGlass(V3{A.x, A.y, A.z}, parked ? V3{} : s.carVel);
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
    const auto finishNormals = [&](size_t from, size_t to) {
      for (size_t v = from; v < to; v++) {
        const float nx = nrmAccum[v * 3], ny = nrmAccum[v * 3 + 1], nz = nrmAccum[v * 3 + 2];
        const float l = std::sqrt(nx * nx + ny * ny + nz * nz) + 1e-12f;
        carVerts[v].nx = nx / l;
        carVerts[v].ny = ny / l;
        carVerts[v].nz = nz / l;
      }
    };
    finishNormals(0, drivenVerts);
    for (size_t k = 0; k < parkedCount; k++)
      if (parkedDirty[k]) finishNormals(parkedRange[k].first, parkedRange[k].second);

    // Only what changed goes to the card: the parked vehicles at the front of each index list, the driven
    // one behind them, and of the vertices the driven one and whichever parked vehicle moved.
    glBindBuffer(GL_ARRAY_BUFFER, carVbo);
    const bool firstUpload = !carBuffersMade;
    if (firstUpload) {
      glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(carVerts.size() * sizeof(Vtx)), carVerts.data(), GL_DYNAMIC_DRAW);
    } else {
      glBufferSubData(GL_ARRAY_BUFFER, 0, GLsizeiptr(drivenVerts * sizeof(Vtx)), carVerts.data());
      for (size_t k = 0; k < parkedCount; k++)
        if (parkedDirty[k]) glBufferSubData(GL_ARRAY_BUFFER, GLsizeiptr(parkedRange[k].first * sizeof(Vtx)), GLsizeiptr((parkedRange[k].second - parkedRange[k].first) * sizeof(Vtx)), carVerts.data() + parkedRange[k].first);
    }
    const GLuint vaos[3] = {carVao, carGlassVao, lensVao};
    const GLuint ibos[3] = {carIbo, carGlassIbo, lensIbo};
    const std::vector<u32>* driven[3] = {&carIdx, &glassIdx, &lensIdx};
    for (int i = 0; i < 3; i++) {
      glBindVertexArray(vaos[i]);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibos[i]);
      if (firstUpload) glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(M.tris.size() * sizeof(u32)), nullptr, GL_DYNAMIC_DRAW);
      if (firstUpload || anyParkedDirty) {
        parkedIdx.clear();
        for (const auto& lists : parkedLists) parkedIdx.insert(parkedIdx.end(), lists[size_t(i)].begin(), lists[size_t(i)].end());
        parkedIdxCount[i] = parkedIdx.size();
        if (!parkedIdx.empty()) glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, GLsizeiptr(parkedIdx.size() * sizeof(u32)), parkedIdx.data());
      }
      if (!driven[i]->empty()) glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(parkedIdxCount[i] * sizeof(u32)), GLsizeiptr(driven[i]->size() * sizeof(u32)), driven[i]->data());
    }
    carBuffersMade = true;
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
    constexpr double kDraw2 = 1000.0 * 1000.0;
    const u32 firstProp = std::max(sim->firstSceneryNode, sim->objs[0].nodeCount);
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
        case EV_ROCK: {
          // Rubble and stone dust.
          const float size = std::max(e.strength, 0.25f);
          const int rubble = 18 + int(size * 30.f);
          for (int i = 0; i < rubble; i++) {
            Particle q;
            const V3 d = norm(V3{rnd(-1, 1), rnd(-1, 1), rnd(.1f, 1.4f)});
            q.p = e.p + d * rnd(0.f, size);
            q.v = d * rnd(2.f, 14.f);
            q.maxLife = q.life = rnd(.8f, 2.2f);
            q.size = rnd(.03f, .10f + size * .08f);
            const float g = rnd(.25f, .48f);
            q.r = g; q.g = g * .97f; q.b = g * .92f; q.a = 1.f;
            q.gravity = 9.81f;
            q.drag = .2f;
            particles.push_back(q);
          }
          for (int i = 0; i < 10 + int(size * 8.f); i++) {
            Particle q;
            q.p = e.p + V3{rnd(-size, size), rnd(-size, size), rnd(0.f, size)};
            q.v = V3{rnd(-2, 2), rnd(-2, 2), rnd(.5f, 3.f)};
            q.maxLife = q.life = rnd(1.2f, 3.5f);
            q.size = rnd(.3f, .7f) * (.6f + size * .5f);
            const float g = rnd(.5f, .62f);
            q.r = g; q.g = g * .96f; q.b = g * .9f; q.a = .28f;
            q.gravity = -.25f;
            q.drag = 1.8f;
            particles.push_back(q);
          }
          break;
        }
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
          // Earth thrown out of the hole.
          if (e.p.z - world->height(e.p.x, e.p.y) < 3.0)
            for (int i = 0; i < 120; i++) {
              Particle q;
              const V3 d = norm(V3{rnd(-1, 1), rnd(-1, 1), rnd(.4f, 1.6f)});
              q.p = e.p + V3{rnd(-1, 1), rnd(-1, 1), 0} * (R * .08);
              q.v = d * rnd(.3f, 1.2f) * R;
              q.maxLife = q.life = rnd(1.2f, 3.2f);
              q.size = rnd(.05f, .22f);
              const float g = rnd(.08f, .22f);
              q.r = g * 1.2f; q.g = g; q.b = g * .75f; q.a = 1.f;
              q.gravity = 9.81f;
              q.drag = .15f;
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

  // ---- UI toolkit. Everything is a coloured triangle or a distance-field glyph, in pixels.
  struct Col { float r, g, b, a; };
  struct Glyph { float u0, v0, u1, v1, xo, yo, w, h, adv; };
  std::unordered_map<u32, Glyph> glyphs;
  GLuint atlasTex = 0;
  bool loadAtlas(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) {
      fprintf(stderr, "%s fehlt: Starte das Spiel mit ./run.sh\n", path);
      return false;
    }
    char magic[4];
    u32 head[3];
    bool ok = fread(magic, 1, 4, f) == 4 && !memcmp(magic, "UIAT", 4) && fread(head, 4, 3, f) == 3;
    if (ok) {
      for (u32 i = 0; i < head[2]; i++) {
        u32 code;
        float m[9];
        if (fread(&code, 4, 1, f) != 1 || fread(m, 4, 9, f) != 9) {
          ok = false;
          break;
        }
        glyphs[code] = Glyph{m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]};
      }
      std::vector<u8> px(size_t(head[0]) * head[1]);
      ok = ok && fread(px.data(), 1, px.size(), f) == px.size();
      if (ok) {
        glGenTextures(1, &atlasTex);
        glBindTexture(GL_TEXTURE_2D, atlasTex);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, GLsizei(head[0]), GLsizei(head[1]), 0, GL_RED, GL_UNSIGNED_BYTE, px.data());
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      }
    }
    fclose(f);
    return ok;
  }
  void tri(float x0, float y0, Col c0, float x1, float y1, Col c1, float x2, float y2, Col c2) {
    hudVerts.push_back({x0, y0, c0.r, c0.g, c0.b, c0.a});
    hudVerts.push_back({x1, y1, c1.r, c1.g, c1.b, c1.a});
    hudVerts.push_back({x2, y2, c2.r, c2.g, c2.b, c2.a});
  }
  // A filled convex shape with a one pixel soft edge. Points run round it in either direction.
  void shape(const std::vector<std::pair<float, float>>& p, Col c) {
    const size_t n = p.size();
    if (n < 3) return;
    float cx = 0, cy = 0;
    for (const auto& q : p) {
      cx += q.first;
      cy += q.second;
    }
    cx /= float(n);
    cy /= float(n);
    Col e = c;
    e.a = 0;
    for (size_t i = 0; i < n; i++) {
      const auto& a = p[i];
      const auto& b = p[(i + 1) % n];
      tri(cx, cy, c, a.first, a.second, c, b.first, b.second, c);
      auto out = [&](const std::pair<float, float>& q, float& x, float& y) {
        const float dx = q.first - cx, dy = q.second - cy, l = std::max(std::sqrt(dx * dx + dy * dy), 1e-3f);
        x = q.first + dx / l;
        y = q.second + dy / l;
      };
      float ax, ay, bx, by;
      out(a, ax, ay);
      out(b, bx, by);
      tri(a.first, a.second, c, b.first, b.second, c, bx, by, e);
      tri(a.first, a.second, c, bx, by, e, ax, ay, e);
    }
  }
  void rrect(float x, float y, float w, float h, float rad, Col c) {
    rad = std::min({rad, w * 0.5f, h * 0.5f});
    std::vector<std::pair<float, float>> p;
    const float cx[4] = {x + w - rad, x + w - rad, x + rad, x + rad}, cy[4] = {y + rad, y + h - rad, y + h - rad, y + rad};
    const float start[4] = {-1.5708f, 0.f, 1.5708f, 3.1416f};
    for (int k = 0; k < 4; k++)
      for (int i = 0; i <= 6; i++) {
        const float a = start[k] + 1.5708f * float(i) / 6.f;
        p.push_back({cx[k] + std::cos(a) * rad, cy[k] + std::sin(a) * rad});
      }
    shape(p, c);
  }
  void circle(float x, float y, float r, Col c) {
    std::vector<std::pair<float, float>> p;
    const int n = std::clamp(int(r * 1.2f), 12, 64);
    for (int i = 0; i < n; i++) {
      const float a = 6.28319f * float(i) / float(n);
      p.push_back({x + std::cos(a) * r, y + std::sin(a) * r});
    }
    shape(p, c);
  }
  // A thick arc, angles in radians clockwise on screen from the right.
  void arc(float x, float y, float r, float th, float a0, float a1, Col c) {
    if (a1 <= a0) return;
    const int n = std::clamp(int((a1 - a0) * r / 5.f) + 2, 3, 96);
    const float ri = r - th * .5f, ro = r + th * .5f;
    Col e = c;
    e.a = 0;
    for (int i = 0; i < n; i++) {
      const float t0 = a0 + (a1 - a0) * float(i) / float(n), t1 = a0 + (a1 - a0) * float(i + 1) / float(n);
      const float c0 = std::cos(t0), s0 = std::sin(t0), c1 = std::cos(t1), s1 = std::sin(t1);
      auto ring = [&](float ra, Col ca, float rb, Col cb) {
        tri(x + c0 * ra, y + s0 * ra, ca, x + c1 * ra, y + s1 * ra, ca, x + c1 * rb, y + s1 * rb, cb);
        tri(x + c0 * ra, y + s0 * ra, ca, x + c1 * rb, y + s1 * rb, cb, x + c0 * rb, y + s0 * rb, cb);
      };
      ring(ri, c, ro, c);
      ring(ri - 1.f, e, ri, c);
      ring(ro, c, ro + 1.f, e);
    }
  }
  void quadGlyph(const Glyph& g, float x, float y, float size, Col c) {
    const float x0 = x + g.xo * size, y0 = y + g.yo * size, x1 = x0 + g.w * size, y1 = y0 + g.h * size;
    const HudV v[6] = {{x0, y0, c.r, c.g, c.b, c.a, g.u0, g.v0}, {x1, y0, c.r, c.g, c.b, c.a, g.u1, g.v0}, {x1, y1, c.r, c.g, c.b, c.a, g.u1, g.v1},
                       {x0, y0, c.r, c.g, c.b, c.a, g.u0, g.v0}, {x1, y1, c.r, c.g, c.b, c.a, g.u1, g.v1}, {x0, y1, c.r, c.g, c.b, c.a, g.u0, g.v1}};
    hudVerts.insert(hudVerts.end(), v, v + 6);
  }
  static u32 nextCode(const char*& s) {
    const u8 c = u8(*s++);
    if (c < 0x80) return c;
    if ((c & 0xE0) == 0xC0 && *s) return u32(c & 0x1F) << 6 | (u8(*s++) & 0x3F);
    if ((c & 0xF0) == 0xE0 && s[0] && s[1]) {
      const u32 a = u8(*s++) & 0x3F, b = u8(*s++) & 0x3F;
      return u32(c & 0x0F) << 12 | a << 6 | b;
    }
    return '?';
  }
  float textWidth(const char* s, float size) {
    float w = 0;
    while (*s) {
      const auto it = glyphs.find(nextCode(s));
      if (it != glyphs.end()) w += it->second.adv * size;
    }
    return w;
  }
  // y is the middle of the line, which is what lining things up in a row wants.
  float label(float x, float y, const char* s, float size, Col c) {
    y -= size * 0.70f;
    while (*s) {
      const auto it = glyphs.find(nextCode(s));
      if (it == glyphs.end()) continue;
      quadGlyph(it->second, x, y, size, c);
      x += it->second.adv * size;
    }
    return x;
  }
  float labelRight(float x, float y, const char* s, float size, Col c) { return label(x - textWidth(s, size), y, s, size, c); }
  float labelMid(float x, float y, const char* s, float size, Col c) { return label(x - textWidth(s, size) * .5f, y, s, size, c); }
  void icon(u32 code, float x, float y, float size, Col c) {
    const auto it = glyphs.find(code);
    if (it != glyphs.end()) quadGlyph(it->second, x, y, size, c);
  }
  void meter(float x, float y, float w, float h, float v, Col c) {
    rrect(x, y, w, h, h * .5f, {1, 1, 1, .13f});
    v = std::clamp(v, 0.f, 1.f);
    if (v > 0.005f) rrect(x, y, std::max(w * v, h), h, h * .5f, c);
  }
  enum { IC_WATER = 0xE000, IC_OIL, IC_FUEL, IC_WRENCH, IC_ENGINE, IC_TYRE, IC_WARN, IC_BRAKE, IC_SKID, IC_CAMERA, IC_CLOCK, IC_SHELL, IC_TURRET, IC_PAD };

  std::vector<Col> mapColors;
  V3 camEye, camTarget;
  void buildHud(const Snapshot& s, const HudOptions& o, int w, int h) {
    hudVerts.clear();
    const float W = float(w), H = float(h);
    const float u = std::max(0.7f, H / 900.f);
    char buf[160];
    const Col card{.035f, .04f, .05f, .62f}, white{1, 1, 1, .95f}, dim{1, 1, 1, .62f}, faint{1, 1, 1, .32f};
    const Col amber{1.f, .72f, .18f, 1}, red{1.f, .27f, .22f, 1}, green{.36f, .86f, .50f, 1}, teal{.35f, .85f, .95f, 1};
    const float kPi2 = 6.28319f;

    if (o.vehicleHud) {
    // ---- Lower right: the round instrument, revs around the speed.
    {
      const float R = 118 * u, cx = W - R - 26 * u, cy = H - R - 26 * u;
      circle(cx, cy, R + 12 * u, card);
      const float a0 = 2.356f, a1 = a0 + 4.712f;  // from lower left, clockwise over the top
      const float rn = float(std::clamp(s.rpm / s.rpmMax, 0.0, 1.0));
      const float red0 = float(s.rpmRed / s.rpmMax);
      arc(cx, cy, R - 5 * u, 11 * u, a0, a0 + (a1 - a0) * red0, {1, 1, 1, .12f});
      arc(cx, cy, R - 5 * u, 11 * u, a0 + (a1 - a0) * red0, a1, {1.f, .25f, .2f, .45f});
      const Col revCol = rn >= red0 ? red : (rn > red0 * .8f ? amber : teal);
      arc(cx, cy, R - 5 * u, 11 * u, a0, a0 + (a1 - a0) * rn, revCol);
      // Scale: one number per thousand, or per five hundred for the slow-revving diesel.
      const double step = s.rpmMax > 4000 ? 1000 : 500;
      for (double t = 0; t <= s.rpmMax + 1; t += step) {
        const float an = a0 + (a1 - a0) * float(t / s.rpmMax);
        const float c = std::cos(an), n = std::sin(an);
        snprintf(buf, sizeof buf, step > 900 ? "%.0f" : "%.1f", t / 1000.0);
        labelMid(cx + c * (R - 27 * u), cy + n * (R - 27 * u), buf, 13 * u, dim);
      }
      snprintf(buf, sizeof buf, "%.0f", std::fabs(s.speedKmh));
      labelMid(cx, cy - 8 * u, buf, 62 * u, white);
      labelMid(cx, cy + 27 * u, "km/h", 15 * u, dim);
      // Gear in a chip below.
      if (s.gear > 0) snprintf(buf, sizeof buf, "%d", s.gear);
      else snprintf(buf, sizeof buf, "%s", s.gear < 0 ? "R" : "N");
      rrect(cx - 21 * u, cy + 44 * u, 42 * u, 34 * u, 9 * u, Col{.35f, .85f, .95f, .22f});
      labelMid(cx, cy + 60 * u, buf, 25 * u, teal);
      if (!s.running) labelMid(cx, cy - 56 * u, "MOTOR AUS", 13 * u, red);
    }

    // ---- Lower left: the vehicle's systems.
    {
      const float cw = 400 * u, ch = 186 * u;
      const float x = 22 * u, y = H - ch - 22 * u;
      rrect(x, y, cw, ch, 14 * u, card);
      float ry = y + 28 * u;
      const float ix = x + 16 * u, tx = x + 44 * u, bx = x + 104 * u, bw = 100 * u, vx = x + 284 * u;
      auto row = [&](u32 ic, const char* name, float v, Col c, const char* value) {
        icon(ic, ix, ry - 10 * u, 20 * u, c.a > 0 ? Col{c.r, c.g, c.b, 1} : dim);
        label(tx, ry, name, 13 * u, dim);
        meter(bx, ry - 4 * u, bw, 8 * u, v, c);
        labelRight(vx, ry, value, 15 * u, white);
        ry += 28 * u;
      };
      const float gx = x + 344 * u, gy = y + 62 * u, gr = 40 * u;
      if (s.foot.inCar) {
        snprintf(buf, sizeof buf, "%.0f %%", s.foot.hp);
        row(IC_ENGINE, "Gesundheit", float(s.foot.hp / 100.0), s.foot.hp < 34 ? red : green, buf);
      } else if (!s.tank) {
        snprintf(buf, sizeof buf, "%.0f °C", s.coolant);
        row(IC_WATER, "Wasser", float((s.coolant - 40.0) / 95.0), s.coolant > 112 ? red : teal, buf);
        snprintf(buf, sizeof buf, "%.0f °C", s.oilTemp);
        row(IC_OIL, "Öl", float((s.oilTemp - 40.0) / 150.0), s.oilTemp > 135 ? red : amber, buf);
        snprintf(buf, sizeof buf, "%.0f %%", s.oilLevel * 100.0);
        row(IC_OIL, "Ölstand", float(s.oilLevel), s.oilLevel < .35 ? red : green, buf);
        if (s.conventional) {
          snprintf(buf, sizeof buf, "%.1f bar", s.boost * 1.4);
          row(IC_ENGINE, "Ladedruck", float(s.boost), s.boost > .85 ? amber : teal, buf);
        } else {
          snprintf(buf, sizeof buf, "%.0f bar", s.accum / 1e5);
          row(IC_WRENCH, "LHM", float(s.accum / 170e5), s.accum > 90e5 ? green : red, buf);
        }
        snprintf(buf, sizeof buf, "%.0f L", s.fuel);
        row(IC_FUEL, "Tank", float(s.fuel / (s.conventional ? 80.0 : 55.0)), s.fuel < 6 ? red : green, buf);
      } else {
        if (s.reload > 0) snprintf(buf, sizeof buf, "%.1f s", s.reload);
        else if (s.tankAmmo >= 0) snprintf(buf, sizeof buf, s.tankAmmo ? "bereit · %d" : "leer", s.tankAmmo);
        else snprintf(buf, sizeof buf, "bereit");
        row(IC_SHELL, "Kanone", s.reload > 0 ? float(1.0 - s.reload / std::max(s.reloadTime, 0.1)) : 1.f, s.reload > 0 ? amber : green, buf);
        snprintf(buf, sizeof buf, "%+.0f°", s.gunPitch * 57.3);
        row(IC_TURRET, "Rohr", float((s.gunPitch * 57.3 + 9.0) / 29.0), teal, buf);
        if (s.missiles > 0 || s.missileLive) {
          snprintf(buf, sizeof buf, s.missileLive ? "%d + fliegt" : "%d", s.missiles);
          row(IC_SHELL, "Raketen", float(s.missiles) / 8.f, teal, buf);
        } else {
          snprintf(buf, sizeof buf, "%.0f L", s.fuel);
          row(IC_FUEL, "Tank", float(s.fuel / 1160.0), green, buf);
        }
        if (s.tankDead) snprintf(buf, sizeof buf, "zerstört");
        else snprintf(buf, sizeof buf, "%.0f %%", s.tankArmour * 100.0);
        row(IC_ENGINE, "Zustand", float(s.tankArmour), s.tankDead || s.tankArmour < 0.34 ? red : (s.tankArmour < 0.67 ? amber : green), buf);
        ry += 28 * u;
      }
      // Pedals, as two bars side by side at the end of the card.
      const float px = x + 335 * u, ph = 42 * u, py = y + 128 * u;
      auto pedal = [&](float xx, double v, Col c) {
        rrect(xx, py, 8 * u, ph, 4 * u, {1, 1, 1, .13f});
        const float f = float(std::clamp(v, 0.0, 1.0));
        if (f > .01f) rrect(xx, py + ph * (1 - f), 8 * u, std::max(ph * f, 8 * u), 4 * u, c);
      };
      pedal(px, s.throttle, green);
      pedal(px + 12 * u, s.brake, red);
      if (!s.tank) {
        // The tyres' friction circle: where the car is using its grip right now.
        circle(gx, gy, gr, {1, 1, 1, .06f});
        arc(gx, gy, gr, 1.2f * u, 0, kPi2, {1, 1, 1, .25f});
        arc(gx, gy, gr * .5f, 1.f * u, 0, kPi2, {1, 1, 1, .14f});
        rrect(gx - gr, gy - .5f * u, 2 * gr, 1 * u, 0, {1, 1, 1, .14f});
        rrect(gx - .5f * u, gy - gr, 1 * u, 2 * gr, 0, {1, 1, 1, .14f});
        const float dx = float(std::clamp(-s.latG, -1.2, 1.2)), dy = float(std::clamp(-s.lonG, -1.2, 1.2));
        circle(gx + dx * gr * .83f, gy + dy * gr * .83f, 5 * u, amber);
        labelMid(gx, gy + gr + 11 * u, "G", 11 * u, faint);
      } else {
        // Hull and gun seen from above: which way the turret points relative to the tracks.
        circle(gx, gy, gr, {1, 1, 1, .06f});
        arc(gx, gy, gr, 1.2f * u, 0, kPi2, {1, 1, 1, .25f});
        rrect(gx - 10 * u, gy - 20 * u, 20 * u, 40 * u, 3 * u, {1, 1, 1, .25f});
        const float ya = float(s.turretYaw);
        const float dx = -std::sin(ya), dy = -std::cos(ya);
        const float nx = -dy, ny = dx;
        const float L = gr - 5 * u, wd = 1.6f * u;
        shape({{gx + nx * wd, gy + ny * wd}, {gx + dx * L + nx * wd, gy + dy * L + ny * wd}, {gx + dx * L - nx * wd, gy + dy * L - ny * wd}, {gx - nx * wd, gy - ny * wd}}, amber);
        circle(gx, gy, 8 * u, amber);
      }
    }
    // Driver aids and drive modes, as a row of small lamps above the card.
    if (!s.tank) {
      float x = 22 * u;
      const float y = H - 186 * u - 22 * u - 40 * u;
      auto lamp = [&](u32 ic, const char* name, bool available, bool working) {
        const float tw = textWidth(name, 13 * u);
        const float wid = 36 * u + tw;
        const Col bg = !available ? Col{.5f, .1f, .08f, .55f} : (working ? Col{1.f, .68f, .12f, .95f} : card);
        const Col fg = !available ? Col{1, .6f, .55f, 1} : (working ? Col{.08f, .06f, .02f, 1} : dim);
        rrect(x, y, wid, 30 * u, 15 * u, bg);
        icon(ic, x + 9 * u, y + 6 * u, 18 * u, fg);
        label(x + 30 * u, y + 15 * u, name, 13 * u, fg);
        x += wid + 7 * u;
      };
      lamp(IC_BRAKE, "ABS", s.absOn, s.absActive);
      lamp(IC_SKID, "ESP", s.espOn, s.espActive);
      lamp(IC_TYRE, "ASR", s.tcsOn, s.tcsActive);
      if (!s.conventional) {
        lamp(IC_WRENCH, s.soft ? "Soft" : "Firm", true, false);
        if (s.activa) lamp(IC_SKID, "Activa", true, false);
        if (s.sport) lamp(IC_ENGINE, "Sport", true, true);
      }
    }

    }
    if (s.foot.onFoot) {
      // On foot there is less to read: how one is doing, and what one carries.
      const float cw = 300 * u, ch = 74 * u, cx0 = 24 * u, cy0 = H - ch - 24 * u;
      rrect(cx0, cy0, cw, ch, 14 * u, card);
      label(cx0 + 16 * u, cy0 + 22 * u, "Gesundheit", 14 * u, dim);
      snprintf(buf, sizeof buf, "%.0f", std::max(s.foot.hp, 0.0));
      labelRight(cx0 + cw - 16 * u, cy0 + 22 * u, buf, 16 * u, s.foot.hp < 34 ? red : white);
      rrect(cx0 + 16 * u, cy0 + 40 * u, cw - 32 * u, 8 * u, 4 * u, {1, 1, 1, .15f});
      rrect(cx0 + 16 * u, cy0 + 40 * u, std::max((cw - 32 * u) * float(std::clamp(s.foot.hp / 100.0, 0.0, 1.0)), 8 * u), 8 * u, 4 * u, s.foot.hp < 34 ? red : green);
      // What is in the hand, and with the number keys what else there is.
      static const char* kNames[5] = {"Kampfmesser", "Pistole", "Sturmgewehr", "Scharfschützengewehr", "Raketenwerfer"};
      const float wx = cx0, wy = cy0 - 58 * u;
      rrect(wx, wy, cw, 50 * u, 14 * u, card);
      snprintf(buf, sizeof buf, "%d  %s", s.foot.weapon + 1, kNames[std::clamp(s.foot.weapon, 0, 4)]);
      label(wx + 16 * u, wy + 20 * u, buf, 15 * u, white);
      if (s.foot.ammo >= 0) {
        snprintf(buf, sizeof buf, "%d", s.foot.ammo);
        labelRight(wx + cw - 16 * u, wy + 20 * u, buf, 18 * u, s.foot.ammo == 0 ? red : white);
      }
      // And what else is carried.
      snprintf(buf, sizeof buf, "Tretminen %d  ·  Panzerminen %d  ·  Reparatursets %d", s.foot.items[0], s.foot.items[1], s.foot.items[2]);
      label(wx, wy - 12 * u, buf, 13 * u, dim);
      snprintf(buf, sizeof buf, "Granaten %d  ·  Lenkraketen %d  ·  Bombencodes %d", s.foot.shells, s.foot.missiles, s.foot.codes);
      label(wx, wy - 30 * u, buf, 13 * u, dim);
      float kx = wx + 16 * u;
      for (int i = 0; i < 5; i++) {
        snprintf(buf, sizeof buf, "%d", i + 1);
        const bool got = s.foot.has & (1 << i);
        kx = label(kx, wy + 40 * u, buf, 13 * u, i == s.foot.weapon ? teal : (got ? dim : Col{1, 1, 1, .15f})) + 10 * u;
      }
    }
    // ---- Upper left: warnings, as pills with a symbol.
    {
      float ty = 20 * u;
      if (s.plastic > 500.0) {
        snprintf(buf, sizeof buf, "Verformung %.0f kJ   Spitze %.0f g", s.plastic / 1e3, s.peakG);
        const float tw = textWidth(buf, 14 * u);
        rrect(20 * u, ty, tw + 24 * u, 28 * u, 14 * u, card);
        label(32 * u, ty + 14 * u, buf, 14 * u, dim);
        ty += 34 * u;
      }
      auto warn = [&](bool on, u32 ic, const char* name) {
        if (!on) return;
        const float tw = textWidth(name, 15 * u);
        rrect(20 * u, ty, tw + 46 * u, 30 * u, 15 * u, {.72f, .10f, .07f, .92f});
        icon(ic, 30 * u, ty + 5 * u, 20 * u, white);
        label(56 * u, ty + 15 * u, name, 15 * u, white);
        ty += 35 * u;
      };
      warn(s.hydraulics, IC_WRENCH, "Stop: Hydraulik");
      warn(s.seized, IC_ENGINE, "Motor defekt");
      warn(s.radiator && !s.seized, IC_WATER, "Kühler leck");
      warn(s.mount, IC_ENGINE, "Motorlager");
      warn(s.steering, IC_WARN, "Lenkung defekt");
      warn(s.flat[0] || s.flat[1] || s.flat[2] || s.flat[3], IC_TYRE, "Reifenschaden");
      warn(s.gearboxBad, IC_WARN, "Getriebe defekt");
      warn(s.tankBad, IC_FUEL, "Tank leck");
      warn(s.oilBad, IC_OIL, "Ölverlust");
    }

    // ---- Upper right: small status.
    {
      float ry = 26 * u;
      const float rx = W - 22 * u;
      const bool behind = !s.paused && s.realtime < s.timeScale * 0.9;
      if (o.stats) {
        snprintf(buf, sizeof buf, "%.0f FPS  ·  %s", o.fps, o.gpuName.c_str());
        labelRight(rx, ry, buf, 14 * u, dim);
        ry += 22 * u;
        snprintf(buf, sizeof buf, "Physik %.0f %%  ·  Echtzeit %.0f %%  ·  wach %d/%d", s.physLoad * 100.0, s.realtime * 100.0, s.awake, s.objectCount);
        labelRight(rx, ry, buf, 14 * u, behind ? red : dim);
        ry += 26 * u;
      } else if (behind) {
        // Whatever the setting: the driver must know when the simulation cannot keep up.
        snprintf(buf, sizeof buf, "Echtzeit %.0f %%", s.realtime * 100.0);
        labelRight(rx, ry, buf, 14 * u, red);
        ry += 26 * u;
      }
      {
        const float tw = textWidth(o.cameraName.c_str(), 13 * u);
        label(rx - tw, ry, o.cameraName.c_str(), 13 * u, dim);
        icon(IC_CAMERA, rx - tw - 24 * u, ry - 9 * u, 18 * u, dim);
        if (o.gamepad) icon(IC_PAD, rx - tw - 50 * u, ry - 9 * u, 18 * u, teal);
      }
      ry += 28 * u;
      auto chip = [&](const char* t, Col c) {
        const float tw = textWidth(t, 14 * u);
        rrect(rx - tw - 22 * u, ry - 13 * u, tw + 22 * u, 26 * u, 13 * u, {c.r, c.g, c.b, .85f});
        label(rx - tw - 11 * u, ry, t, 14 * u, {.05f, .05f, .05f, 1});
        ry += 31 * u;
      };
      if (o.mouseSteer) chip("Mauslenkung", amber);
      if (s.timeScale < 0.99) {
        snprintf(buf, sizeof buf, "Zeitlupe %.2fx", s.timeScale);
        chip(buf, amber);
      }
      if (s.paused && !o.panel.open) chip("Pause", amber);
    }

    // Latest damage message, fading out.
    if (s.damageAge < 4.0 && !s.damageMsg.empty()) {
      const float a = float(std::min(1.0, 4.0 - s.damageAge));
      const float tw = textWidth(s.damageMsg.c_str(), 20 * u);
      rrect((W - tw) * .5f - 22 * u, 22 * u, tw + 44 * u, 40 * u, 20 * u, {.05f, .05f, .06f, .7f * a});
      labelMid(W * .5f, 42 * u, s.damageMsg.c_str(), 20 * u, {1.f, .72f, .3f, a});
    }

    // ---- The vehicle from above, every part coloured by its state.
    if (o.damageView && !s.parts.empty()) {
      const float dw = 84 * u, dh = 202 * u;
      const float dx = W - dw - 30 * u, dy = H - 2 * 130 * u - 26 * u - dh - 20 * u;
      rrect(dx - 12 * u, dy - 30 * u, dw + 24 * u, dh + 42 * u, 14 * u, card);
      icon(IC_WRENCH, dx - 3 * u, dy - 25 * u, 16 * u, dim);
      label(dx + 16 * u, dy - 16 * u, "Zustand", 12 * u, dim);
      const float ux = dw / 100.f, uy = dh / 240.f;
      for (const PartState& p : s.parts) {
        const float d = std::clamp(p.damage, 0.f, 1.f);
        // Green through amber to red, and dark when it is gone.
        Col c{d < .5f ? .25f + d * 1.5f : 1.f, d < .5f ? .82f - d * .1f : .77f - (d - .5f) * 1.3f, .28f, .88f};
        if (d >= .98f) c = {.32f, .05f, .05f, .9f};
        rrect(dx + p.x * ux, dy + p.y * uy, p.w * ux - 1, p.h * uy - 1, 2 * u, c);
      }
    }

    // ---- The scope: the world through a round hole, with its marks and the wind written beside them.
    if (o.scope > 0) {
      const float cx = W * .5f, cy = H * .5f, r = H * .46f;
      const Col black{0, 0, 0, 1}, fine{0, 0, 0, .9f};
      rrect(0, 0, cx - r, H, 0, black);
      rrect(cx + r, 0, W - cx - r, H, 0, black);
      // The edge of the round view, closed with slices of black towards the corners.
      // One slice per pixel row, so that the edge is a true circle at any window size.
      const int kSlices = std::max(int(2.f * r), 1);
      for (int i = 0; i < kSlices; i++) {
        const float t0 = (float(i) + .5f) / float(kSlices);
        const float dy = (t0 * 2.f - 1.f) * r;
        const float half = std::sqrt(std::max(r * r - dy * dy, 0.f));
        const float rowH = 2.f * r / float(kSlices) + .5f;
        rrect(cx - r, cy + dy - rowH * .5f, r - half, rowH, 0, black);
        rrect(cx + half, cy + dy - rowH * .5f, r - half, rowH, 0, black);
      }
      rrect(0, 0, W, cy - r + 1, 0, black);
      rrect(0, cy + r - 1, W, H - cy - r + 1, 0, black);
      rrect(cx - r, cy - .6f * u, r * 2.f, 1.2f * u, 0, fine);
      rrect(cx - .6f * u, cy - r, 1.2f * u, r * 2.f, 0, fine);
      // A mark for every milliradian: to hold over for the drop, or off for the wind.
      const float perMil = float(H) / (2.f * float(std::tan((60.0 / o.scope) * PI / 360.0))) * 0.001f;
      for (int k = -10; k <= 10; k++) {
        if (k == 0) continue;
        const float len0 = (k % 5 == 0 ? 9.f : 5.f) * u;
        rrect(cx + k * perMil - .6f * u, cy - len0, 1.2f * u, len0 * 2.f, 0, fine);
        rrect(cx - len0, cy + k * perMil - .6f * u, len0 * 2.f, 1.2f * u, 0, fine);
      }
      snprintf(buf, sizeof buf, "%.0fx", o.scope);
      label(cx + r * .62f, cy + r * .62f, buf, 22 * u, white);
      const double speed = len(s.foot.wind);
      // Where it comes from, seen from where one looks: from the left means the bullet goes to the right.
      const V3 f = norm(camTarget - camEye);
      const V3 sideways{-f.y, f.x, 0};
      const double across = dot(s.foot.wind, sideways), along = dot(s.foot.wind, V3{f.x, f.y, 0});
      snprintf(buf, sizeof buf, "Wind %.1f m/s  ·  quer %+.1f  ·  längs %+.1f", speed, -across, along);
      labelMid(cx, cy + r * .82f, buf, 15 * u, white);
    } else if (o.crosshair) {
      // The crosshair is where the gunner looks. The ring is where the gun really points just now.
      const float cx = W * .5f, cy = H * .5f;
      const Col line{1, 1, 1, .85f};
      for (float sgn : {-1.f, 1.f}) {
        rrect(cx + (sgn < 0 ? -22 * u : 8 * u), cy - .75f * u, 14 * u, 1.5f * u, 0, line);
        if (sgn > 0) rrect(cx - .75f * u, cy + 8 * u, 1.5f * u, 14 * u, 0, line);
      }
      const V3 p = s.muzzle + s.gunDir * 400.0;
      const Mat4& m = camVP;
      const double qx = m.m[0] * p.x + m.m[4] * p.y + m.m[8] * p.z + m.m[12];
      const double qy = m.m[1] * p.x + m.m[5] * p.y + m.m[9] * p.z + m.m[13];
      const double qw = m.m[3] * p.x + m.m[7] * p.y + m.m[11] * p.z + m.m[15];
      if (qw > 0.1) {
        const float sx = float((qx / qw * .5 + .5) * W), sy = float((.5 - qy / qw * .5) * H);
        const bool ready = s.reload <= 0;
        const float fr = ready ? 1.f : float(1.0 - s.reload / std::max(s.reloadTime, 0.1));
        arc(sx, sy, 14 * u, 2.f * u, 0, kPi2, {1, 1, 1, .22f});
        arc(sx, sy, 14 * u, 2.f * u, -1.5708f, -1.5708f + kPi2 * fr, ready ? green : amber);
      }
    }

    // ---- The map.
    if (o.map.open) {
      float mx0, my0, ms;
      mapLayout(w, h, mx0, my0, ms);
      constexpr int kCells = 110;
      if (mapColors.empty()) {
        mapColors.resize(size_t(kCells) * kCells);
        for (int iy = 0; iy < kCells; iy++)
          for (int ix = 0; ix < kCells; ix++) {
            const double wx = -kMapHalf + (ix + .5) * 2.0 * kMapHalf / kCells, wy = -kMapHalf + (iy + .5) * 2.0 * kMapHalf / kCells;
            const double z = world->height(wx, wy);
            const double shade = 0.8 + 0.2 * clampd((world->height(wx - 10, wy + 10) - z) * 0.12 + 0.5, 0, 1) * 2.0 - 0.2;
            Col c{.30f, .43f, .24f, 1};
            const u8 sf = world->surfaceAt(wx, wy);
            if (world->isWater(wx, wy)) c = {.16f, .34f, .44f, 1};
            else if (sf == SURF_CONCRETE) c = {.52f, .52f, .5f, 1};
            else if (sf == SURF_ROCK) c = {.44f, .42f, .39f, 1};
            else if (sf == SURF_SNOW) c = {.85f, .87f, .9f, 1};
            else c = {.30f + float(z) * .0012f, .43f + float(z) * .0006f, .24f, 1};
            c.r *= float(shade);
            c.g *= float(shade);
            c.b *= float(shade);
            mapColors[size_t(iy) * kCells + ix] = c;
          }
      }
      const auto toMap = [&](double x, double y, float& sx, float& sy) {
        sx = mx0 + float((x + kMapHalf) / (2.0 * kMapHalf)) * ms;
        sy = my0 + float((kMapHalf - y) / (2.0 * kMapHalf)) * ms;
      };
      const float cell = ms / kCells;
      for (int iy = 0; iy < kCells; iy++)
        for (int ix = 0; ix < kCells; ix++) rrect(mx0 + ix * cell, my0 + ms - (iy + 1) * cell, cell + .5f, cell + .5f, 0, mapColors[size_t(iy) * kCells + ix]);
      for (const Road& road : world->roads())
        for (size_t i = 0; i < road.pts.size(); i += 12) {
          float sx, sy;
          toMap(road.pts[i].x, road.pts[i].y, sx, sy);
          rrect(sx - 1 * u, sy - 1 * u, 2 * u, 2 * u, 0, road.trail ? Col{.62f, .54f, .38f, 1} : Col{.86f, .84f, .78f, 1});
        }
      for (const StaticBox& b : world->boxes()) {
        // What stands in the way of a gun: barriers, bridges and the command centres.
        if (b.surface != SURF_CONCRETE) continue;
        float sx, sy;
        toMap(b.c.x, b.c.y, sx, sy);
        const float bw = std::max(float((std::fabs(b.ax[0].x) * b.h.x + std::fabs(b.ax[1].x) * b.h.y) * 2.0 / (2.0 * kMapHalf)) * ms, 2 * u), bh = std::max(float((std::fabs(b.ax[0].y) * b.h.x + std::fabs(b.ax[1].y) * b.h.y) * 2.0 / (2.0 * kMapHalf)) * ms, 2 * u);
        rrect(sx - bw * .5f, sy - bh * .5f, bw, bh, 0, {.12f, .12f, .13f, 1});
      }
      const auto dot2 = [&](V3 p, float r, Col c) {
        float sx, sy;
        toMap(p.x, p.y, sx, sy);
        circle(sx, sy, r, c);
      };
      for (int side = 0; side < 2; side++) dot2(world->base(side).centre, 7 * u, {.95f, .95f, .95f, .9f});
      if (o.map.hasEnemy) dot2(o.map.enemy, 7 * u, {.95f, .25f, .2f, 1});
      if (o.map.hasMark) {
        float sx, sy;
        toMap(o.map.mark.x, o.map.mark.y, sx, sy);
        arc(sx, sy, 11 * u, 2.f * u, 0, kPi2, {1, .55f, .15f, 1});
        rrect(sx - 16 * u, sy - .75f * u, 32 * u, 1.5f * u, 0, {1, .55f, .15f, 1});
        rrect(sx - .75f * u, sy - 16 * u, 1.5f * u, 32 * u, 0, {1, .55f, .15f, 1});
      }
      dot2(o.map.me, 6 * u, {.35f, .85f, .95f, 1});
      {
        float sx, sy;
        toMap(o.map.me.x, o.map.me.y, sx, sy);
        rrect(sx + float(std::cos(o.map.meYaw)) * 9 * u - 2 * u, sy - float(std::sin(o.map.meYaw)) * 9 * u - 2 * u, 4 * u, 4 * u, 2 * u, {.35f, .85f, .95f, 1});
      }
      label(mx0 + 14 * u, my0 + ms - 18 * u, "Klick: Ziel markieren  ·  Rechtsklick: löschen  ·  B: Bombencode  ·  K: schließen", 14 * u, {1, 1, 1, .9f});
    }

    // ---- The chat, above the instruments.
    if (!s.chat.empty() || s.chatOpen) {
      const float cx0 = 24 * u;
      float cy1 = H - 310 * u;
      if (s.chatOpen) {
        const std::string line = "> " + s.chatInput + "_";
        rrect(cx0 - 6 * u, cy1 - 14 * u, std::max(textWidth(line.c_str(), 16 * u) + 16 * u, 420 * u), 28 * u, 8 * u, {.03f, .03f, .04f, .78f});
        label(cx0 + 2 * u, cy1 + 6 * u, line.c_str(), 16 * u, white);
      }
      for (size_t i = s.chat.size(); i-- > 0;) {
        cy1 -= 24 * u;
        const float tw = textWidth(s.chat[i].c_str(), 15 * u);
        rrect(cx0 - 6 * u, cy1 - 12 * u, tw + 16 * u, 23 * u, 6 * u, {.03f, .03f, .04f, .5f});
        label(cx0 + 2 * u, cy1 + 5 * u, s.chat[i].c_str(), 15 * u, white);
      }
    }

    // ---- Lists: the vehicle menu and the settings.
    if (o.panel.open) {
      const UiPanel& P = o.panel;
      float x, y, pw, rowH;
      panelLayout(w, h, P.rows.size(), x, y, pw, rowH);
      const float sc = rowH / 27.f;
      rrect(x, y - 2.0f * rowH, pw, (float(P.rows.size()) + 3.6f) * rowH, 18 * sc, {.04f, .045f, .055f, .88f});
      label(x + 22 * sc, y - 1.05f * rowH, P.title.c_str(), 25 * sc, white);
      for (size_t i = 0; i < P.rows.size(); i++) {
        const float ry = y + float(i) * rowH;
        const float mid = ry + rowH * .46f;
        const bool sel = int(i) == P.sel;
        if (sel) rrect(x + 8 * sc, ry, pw - 16 * sc, rowH - 2 * sc, 8 * sc, {.35f, .85f, .95f, .17f});
        label(x + 22 * sc, mid, P.rows[i].c_str(), 16 * sc, sel ? white : Col{1, 1, 1, .8f});
        const std::string val = i < P.values.size() ? P.values[i] : "";
        const float right = x + pw - 22 * sc;
        if (val == "AN" || val == "AUS") {
          // A switch.
          const bool on = val == "AN";
          const float sw = 40 * sc, sh = 20 * sc;
          rrect(right - sw, mid - sh * .5f, sw, sh, sh * .5f, on ? Col{.35f, .85f, .95f, .9f} : Col{1, 1, 1, .18f});
          circle(on ? right - sh * .5f : right - sw + sh * .5f, mid, sh * .38f, white);
        } else {
          if (!val.empty()) labelRight(right, mid, val.c_str(), 15 * sc, sel ? teal : dim);
          if (i < P.fill.size() && P.fill[i] >= 0) {
            // A slider.
            const float tx = x + pw * .50f, tw = pw * .30f, f = std::clamp(P.fill[i], 0.f, 1.f);
            rrect(tx, mid - 2.5f * sc, tw, 5 * sc, 2.5f * sc, {1, 1, 1, .16f});
            rrect(tx, mid - 2.5f * sc, std::max(tw * f, 5 * sc), 5 * sc, 2.5f * sc, teal);
            circle(tx + tw * f, mid, 7 * sc, white);
          }
        }
      }
      label(x + 22 * sc, y + (float(P.rows.size()) + .75f) * rowH, P.foot.c_str(), 13 * sc, faint);
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
    glUniform1f(glGetUniformLocation(prog, "uBead"), rig->tyreBead);
    glUniform1f(glGetUniformLocation(prog, "uWall"), rig->tyreWall);
    glUniform1f(glGetUniformLocation(prog, "uHalfW"), rig->tyreHalfWidth);
    int wheelIndex = 0;
    glBindVertexArray(wheelVao2);
    const GLint uMode = glGetUniformLocation(prog, "uMode"), uSpin = glGetUniformLocation(prog, "uSpin");
    for (const WheelView& wv : s.wheels) {
      V3 ex, ey, ez;
      wheelAxes(wv, ex, ey, ez);
      {
        // The rear tyres of a hypercar are much wider than the front ones.
        Mat4 mw = wheelMatrix(wv);
        if (wheelIndex >= 2)
          for (int k = 4; k < 7; k++) mw.m[k] *= rig->rearWidth;
        setMat(prog, "uModel", mw);
        wheelIndex++;
      }
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
  d->buildOuter();
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
  d->loadAtlas("assets/ui.atlas");
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
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(Impl::kMarkVerts * sizeof(LineV)), gMarkCount > 0 ? gMarks.data() : nullptr, GL_DYNAMIC_DRAW);
  d->markNext = gMarkNext;
  d->markCount = gMarkCount;
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
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(HudV), (void*)24);
  glBindVertexArray(0);

  glEnable(GL_PROGRAM_POINT_SIZE);
  return true;
}

void Renderer::clearSkidMarks() {
  d->markNext = d->markCount = 0;
  gMarkNext = gMarkCount = 0;
  for (auto& t : d->trail) t.valid = false;
}

void Renderer::resetDamage() {
  // Parked vehicles stay as broken as they are.
  for (size_t i = 0; i < d->partHidden.size(); i++)
    if (d->mesh->parts[i].instance < 0) d->partHidden[i] = 0;
  std::fill(d->scratchPx.begin(), d->scratchPx.end(), u8(0));
  d->uploadScratches(true);
  d->particles.clear();
}

void Renderer::draw(const Snapshot& s, const Camera& cam, const HudOptions& hud, int w, int h, double dt) {
  d->camEye = cam.eye;
  d->camTarget = cam.target;
  Impl& D = *d;
  D.origin = {};  // positions stay within +-512 m, which is fine for float vertex data
  auto tp0 = std::chrono::steady_clock::now();
  D.handleEvents(s);
  D.updateCraters(s);
  D.applyScrapes(s, dt);
  D.xrayNow = hud.xray;
  if (s.carResets != D.seenCarResets) {
    // The car alone went back to its start: it is whole again, whatever became of the rest.
    D.mesh->bindNode = D.pristine.bindNode;
    D.mesh->bindWeight = D.pristine.bindWeight;
    D.seenCarResets = s.carResets;
    D.seenSplit = s.splitGeneration;
  }
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
  if (D.world->boxGeneration() != D.builtBoxGeneration) D.buildStatic();  // a boulder has gone
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
    glDrawElements(GL_TRIANGLES, GLsizei(D.carIdx.size()), GL_UNSIGNED_INT, (void*)(D.parkedIdxCount[0] * sizeof(u32)));
    D.setMat(prog, "uPrevWorld", I);
    const GLint uScorchLoc = glGetUniformLocation(prog, "uScorch");
    if (uScorchLoc >= 0) glUniform1f(uScorchLoc, 0.f);
    // What stands about stands still in the world, and did not burn along with the driven car.
    {
      size_t first = 0;
      for (size_t k = 0; k < D.parkedLists.size(); k++) {
        const size_t count = D.parkedLists[k][0].size();
        if (uScorchLoc >= 0) glUniform1f(uScorchLoc, k < 32 ? s.parkedScorch[k] : 0.f);
        // What the camera follows moves with it. Blurred as if standing still, it would smear while the world stayed sharp.
        const bool self = int(k) == s.selfInstance;
        if (self) D.setMat(prog, "uPrevWorld", D.carPrevWorld);
        if (count) glDrawElements(GL_TRIANGLES, GLsizei(count), GL_UNSIGNED_INT, (void*)(first * sizeof(u32)));
        if (self) D.setMat(prog, "uPrevWorld", I);
        first += count;
      }
      if (uScorchLoc >= 0) glUniform1f(uScorchLoc, 0.f);
    }
    glBindVertexArray(D.propVao);
    glDrawArrays(GL_TRIANGLES, 0, GLsizei(D.propVerts.size()));
    if (uScorchLoc >= 0) glUniform1f(uScorchLoc, float(s.scorch));
    D.setMat(prog, "uPrevWorld", I);
    if (uScorchLoc >= 0) glUniform1f(uScorchLoc, 0.f);
    // Running gear: arms, shafts and struts between their real nodes.
    glBindVertexArray(D.rodVao);
    bool linkSelf = false;
    for (size_t li = 0; li < D.rig->links.size(); li++) {
      const VisLink& l = D.rig->links[li];
      const bool self = li < D.rig->linkInstance.size() && D.rig->linkInstance[li] == s.selfInstance;
      if (self != linkSelf) {
        D.setMat(prog, "uPrevWorld", self ? D.carPrevWorld : I);
        linkSelf = self;
      }
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
    if (!D.trackVerts.empty()) {
      glBindVertexArray(D.trackVao);
      D.setMat(prog, "uPrevWorld", D.carPrevWorld);
      glDrawArrays(GL_TRIANGLES, 0, GLsizei(D.trackDriven));
      D.setMat(prog, "uPrevWorld", I);
      glDrawArrays(GL_TRIANGLES, GLint(D.trackDriven), GLsizei(D.trackVerts.size() - D.trackDriven));
    }
    if (uScorchLoc >= 0) glUniform1f(uScorchLoc, float(s.scorch));
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
  if (hud.farShadows) {
    D.setMat(D.progDepth, "uVP", lightFarVP);
    drawDyn(D.progDepth);
    D.setMat(D.progDepth, "uModel", I);
    glBindVertexArray(D.staticVao);
    glDrawArrays(GL_TRIANGLES, 0, D.staticCount);
  }
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
  glUniform1f(glGetUniformLocation(D.progMesh, "uRoadCut"), 0.f);
  {
    // The newest ones, if there are more than the shader takes.
    const int nc = std::min(int(s.craters.size()), 24);
    GLfloat cv[24 * 3];
    for (int i = 0; i < nc; i++) {
      const World::Crater& c = s.craters[s.craters.size() - size_t(nc) + size_t(i)];
      cv[i * 3] = float(c.x);
      cv[i * 3 + 1] = float(c.y);
      cv[i * 3 + 2] = float(c.r);
    }
    glUniform1i(glGetUniformLocation(D.progMesh, "uCraterCount"), nc);
    if (nc > 0) glUniform3fv(glGetUniformLocation(D.progMesh, "uCraters"), nc, cv);
  }

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
  glBindVertexArray(D.outerVao);
  glDrawElements(GL_TRIANGLES, D.outerCount, GL_UNSIGNED_INT, nullptr);
  glUniform1f(glGetUniformLocation(D.progMesh, "uRoadCut"), 1.f);
  glBindVertexArray(D.staticVao);
  glDrawArrays(GL_TRIANGLES, 0, D.staticCount);
  glUniform1f(glGetUniformLocation(D.progMesh, "uRoadCut"), 0.f);

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
  glUniform1i(glGetUniformLocation(D.progWheel, "uCraterCount"), 0);
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
  glDrawElements(GL_TRIANGLES, GLsizei(D.parkedIdxCount[2] + D.lensIdx.size()), GL_UNSIGNED_INT, nullptr);
  glUniform1f(glGetUniformLocation(D.progMesh, "uAlpha"), cam.inside ? 0.14f : 0.80f);
  glBindVertexArray(D.carGlassVao);
  glDrawElements(GL_TRIANGLES, GLsizei(D.parkedIdxCount[1] + D.glassIdx.size()), GL_UNSIGNED_INT, nullptr);

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
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, D.atlasTex);
  glUniform1i(glGetUniformLocation(D.progHud, "uAtlas"), 0);
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
