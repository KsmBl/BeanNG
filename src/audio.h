// Engine, tyres, wind and crashes, all synthesised at runtime. No sound files and no dependency on the
// audio device: render() can just as well fill a WAV file, which is how the sound gets tested.
#pragma once
#include <cstdint>
#include <mutex>
#include <random>
#include <vector>

// What the car is doing right now. The game sets this once per frame; the synth glides to it.
struct AudioFeed {
  float rpm = 0;           // engine speed
  float load = 0;          // cylinder fill, 0 overrun .. 1 full throttle
  bool engineOn = false;
  bool starter = false;
  float inputRpm = 0;      // gearbox input shaft, for gear whine
  float speed = 0;         // m/s, absolute
  float skid = 0;          // tyres on tarmac sliding, 0..1
  float dirt = 0;          // share of the rolling sound that is gravel or grass, 0..1
  float rough = 0;         // how hard the road shakes the suspension, 0..1
  float scrape = 0;        // bodywork grinding along something, 0..1
  float listener = 1;      // gain from the camera's distance to the car, 0..1
  float interior = 0;      // 1 with the camera inside the car
  float master = 1;        // volume, already including mute and pause
  float fire = 0;          // wreck burning, 0..1
  int cylinders = 6;       // firings per two revolutions of the crank
  float exhaustPitch = 1;  // scales the exhaust's resonances: a big diesel sits far lower
  float tracks = 0;        // tank tracks running, 0..1
  float turbo = 0;         // boost pressure, 0..1
  float rotor = 0;         // a helicopter, 0..1
  // What the others are driving, as heard from here: their own engines, however far off.
  struct Distant {
    float rpm = 0, load = 0;
    float gain = 0, pan = 0;
    float dull = 0;        // 0 close by .. 1 far off, when only the low end carries
    float cylinders = 8, pitch = 1;
    float tracks = 0;      // m/s of track, for the rattle; 0 on wheels
  };
  static constexpr int kDistant = 6;
  Distant distant[kDistant];
};

enum class Sfx : uint8_t { Metal, Glass, Thump, Clunk, Boom, Gun, Crack };

class AudioSynth {
 public:
  static constexpr int kRate = 48000;

  AudioSynth();
  void setFeed(const AudioFeed& f);
  // strength 0..1. pan -1 left .. 1 right.
  void trigger(Sfx kind, float strength, float pan, float gain);
  // Fills interleaved stereo. Safe to call from the audio thread.
  void render(float* out, int frames);

 private:
  struct Svf {  // state-variable filter, stable for all the frequencies used here
    float lo = 0, band = 0;
    float bandpass(float in, float f, float q) {
      lo += f * band;
      const float hi = in - lo - q * band;
      band += f * hi;
      return band;
    }
  };
  static constexpr int kModes = 7;
  struct Mode {
    float re = 0, im = 0, cs = 1, sn = 0, decay = 0;
  };
  struct Voice {
    bool on = false;
    Sfx kind = Sfx::Metal;
    float gainL = 0, gainR = 0;
    Mode modes[kModes];
    float noise = 0, noiseDecay = 0;
    Svf noiseFilter;
    float noiseF = 0.3f;
    int grainsLeft = 0, grainClock = 0;
    float peak = 0;
    int age = 0;
  };

  std::mutex lock_;
  AudioFeed target_, now_;
  struct Pending { Sfx kind; float strength, pan, gain; };
  std::vector<Pending> pending_;
  std::vector<Voice> voices_;
  std::minstd_rand rng_{4711};

  float noise();
  void startVoice(const Pending& p);

  // The engines of the others: a firing pulse through two resonances, enough to tell what it is and how hard it works.
  struct DistantVoice {
    AudioFeed::Distant now;
    double firing = 0, link = 0;
    float pulse = 0, hit = 0, low = 0;
    Svf a, b, rattle;
  };
  DistantVoice distant_[AudioFeed::kDistant];

  // Engine.
  double crank_ = 0;       // revolutions within the four-stroke cycle, 0..2
  int nextCyl_ = 0;
  float pulse_[2] = {};    // decaying exhaust pulse of each cylinder bank
  float cylGain_[6];
  float intakeEnv_ = 0;
  Svf exhA_[2], exhB_[2], exhC_[2], intake_, valve_, whine_;
  std::vector<float> pipeL_, pipeR_;
  int pipePos_ = 0;
  float dcL_ = 0, dcR_ = 0;
  double whinePhase_ = 0, starterPhase_ = 0;

  // Road and air.
  Svf windA_[2], windB_[2], roadA_[2], gravel_[2], squeal_[3], grind_[3];
  float roadLow_[2] = {};
  double squealLfo_ = 0, gustPhase_ = 0;
  float crackle_ = 0;
  double turboPhase_ = 0;
  float lastTurbo_ = 0, blowoff_ = 0;
  Svf blow_;
  Svf fireF_[2];
  float fireLow_[2] = {};
  float fireEnv_ = 0;
  float trackClank_ = 0;
  double trackPhase_ = 0;
  Svf trackF_[2];
  double rotorPhase_ = 0;
  float rotorEnv_ = 0;
  Svf rotorF_;
};
