#include "audio.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr float kPi = 3.14159265f;
constexpr float kRateF = float(AudioSynth::kRate);

float clamp01(float v) { return std::clamp(v, 0.f, 1.f); }
float smooth(float a, float b, float x) {
  const float t = clamp01((x - a) / (b - a));
  return t * t * (3.f - 2.f * t);
}
// Frequency coefficient of the state-variable filter.
float fc(float hz) { return std::min(2.f * std::sin(kPi * std::min(hz, kRateF * 0.2f) / kRateF), 1.2f); }
}  // namespace

AudioSynth::AudioSynth() {
  // No two cylinders of a real engine burn quite alike. That is what gives the idle its grain.
  const float g[6] = {1.00f, 0.93f, 1.05f, 0.97f, 1.03f, 0.90f};
  std::copy(g, g + 6, cylGain_);
  voices_.resize(24);
  pending_.reserve(64);
  // Exhaust pipe length, from the manifold to the silencer and back: gives the note its body.
  pipeL_.assign(size_t(kRateF * 0.0058f), 0.f);
  pipeR_.assign(size_t(kRateF * 0.0058f), 0.f);
}

void AudioSynth::setFeed(const AudioFeed& f) {
  std::lock_guard<std::mutex> g(lock_);
  target_ = f;
}

void AudioSynth::trigger(Sfx kind, float strength, float pan, float gain) {
  std::lock_guard<std::mutex> g(lock_);
  if (pending_.size() < 64) pending_.push_back({kind, strength, pan, gain});
}

float AudioSynth::noise() {
  return float(rng_()) / float(std::minstd_rand::max()) * 2.f - 1.f;
}

void AudioSynth::startVoice(const Pending& p) {
  Voice* v = nullptr;
  for (Voice& c : voices_)
    if (!c.on) {
      v = &c;
      break;
    }
  if (!v) {
    // All busy: steal the one that has been ringing longest.
    v = &voices_[0];
    for (Voice& c : voices_)
      if (c.age > v->age) v = &c;
  }
  *v = Voice{};
  v->on = true;
  v->kind = p.kind;
  const float s = clamp01(p.strength);
  const float ang = (clamp01(p.pan * 0.5f + 0.5f)) * kPi * 0.5f;
  v->gainL = std::cos(ang) * p.gain * 1.41f;
  v->gainR = std::sin(ang) * p.gain * 1.41f;
  auto rnd = [&](float a, float b) { return a + (b - a) * (noise() * 0.5f + 0.5f); };
  auto setMode = [&](Mode& m, float hz, float seconds, float amp) {
    const float w = 2.f * kPi * hz / kRateF;
    m.cs = std::cos(w);
    m.sn = std::sin(w);
    m.re = 0;
    m.im = amp;
    m.decay = std::exp(-1.f / (seconds * kRateF));
  };
  switch (p.kind) {
    case Sfx::Metal: {
      // Sheet steel: inharmonic panel modes over a burst of tearing noise and a heavy low blow.
      setMode(v->modes[0], rnd(55, 85), 0.10f + 0.12f * s, 0.9f * s);
      for (int i = 1; i < kModes; i++) {
        const float hz = rnd(170, 520) * (1.f + float(i) * rnd(0.6f, 1.3f));
        setMode(v->modes[i], hz, rnd(0.05f, 0.20f + 0.25f * s), rnd(0.10f, 0.32f) / (1.f + 0.35f * float(i)));
      }
      v->noise = 0.8f;
      v->noiseDecay = std::exp(-1.f / ((0.03f + 0.06f * s) * kRateF));
      v->noiseF = fc(rnd(1200, 3200));
      break;
    }
    case Sfx::Glass: {
      // Many tiny bright fragments arriving over a moment.
      for (int i = 0; i < kModes; i++) setMode(v->modes[i], rnd(2800, 7500), rnd(0.02f, 0.07f), rnd(0.05f, 0.16f));
      v->noise = 0.35f;
      v->noiseDecay = std::exp(-1.f / (0.05f * kRateF));
      v->noiseF = fc(rnd(4000, 6500));
      v->grainsLeft = 10 + int(14.f * s);
      break;
    }
    case Sfx::Thump: {
      // The suspension hitting a bump, felt more than heard.
      setMode(v->modes[0], rnd(48, 72), 0.11f, 0.9f * s);
      setMode(v->modes[1], rnd(110, 170), 0.05f, 0.35f * s);
      setMode(v->modes[2], rnd(340, 520), 0.018f, 0.10f * s);
      v->noise = 0.15f * s;
      v->noiseDecay = std::exp(-1.f / (0.02f * kRateF));
      v->noiseF = fc(800);
      break;
    }
    case Sfx::Boom: {
      // A detonation: the air itself thumps, then rumbles away over several seconds.
      setMode(v->modes[0], 34, 1.1f, 1.5f * s);
      setMode(v->modes[1], 47, 0.8f, 1.1f * s);
      setMode(v->modes[2], 71, 0.5f, 0.8f * s);
      setMode(v->modes[3], 118, 0.25f, 0.5f * s);
      setMode(v->modes[4], 210, 0.14f, 0.3f * s);
      v->noise = 1.6f * s;
      v->noiseDecay = std::exp(-1.f / (0.32f * kRateF));
      v->noiseF = fc(420);
      break;
    }
    case Sfx::Gun: {
      // The tank gun: a hard crack first, then the thump and a long echo.
      setMode(v->modes[0], 42, 0.65f, 1.3f * s);
      setMode(v->modes[1], 66, 0.4f, 0.9f * s);
      setMode(v->modes[2], 130, 0.16f, 0.6f * s);
      setMode(v->modes[3], 310, 0.05f, 0.5f * s);
      v->noise = 1.8f * s;
      v->noiseDecay = std::exp(-1.f / (0.09f * kRateF));
      v->noiseF = fc(1500);
      break;
    }
    case Sfx::Crack: {
      // Wood splitting: a dry snap, with the hollow knock of the trunk beneath it.
      setMode(v->modes[0], rnd(90, 150), 0.09f, 0.7f * s);
      setMode(v->modes[1], rnd(320, 520), 0.04f, 0.5f * s);
      setMode(v->modes[2], rnd(900, 1500), 0.018f, 0.4f * s);
      setMode(v->modes[3], rnd(2100, 3400), 0.009f, 0.3f * s);
      v->noise = 0.9f * s;
      v->noiseDecay = std::exp(-1.f / (0.022f * kRateF));
      v->noiseF = fc(rnd(2000, 3500));
      break;
    }
    case Sfx::Clunk: {
      setMode(v->modes[0], 95, 0.05f, 0.35f * s);
      setMode(v->modes[1], 410, 0.022f, 0.22f * s);
      setMode(v->modes[2], 1250, 0.010f, 0.08f * s);
      break;
    }
  }
}

void AudioSynth::render(float* out, int frames) {
  // Parameters glide to their targets over the block, or the stepping would be audible as a buzz.
  AudioFeed tgt;
  {
    std::unique_lock<std::mutex> g(lock_, std::try_to_lock);
    if (g.owns_lock()) {
      for (const Pending& p : pending_) startVoice(p);
      pending_.clear();
    }
    tgt = target_;
  }
  const float dtS = 1.f / kRateF;
  const float k = 1.f - std::exp(-dtS / 0.012f);

  for (int fr = 0; fr < frames; fr++) {
    now_.rpm += (tgt.rpm - now_.rpm) * k;
    now_.load += (tgt.load - now_.load) * k;
    now_.inputRpm += (tgt.inputRpm - now_.inputRpm) * k;
    now_.speed += (tgt.speed - now_.speed) * k;
    now_.skid += (tgt.skid - now_.skid) * k * 0.5f;
    now_.dirt += (tgt.dirt - now_.dirt) * k * 0.3f;
    now_.rough += (tgt.rough - now_.rough) * k * 0.5f;
    now_.scrape += (tgt.scrape - now_.scrape) * k * 0.5f;
    now_.listener += (tgt.listener - now_.listener) * k * 0.2f;
    now_.interior += (tgt.interior - now_.interior) * k * 0.2f;
    now_.master += (tgt.master - now_.master) * k * 0.5f;
    now_.fire += (tgt.fire - now_.fire) * k * 0.1f;
    now_.tracks += (tgt.tracks - now_.tracks) * k * 0.3f;
    now_.turbo += (tgt.turbo - now_.turbo) * k * 0.5f;
    now_.exhaustPitch = tgt.exhaustPitch;
    now_.cylinders = tgt.cylinders;
    now_.engineOn = tgt.engineOn;
    now_.starter = tgt.starter;

    float L = 0, R = 0;
    const float n1 = noise(), n2 = noise();

    // ---- Engine: a real V6 fires six times in two revolutions, in turn from the two banks. Each
    // firing is a pressure pulse that rings the exhaust. The pitch comes straight from the crank, so
    // every rev, stall and shift in the simulation is heard exactly as it happens.
    const float rpm = std::max(now_.rpm, 0.f);
    if (rpm > 20.f) {
      const float load = clamp01(now_.load);
      const double revPerSample = double(rpm) / 60.0 / double(kRateF);
      crank_ += revPerSample;
      const double step = 2.0 / double(std::max(now_.cylinders, 1));
      static constexpr int kOrder[6] = {0, 3, 1, 4, 2, 5};  // alternating between the banks
      while (crank_ >= step) {
        crank_ -= step;
        const int cyl = kOrder[nextCyl_ % 6];
        nextCyl_ = (nextCyl_ + 1) % 6;
        const float frac = float(crank_ / revPerSample);
        const float firing = now_.engineOn ? 0.16f + 0.84f * load : 0.05f;
        const float amp = firing * cylGain_[cyl] * (1.f + 0.10f * noise());
        pulse_[cyl & 1] += amp * std::exp(-frac * 0.01f);
        intakeEnv_ += load * 0.35f;
      }
      // Higher revs mean shorter, sharper pulses.
      const float pulseDecay = std::exp(-dtS / std::clamp(0.0022f * 3000.f / std::max(rpm, 800.f), 0.0009f, 0.003f));
      float bank[2];
      for (int b = 0; b < 2; b++) {
        pulse_[b] *= pulseDecay;
        bank[b] = pulse_[b] * (0.86f + 0.14f * (b ? n2 : n1));
      }
      // A little of the opposite bank reaches each ear: both pipes join into one silencer.
      float exL = bank[0] * 0.72f + bank[1] * 0.28f;
      float exR = bank[1] * 0.72f + bank[0] * 0.28f;
      // Pipe reflection.
      const int plen = int(pipeL_.size());
      const float dlL = pipeL_[size_t(pipePos_)], dlR = pipeR_[size_t(pipePos_)];
      const float inL = exL - 0.42f * dlL, inR = exR - 0.42f * dlR;
      pipeL_[size_t(pipePos_)] = inL;
      pipeR_[size_t(pipePos_)] = inR;
      pipePos_ = (pipePos_ + 1) % plen;
      // Silencer: the note sits low at part throttle and opens up with load.
      const float bright = 0.35f + 0.65f * load;
      const float ep = now_.exhaustPitch;
      const float a0 = exhA_[0].bandpass(inL, fc(118 * ep), 0.55f), a1 = exhA_[1].bandpass(inR, fc(118 * ep), 0.55f);
      const float b0 = exhB_[0].bandpass(inL, fc(265 * ep), 0.55f), b1 = exhB_[1].bandpass(inR, fc(265 * ep), 0.55f);
      const float c0 = exhC_[0].bandpass(inL, fc(640 * ep), 0.75f), c1 = exhC_[1].bandpass(inR, fc(640 * ep), 0.75f);
      float eL = a0 * 2.4f + b0 * 1.5f + c0 * 0.9f * bright;
      float eR = a1 * 2.4f + b1 * 1.5f + c1 * 0.9f * bright;

      // Intake roar, in time with the firings.
      intakeEnv_ *= std::exp(-dtS / 0.004f);
      const float roar = intake_.bandpass(n1 * intakeEnv_, fc(900.f + rpm * 0.12f), 0.7f) * smooth(1500, 5000, rpm);
      // Valve train and timing belt.
      const float tick = valve_.bandpass(n2 * (0.5f + 0.5f * pulse_[0]), fc(3300), 0.8f) * smooth(600, 6000, rpm) * 0.05f;
      // Gearbox whine, tied to the input shaft (29 teeth on the input gear).
      whinePhase_ += double(now_.inputRpm) / 60.0 * 29.0 / double(kRateF);
      whinePhase_ -= std::floor(whinePhase_);
      const float whine = float(std::sin(2.0 * 3.14159265358979 * whinePhase_)) * 0.012f * smooth(800, 5000, now_.inputRpm) * (0.5f + 0.5f * load);

      float eng = 0.55f + 0.45f * load;
      // Outside the car the exhaust dominates. Inside it the bulkhead takes the top off the note.
      const float inside = now_.interior;
      eL = eL * eng * (1.f - 0.25f * inside) + roar * (0.75f + 0.4f * inside) + tick + whine;
      eR = eR * eng * (1.f - 0.25f * inside) + roar * (0.75f + 0.4f * inside) + tick + whine;
      L += eL * 0.62f;
      R += eR * 0.62f;
    }
    if (now_.starter) {
      // Starter motor: a coarse whirr that follows its pinion.
      starterPhase_ += 18.0 * (40.0 + double(rpm) * 0.12) / double(kRateF);
      starterPhase_ -= std::floor(starterPhase_);
      const float st = (float(starterPhase_) * 2.f - 1.f) * 0.05f;
      L += st;
      R += st;
    }

    // ---- Turbos: a whistle climbing with boost, and the sigh of the bypass valves when you lift off.
    if (now_.turbo > 0.01f || blowoff_ > 0.001f) {
      turboPhase_ += double(900.f + 3400.f * now_.turbo) / double(kRateF);
      turboPhase_ -= std::floor(turboPhase_);
      const float wh = std::sin(2.f * kPi * float(turboPhase_)) * 0.028f * now_.turbo * now_.turbo;
      if (lastTurbo_ - now_.turbo > 0.0008f && now_.turbo > 0.2f) blowoff_ = std::min(1.f, blowoff_ + 0.012f);
      lastTurbo_ = now_.turbo;
      blowoff_ *= 0.99985f;
      const float sigh = blow_.bandpass(n1 * blowoff_, fc(2200.f), 0.6f) * 0.5f;
      L += wh + sigh;
      R += wh + sigh;
    }

    // ---- Air and road. These are what really tell you how fast you are going.
    const float v = now_.speed;
    if (v > 0.2f) {
      gustPhase_ += 0.37 / double(kRateF);
      if (gustPhase_ > 1.0) gustPhase_ -= 1.0;
      const float gust = 1.f + 0.12f * std::sin(2.f * kPi * float(gustPhase_)) + 0.06f * std::sin(2.f * kPi * float(gustPhase_) * 3.3f);
      const float windAmp = 0.34f * std::pow(v / 55.f, 2.0f) * gust * (1.f - 0.55f * now_.interior);
      const float f1 = fc(240.f + v * 9.f), f2 = fc(1100.f + v * 42.f);
      const float wL = windA_[0].bandpass(n1, f1, 0.9f) * 0.9f + windB_[0].bandpass(n2, f2, 1.2f) * 0.5f;
      const float wR = windA_[1].bandpass(n2, f1, 0.9f) * 0.9f + windB_[1].bandpass(n1, f2, 1.2f) * 0.5f;
      L += wL * windAmp;
      R += wR * windAmp;

      // Tyres on tarmac: a low roar rising with speed, shaken up by a bad surface.
      const float roadGain = (0.05f + 0.20f * std::min(v / 45.f, 1.5f)) * smooth(0.3f, 5.f, v) * (1.f + 1.2f * now_.rough);
      const float rc = 0.012f + v * 0.0009f;
      roadLow_[0] += (n1 - roadLow_[0]) * rc;
      roadLow_[1] += (n2 - roadLow_[1]) * rc;
      const float ra = roadA_[0].bandpass(n2, fc(420.f + v * 9.f), 1.1f), rb = roadA_[1].bandpass(n1, fc(420.f + v * 9.f), 1.1f);
      L += (roadLow_[0] * 2.2f + ra * 0.35f) * roadGain * (1.f - now_.dirt * 0.5f);
      R += (roadLow_[1] * 2.2f + rb * 0.35f) * roadGain * (1.f - now_.dirt * 0.5f);

      // Gravel and grass: stones pinging at random under the car.
      if (now_.dirt > 0.01f) {
        const float rate = std::min(v * 18.f, 700.f) / kRateF;
        if ((n1 * 0.5f + 0.5f) < rate) crackle_ = 1.f;
        crackle_ *= 0.994f;
        const float g0 = gravel_[0].bandpass(n2 * crackle_, fc(1400), 0.8f), g1 = gravel_[1].bandpass(n1 * crackle_, fc(1100), 0.8f);
        const float gGain = now_.dirt * smooth(0.5f, 8.f, v) * 0.55f;
        L += g0 * gGain;
        R += g1 * gGain;
      }
    }

    // ---- Tyre squeal: rubber stick-slip rings at a few pitches that wander as the load changes.
    if (now_.skid > 0.01f) {
      squealLfo_ += 4.3 / double(kRateF);
      if (squealLfo_ > 1.0) squealLfo_ -= 1.0;
      const float wob = std::sin(2.f * kPi * float(squealLfo_));
      const float bend = 1.f + 0.035f * wob + 0.25f * (now_.skid - 0.5f);
      const float exc = n1 * 0.6f;
      const float s = squeal_[0].bandpass(exc, fc(760.f * bend), 0.035f) * 0.75f + squeal_[1].bandpass(exc, fc(1290.f * bend), 0.03f) * 0.6f + squeal_[2].bandpass(exc, fc(2130.f * bend), 0.05f) * 0.2f;
      const float amp = smooth(0.08f, 0.7f, now_.skid) * 0.085f * (1.f - now_.dirt) * (1.f - 0.3f * now_.interior);
      L += s * amp;
      R += s * amp * (0.9f + 0.1f * wob);
    }

    // ---- Bodywork grinding along a wall or the road.
    if (now_.scrape > 0.005f) {
      const float rasp = n2 * (0.45f + 0.55f * std::fabs(n1));
      const float g = grind_[0].bandpass(rasp, fc(1250), 0.20f) * 0.5f + grind_[1].bandpass(rasp, fc(2750), 0.16f) * 0.45f + grind_[2].bandpass(rasp, fc(5200), 0.3f) * 0.3f;
      const float amp = now_.scrape * 0.42f;
      L += g * amp;
      R += g * amp;
    }

    // ---- Tank tracks: every link hitting the sprocket and the road wheels.
    if (now_.tracks > 0.01f) {
      trackPhase_ += double(v) / 0.20 / double(kRateF);
      if (trackPhase_ >= 1.0) {
        trackPhase_ -= 1.0;
        trackClank_ = 0.8f + 0.2f * n1;
      }
      trackClank_ *= 0.9975f;
      const float tk0 = trackF_[0].bandpass(n2 * trackClank_ + trackClank_ * 0.4f, fc(700), 0.5f), tk1 = trackF_[1].bandpass(trackClank_, fc(180), 0.4f);
      const float tGain = now_.tracks * 0.42f;
      L += (tk0 + tk1) * tGain;
      R += (tk0 + tk1) * tGain;
    }

    // ---- The wreck burning: a low roar with crackles.
    if (now_.fire > 0.005f) {
      fireLow_[0] += (n1 - fireLow_[0]) * 0.012f;
      fireLow_[1] += (n2 - fireLow_[1]) * 0.012f;
      if ((n2 * 0.5f + 0.5f) < 0.0009f) fireEnv_ = 0.5f + 0.5f * std::fabs(n1);
      fireEnv_ *= 0.996f;
      const float cr = fireF_[0].bandpass(n1 * fireEnv_, fc(2300), 0.8f);
      L += (fireLow_[0] * 1.7f + cr * 0.4f) * now_.fire * 0.55f;
      R += (fireLow_[1] * 1.7f + cr * 0.4f) * now_.fire * 0.55f;
    }

    L *= now_.listener;
    R *= now_.listener;

    // ---- One-shots: crashes, glass, bumps and shifts. Not attenuated by listener here: their gain was
    // worked out from where they happened.
    for (Voice& vc : voices_) {
      if (!vc.on) continue;
      vc.age++;
      float sum = 0, level = 0;
      for (Mode& m : vc.modes) {
        if (m.decay == 0) continue;
        const float re = m.re * m.cs - m.im * m.sn;
        const float im = m.re * m.sn + m.im * m.cs;
        m.re = re * m.decay;
        m.im = im * m.decay;
        sum += m.im;
        level += std::fabs(m.im) + std::fabs(m.re);
      }
      if (vc.noise > 1e-4f) {
        vc.noise *= vc.noiseDecay;
        sum += vc.noiseFilter.bandpass(n1 * vc.noise, vc.noiseF, 0.6f);
        level += vc.noise;
      }
      if (vc.grainsLeft > 0 && --vc.grainClock <= 0) {
        // Glass: the next fragment lands, restarting one of the bright modes.
        Mode& m = vc.modes[size_t(rng_()) % kModes];
        m.im = 0.05f + 0.13f * (n2 * 0.5f + 0.5f);
        m.re = 0;
        vc.grainClock = 200 + int(rng_() % 1400);
        vc.grainsLeft--;
        level += 1.f;
      }
      L += sum * vc.gainL;
      R += sum * vc.gainR;
      if (level < 1e-3f && vc.grainsLeft <= 0) vc.on = false;
    }

    // ---- Output: remove any DC, then a soft limiter so that a pile-up saturates instead of clipping.
    dcL_ += (L - dcL_) * 0.0008f;
    dcR_ += (R - dcR_) * 0.0008f;
    L = (L - dcL_) * now_.master;
    R = (R - dcR_) * now_.master;
    if (!std::isfinite(L)) L = 0;
    if (!std::isfinite(R)) R = 0;
    out[fr * 2] = std::tanh(L * 1.1f) * 0.92f;
    out[fr * 2 + 1] = std::tanh(R * 1.1f) * 0.92f;
  }
}
