// PlaitsVA - oscillateur "virtual analog" pour Korg NTS-1 (nutekt-digital)
// Inspire du moteur Virtual Analog de Plaits (Mutable Instruments),
// Copyright 2016 Emilie Gillet, licence MIT (https://github.com/pichenettes/eurorack).
// Reecriture allegee et independante pour la contrainte memoire du NTS-1.
//
// Deux oscillateurs a forme d'onde variable (pulse -> saw), detune, sub, polyBLEP.
#include "userosc.h"

typedef struct {
  float phi1   = 0.f;
  float phi2   = 0.f;
  float phiSub = 0.f;
  float wave1  = 0.f;   // 0..1 (knob Shape)
  float wave2  = 0.5f;  // 0..1 (Alt)
  float detune = 0.f;   // demi-tons 0..12
  float mix    = 0.5f;  // niveau osc2
  float sub    = 0.f;   // niveau sub
} State;

static State s;

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

static inline float polyblep(float t, float dt) {
  if (t < dt) {
    t /= dt;
    return t + t - t * t - 1.f;
  } else if (t > 1.f - dt) {
    t = (t - 1.f) / dt;
    return t * t + t + t + 1.f;
  }
  return 0.f;
}

// Forme d'onde variable: m 0..0.5 -> pulse 50% vers pulse etroit,
// m 0.5..1 -> fondu vers dent de scie
static inline float variableWave(float phi, float dt, float m) {
  const float a = (m < 0.5f) ? m * 2.f : 1.f;           // etroitesse du pulse
  const float b = (m > 0.5f) ? (m - 0.5f) * 2.f : 0.f;  // fondu vers saw
  const float pw = 0.5f - 0.42f * a;

  float p = (phi < pw) ? 1.f : -1.f;
  p += polyblep(phi, dt);
  float t2 = phi + (1.f - pw);
  if (t2 >= 1.f) t2 -= 1.f;
  p -= polyblep(t2, dt);
  p -= (2.f * pw - 1.f);  // retire la composante continue

  float saw = 2.f * phi - 1.f;
  saw -= polyblep(phi, dt);

  return p * (1.f - b) + saw * b;
}

void OSC_INIT(uint32_t platform, uint32_t api) {
  s = State();
}

void OSC_CYCLE(const user_osc_param_t * const params,
               int32_t *yn,
               const uint32_t frames) {
  const uint16_t pitch1 = params->pitch;
  const uint16_t pitch2 = pitch1 + (uint16_t)(s.detune * 256.f);

  const float w1 = osc_w0f_for_note(pitch1 >> 8, pitch1 & 0xFF);
  const float w2 = osc_w0f_for_note(pitch2 >> 8, pitch2 & 0xFF);

  const float lfo = params->shape_lfo * (1.f / 2147483648.f);
  const float m1 = clampf(s.wave1 + lfo * 0.5f, 0.f, 1.f);
  const float m2 = s.wave2;

  const float g1 = 1.f - 0.5f * s.mix;
  const float g2 = 0.5f + 0.5f * s.mix;

  q31_t * __restrict y = (q31_t *)yn;
  const q31_t * y_e = y + frames;

  for (; y != y_e; ) {
    float sig = g1 * variableWave(s.phi1, w1, m1)
              + g2 * variableWave(s.phi2, w2, m2);
    sig *= 0.5f;

    const float sb = (s.phiSub < 0.5f) ? 1.f : -1.f;
    sig = sig * (1.f - 0.3f * s.sub) + 0.5f * sb * s.sub;

    *(y++) = f32_to_q31(clampf(sig, -1.f, 1.f) * 0.9f);

    s.phi1 += w1;
    if (s.phi1 >= 1.f) s.phi1 -= 1.f;
    s.phi2 += w2;
    if (s.phi2 >= 1.f) s.phi2 -= 1.f;
    s.phiSub += w1 * 0.5f;
    if (s.phiSub >= 1.f) s.phiSub -= 1.f;
  }
}

void OSC_NOTEON(const user_osc_param_t * const params) {
}

void OSC_NOTEOFF(const user_osc_param_t * const params) {
}

void OSC_PARAM(uint16_t index, uint16_t value) {
  switch (index) {
    case k_user_osc_param_id1: {
      const float x = clampf(value * 0.01f, 0.f, 1.f);
      s.detune = x * x * 12.f;   // plus fin pres de zero
      break;
    }
    case k_user_osc_param_id2: s.mix = clampf(value * 0.01f, 0.f, 1.f); break;
    case k_user_osc_param_id3: s.sub = clampf(value * 0.01f, 0.f, 1.f); break;
    case k_user_osc_param_shape:
      s.wave1 = clampf(param_val_to_f32(value), 0.f, 1.f); break;
    case k_user_osc_param_shiftshape:
      s.wave2 = clampf(param_val_to_f32(value), 0.f, 1.f); break;
    default: break;
  }
}
