#include "AudioEngine.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "Config.h"
#include "Logger.h"
#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define Serial LogSerial

namespace Audio {
namespace {

// Audio is rendered in fixed-size blocks to keep the I2S write cadence stable.
constexpr uint32_t SAMPLE_RATE = 44100;
constexpr size_t CHUNK_SAMPLES = 128;
constexpr uint16_t I2S_DMA_BUF_COUNT = 2;
constexpr uint16_t I2S_DMA_BUF_LEN = 256;
constexpr uint8_t MAX_VOICES = 6;
constexpr uint8_t MAX_NOTE_INDEX = 24;
constexpr uint8_t AUDIO_EVENT_QUEUE_LEN = 48;
constexpr size_t SINE_TABLE_SIZE = 1024;
constexpr float OUTPUT_PCM_SCALE = 22000.0f;
constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 6.28318530717958647692f;

static const float WORMY_FORMANTS[5][3] = {
  {740.0f, 1180.0f, 2550.0f},
  {500.0f, 1860.0f, 2500.0f},
  {310.0f, 2220.0f, 2920.0f},
  {510.0f,  880.0f, 2380.0f},
  {360.0f,  720.0f, 2180.0f}
};

static const float WORMY_FORMANT_GAINS[3] = {0.85f, 0.58f, 0.32f};

enum EnvState : uint8_t {
  ENV_IDLE = 0,
  ENV_ATTACK,
  ENV_SUSTAIN,
  ENV_RELEASE
};

enum AudioEventType : uint8_t {
  AUDIO_EVENT_NOTE_ON = 1,
  AUDIO_EVENT_NOTE_OFF
};

struct AudioEvent {
  AudioEventType type;
  uint8_t noteIndex;
};

// Minimal state for one state-variable formant filter.
struct FormantState {
  float low;
  float band;
};

// One active musical voice. The same struct carries state for all four engines.
struct Voice {
  bool active;
  uint8_t noteIndex;
  OscillatorEngine engine;
  EnvState envState;
  float env;
  float attackStep;
  float releaseStep;
  float freq;
  float phaseA;
  float phaseB;
  float phaseC;
  float phaseD;
  float lp;
  float pluckEnv;
  float noiseEnv;
  uint32_t rng;
  uint32_t startedAt;
  uint8_t vowelIndex;
  float formantCoeff[3];
  float formantDamping[3];
  FormantState formant[3];
};

// Global audio state. New notes use gEngine; existing voices keep their engine.
static Voice gVoices[MAX_VOICES];
static uint32_t gVoiceSeq = 1;
static float gVolume = 0.70f;
static OscillatorEngine gEngine = OSC_ENGINE_LIGHT;
static bool gScaleMinor = true;
static int gScaleBaseMidiNote = 48;
static int gScaleOctaveOffset = 0;
static SpaceFxMode gSpaceFxMode = SPACE_FX_OFF;
static float gReverbAmount = 0.0f;
static bool gI2sReady = false;
static int16_t gOut[CHUNK_SAMPLES * 2];

static float gReverbA[2048];
static float gReverbB[3072];
static uint16_t gReverbIndexA = 0;
static uint16_t gReverbIndexB = 0;
static StaticQueue_t gAudioEventQueueState;
static uint8_t gAudioEventQueueStorage[AUDIO_EVENT_QUEUE_LEN * sizeof(AudioEvent)];
static QueueHandle_t gAudioEventQueue = nullptr;
static bool gAudioEventOverflowLogged = false;
static float gSineTable[SINE_TABLE_SIZE];
static bool gSineTableReady = false;

// Keep normalized control values inside the expected 0..1 range.
static inline float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

// Generic clamp for oscillator and filter parameters.
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Oscillator phases are stored as normalized 0..1 values.
static inline float wrap01(float p) {
  while (p >= 1.0f) p -= 1.0f;
  while (p < 0.0f) p += 1.0f;
  return p;
}

// Fast saturator used instead of tanhf in the real-time audio path.
static inline float fastTanh(float x) {
  x = clampf(x, -3.0f, 3.0f);
  float x2 = x * x;
  return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

// Replace invalid DSP values with silence before they can poison delay buffers.
static inline float finiteOrZero(float x) {
  return isfinite(x) ? x : 0.0f;
}

// Build the sine table once at startup.
static void buildSineTable() {
  for (size_t i = 0; i < SINE_TABLE_SIZE; ++i) {
    gSineTable[i] = sinf(TWO_PI_F * ((float)i / (float)SINE_TABLE_SIZE));
  }
  gSineTableReady = true;
}

// Sine lookup helper; interpolation keeps oscillators smooth without per-sample sinf.
static inline float sine01(float phase) {
  if (!gSineTableReady) return sinf(TWO_PI_F * phase);
  if (phase >= 1.0f) phase -= (int)phase;
  if (phase < 0.0f) phase += 1.0f;

  float pos = phase * (float)SINE_TABLE_SIZE;
  int idx0 = (int)pos;
  if (idx0 >= (int)SINE_TABLE_SIZE) idx0 = 0;
  int idx1 = idx0 + 1;
  if (idx1 >= (int)SINE_TABLE_SIZE) idx1 = 0;
  float frac = pos - (float)idx0;
  return gSineTable[idx0] + (gSineTable[idx1] - gSineTable[idx0]) * frac;
}

// Cheap triangle wave used to add edge to the Light engine.
static inline float triangle01(float phase) {
  return (phase < 0.5f) ? (phase * 4.0f - 1.0f) : (3.0f - phase * 4.0f);
}

// Small deterministic noise generator used by Aurora Light and Wormy.
static uint32_t xorshift(uint32_t& state) {
  uint32_t x = state ? state : 0x9E3779B9u;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  state = x ? x : 0x9E3779B9u;
  return state;
}

// Convert xorshift output to a bipolar floating-point noise signal.
static inline float whiteNoise(uint32_t& state) {
  return ((int32_t)xorshift(state)) / 2147483648.0f;
}

// Standard equal-tempered MIDI note conversion.
static float midiToFrequency(uint8_t note) {
  return 440.0f * powf(2.0f, ((float)note - 69.0f) / 12.0f);
}

// Convert an ASCII letter to lowercase without pulling in locale behavior.
static char lowerAscii(char c) {
  if (c >= 'A' && c <= 'Z') return (char)(c - 'A' + 'a');
  return c;
}

// Return true when s begins with prefix, ignoring ASCII case.
static bool startsWithNoCase(const char* s, const char* prefix) {
  if (!s || !prefix) return false;
  while (*prefix) {
    if (lowerAscii(*s) != lowerAscii(*prefix)) return false;
    ++s;
    ++prefix;
  }
  return true;
}

// Parse root names accepted by PANORYTHE_SCALE.
static bool parseRootSemitone(const char*& p, int& rootSemi) {
  if (!p || !*p) return false;

  switch (lowerAscii(*p++)) {
    case 'c': rootSemi = 0; break;
    case 'd': rootSemi = 2; break;
    case 'e': rootSemi = 4; break;
    case 'f': rootSemi = 5; break;
    case 'g': rootSemi = 7; break;
    case 'a': rootSemi = 9; break;
    case 'b': rootSemi = 11; break;
    default: return false;
  }

  char accidental = lowerAscii(*p);
  if (accidental == '#' || accidental == 's') {
    rootSemi++;
    p++;
  } else if (accidental == 'b') {
    rootSemi--;
    p++;
  }

  while (rootSemi < 0) rootSemi += 12;
  rootSemi %= 12;
  return true;
}

// Parse a signed integer octave.
static bool parseOctave(const char*& p, int& octave) {
  if (!p || !*p) return false;
  int sign = 1;
  if (*p == '-') {
    sign = -1;
    p++;
  }
  if (*p < '0' || *p > '9') return false;

  int value = 0;
  while (*p >= '0' && *p <= '9') {
    value = value * 10 + (*p - '0');
    p++;
  }
  octave = value * sign;
  return true;
}

// Read PANORYTHE_SCALE and cache the base mode/root/octave for note mapping.
static void configureScaleFromCode() {
  const char* p = PANORYTHE_SCALE;
  bool minor = true;

  if (startsWithNoCase(p, "min")) {
    minor = true;
    p += 3;
  } else if (startsWithNoCase(p, "maj")) {
    minor = false;
    p += 3;
  } else {
    p = nullptr;
  }

  int rootSemi = 0;
  int octave = 3;
  bool ok = p &&
            parseRootSemitone(p, rootSemi) &&
            parseOctave(p, octave) &&
            *p == '\0';

  int baseMidi = (octave + 1) * 12 + rootSemi;
  if (!ok || baseMidi < 0 || baseMidi > 127) {
    minor = true;
    baseMidi = 48;
    Serial.println("[Audio] invalid PANORYTHE_SCALE, using minC3.");
  }

  gScaleMinor = minor;
  gScaleBaseMidiNote = baseMidi;
  Serial.printf("[Audio] scale %s, base MIDI note %d\n",
                gScaleMinor ? "minor" : "major",
                gScaleBaseMidiNote);
}

// Map the 16 note pads to a two-octave major or natural minor scale.
static int scaleStepForPad(uint8_t noteIndex) {
  static const int8_t majorSteps[16] = {
    0, 2, 4, 5, 7, 9, 11, 12,
    14, 16, 17, 19, 21, 23, 24, 26
  };
  static const int8_t minorSteps[16] = {
    0, 2, 3, 5, 7, 8, 10, 12,
    14, 15, 17, 19, 20, 22, 24, 26
  };
  uint8_t idx = noteIndex;
  if (idx >= 16) idx = 15;
  return gScaleMinor ? minorSteps[idx] : majorSteps[idx];
}

// Engine-specific attack times keep plucked sounds quick and vocal sounds softer.
static float attackSecondsFor(OscillatorEngine engine) {
  switch (engine) {
    case OSC_ENGINE_WORMY: return 0.030f;
    case OSC_ENGINE_AURORA_LIGHT: return 0.004f;
    case OSC_ENGINE_ORGAN: return 0.012f;
    case OSC_ENGINE_LIGHT:
    default: return 0.014f;
  }
}

// Engine-specific release times define each sound tail.
static float releaseSecondsFor(OscillatorEngine engine) {
  switch (engine) {
    case OSC_ENGINE_WORMY: return 0.34f;
    case OSC_ENGINE_AURORA_LIGHT: return 1.45f;
    case OSC_ENGINE_ORGAN: return 0.26f;
    case OSC_ENGINE_LIGHT:
    default: return 0.42f;
  }
}

// Convert envelope time in seconds to a per-sample linear increment.
static float stepFromSeconds(float seconds) {
  if (seconds <= 0.0f) return 1.0f;
  return 1.0f / (seconds * (float)SAMPLE_RATE);
}

// Reset a voice slot to a silent reusable state.
static void clearVoice(Voice& v) {
  memset(&v, 0, sizeof(v));
  v.envState = ENV_IDLE;
}

// Find a free voice, then prefer released voices, then steal the oldest voice.
static int allocateVoice() {
  int oldest = 0;
  uint32_t oldestSeq = UINT32_MAX;
  float quietestRelease = 2.0f;
  int releaseVoice = -1;

  for (uint8_t i = 0; i < MAX_VOICES; ++i) {
    if (!gVoices[i].active || gVoices[i].envState == ENV_IDLE) return i;
    if (gVoices[i].envState == ENV_RELEASE && gVoices[i].env < quietestRelease) {
      quietestRelease = gVoices[i].env;
      releaseVoice = i;
    }
    if (gVoices[i].startedAt < oldestSeq) {
      oldestSeq = gVoices[i].startedAt;
      oldest = i;
    }
  }
  return (releaseVoice >= 0) ? releaseVoice : oldest;
}

// Initialize all per-voice state for a new note.
static void configureVoice(Voice& v, uint8_t noteIndex) {
  clearVoice(v);
  v.active = true;
  v.noteIndex = noteIndex;
  v.engine = gEngine;
  v.envState = ENV_ATTACK;
  v.attackStep = stepFromSeconds(attackSecondsFor(v.engine));
  v.releaseStep = stepFromSeconds(releaseSecondsFor(v.engine));
  v.freq = midiToFrequency(midiNoteFromNoteIndex(noteIndex));
  v.phaseA = (float)(noteIndex % 7) * 0.071f;
  v.phaseB = 0.25f + (float)(noteIndex % 5) * 0.041f;
  v.phaseC = 0.50f + (float)(noteIndex % 3) * 0.053f;
  v.phaseD = 0.75f + (float)(noteIndex % 11) * 0.019f;
  v.pluckEnv = 1.0f;
  v.noiseEnv = 1.0f;
  v.rng = 0xA341316Cu ^ ((uint32_t)noteIndex * 0x45D9F3Bu) ^ gVoiceSeq;
  v.startedAt = gVoiceSeq++;
  v.vowelIndex = (uint8_t)(noteIndex % 5);

  float shift = 0.90f + 0.035f * (float)(v.noteIndex % 7);
  uint8_t vowel = v.vowelIndex;
  if (vowel > 4) vowel = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    float hz = clampf(WORMY_FORMANTS[vowel][i] * shift, 40.0f, 8000.0f);
    v.formantCoeff[i] = 2.0f * sinf(PI_F * hz / (float)SAMPLE_RATE);
    v.formantDamping[i] = 0.10f + 0.025f * (float)i;
  }
}

// Move a held voice into its release stage.
static void startRelease(Voice& v) {
  if (!v.active || v.envState == ENV_IDLE || v.envState == ENV_RELEASE) return;
  v.envState = ENV_RELEASE;
  v.releaseStep = stepFromSeconds(releaseSecondsFor(v.engine));
}

// Advance the simple attack/sustain/release envelope by one sample.
static void updateEnvelope(Voice& v) {
  if (v.envState == ENV_ATTACK) {
    v.env += v.attackStep;
    if (v.env >= 1.0f) {
      v.env = 1.0f;
      v.envState = ENV_SUSTAIN;
    }
  } else if (v.envState == ENV_RELEASE) {
    v.env -= v.releaseStep;
    if (v.env <= 0.0f) {
      clearVoice(v);
    }
  }
}

// Light: smooth dual-oscillator tone with a filtered edge component.
static float renderLight(Voice& v) {
  float inc = v.freq / (float)SAMPLE_RATE;
  float incB = (v.freq * 1.006f) / (float)SAMPLE_RATE;
  v.phaseA = wrap01(v.phaseA + inc);
  v.phaseB = wrap01(v.phaseB + incB);
  float body = 0.58f * sine01(v.phaseA) + 0.28f * sine01(v.phaseB);
  float edge = 0.18f * triangle01(v.phaseA);
  v.lp += (body + edge - v.lp) * 0.055f;
  return v.lp * 0.85f;
}

// Aurora Light: bell-like pluck with decaying harmonics and a small noise strike.
static float renderAuroraLight(Voice& v) {
  float inc = v.freq / (float)SAMPLE_RATE;
  v.phaseA = wrap01(v.phaseA + inc);
  v.phaseB = wrap01(v.phaseB + inc * 2.006f);
  v.phaseC = wrap01(v.phaseC + inc * 3.014f);
  v.pluckEnv *= 0.99935f;
  v.noiseEnv *= 0.9965f;
  float bell = sine01(v.phaseA) * 0.52f;
  bell += sine01(v.phaseB) * (0.38f * v.pluckEnv);
  bell += sine01(v.phaseC) * (0.20f * v.pluckEnv);
  bell += whiteNoise(v.rng) * (0.045f * v.noiseEnv);
  return bell * 0.95f;
}

// Organ: additive drawbar-style tone with soft saturation.
static float renderOrgan(Voice& v) {
  float inc = v.freq / (float)SAMPLE_RATE;
  v.phaseA = wrap01(v.phaseA + inc);
  v.phaseB = wrap01(v.phaseB + inc * 2.0f);
  v.phaseC = wrap01(v.phaseC + inc * 3.0f);
  v.phaseD = wrap01(v.phaseD + inc * 4.0f);
  float s = 0.62f * sine01(v.phaseA);
  s += 0.23f * sine01(v.phaseB);
  s += 0.14f * sine01(v.phaseC);
  s += 0.08f * sine01(v.phaseD);
  return fastTanh(s * 1.15f) * 0.82f;
}

// Process one state-variable band-pass formant stage.
static float processFormant(FormantState& st, float input, float coeff, float damping) {
  float high = input - st.low - damping * st.band;
  st.band += coeff * high;
  st.low += coeff * st.band;
  return st.band;
}

// Wormy: compact vocal engine built from one glottal source and three formants.
static float renderWormy(Voice& v) {
  float inc = v.freq / (float)SAMPLE_RATE;
  v.phaseA = wrap01(v.phaseA + inc);
  v.phaseB = wrap01(v.phaseB + inc * 0.50f);
  float glottal = (v.phaseA < 0.42f) ? 0.92f : -0.58f;
  glottal += 0.18f * sine01(v.phaseA);
  glottal += 0.05f * whiteNoise(v.rng);

  float out = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) {
    out += WORMY_FORMANT_GAINS[i] * processFormant(v.formant[i],
                                                   glottal,
                                                   v.formantCoeff[i],
                                                   v.formantDamping[i]);
  }
  out += 0.10f * sine01(v.phaseB);
  return fastTanh(out * 1.7f) * 0.72f;
}

// Render one voice sample after envelope processing.
static float renderVoice(Voice& v) {
  updateEnvelope(v);
  if (!v.active) return 0.0f;

  float s = 0.0f;
  switch (v.engine) {
    case OSC_ENGINE_WORMY:
      s = renderWormy(v);
      break;
    case OSC_ENGINE_AURORA_LIGHT:
      s = renderAuroraLight(v);
      break;
    case OSC_ENGINE_ORGAN:
      s = renderOrgan(v);
      break;
    case OSC_ENGINE_LIGHT:
    default:
      s = renderLight(v);
      break;
  }
  return s * v.env;
}

// Warm reverb: two feedback delay lines with modest damping.
static float processReverb(float x) {
  if (gSpaceFxMode != SPACE_FX_WARM_REVERB || gReverbAmount <= 0.001f) {
    return x;
  }

  float amount = clamp01(gReverbAmount);
  float input = fastTanh(finiteOrZero(x) * 0.90f) * 0.85f;
  float a = gReverbA[gReverbIndexA];
  float b = gReverbB[gReverbIndexB];
  float wet = fastTanh(0.58f * a + 0.42f * b);
  float feedback = 0.36f + 0.18f * amount;
  gReverbA[gReverbIndexA] = fastTanh(input + b * feedback);
  gReverbB[gReverbIndexB] = fastTanh(input * 0.70f + a * (feedback * 0.82f));

  gReverbIndexA = (uint16_t)((gReverbIndexA + 1u) % 2048u);
  gReverbIndexB = (uint16_t)((gReverbIndexB + 1u) % 3072u);

  return x * (1.0f - amount * 0.30f) + wet * (amount * 0.50f);
}

// Final soft limiter before converting to 16-bit PCM.
static float softLimit(float x) {
  return fastTanh(finiteOrZero(x) * 1.15f) * 0.90f;
}

// Configure I2S for the PCM5102 output stage.
static void i2sInit() {
  if (gI2sReady) return;

  i2s_config_t cfg{};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_MSB;
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = I2S_DMA_BUF_COUNT;
  cfg.dma_buf_len = I2S_DMA_BUF_LEN;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pins{};
  pins.bck_io_num = I2S_BCLK;
  pins.ws_io_num = I2S_LRCK;
  pins.data_out_num = I2S_DOUT;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  esp_err_t installErr = i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
  esp_err_t pinErr = ESP_FAIL;
  if (installErr == ESP_OK) {
    pinErr = i2s_set_pin(I2S_NUM_0, &pins);
    if (pinErr == ESP_OK) {
      i2s_zero_dma_buffer(I2S_NUM_0);
    }
  }
  gI2sReady = (installErr == ESP_OK && pinErr == ESP_OK);
  if (gI2sReady) {
    Serial.println("[Audio] I2S ready");
  } else {
    Serial.printf("[Audio] I2S init failed install=%d pin=%d\n",
                  (int)installErr, (int)pinErr);
  }
}

// Render one stereo block into gOut. Both channels currently carry the same mix.
static void renderChunk() {
  uint8_t activeCount = 0;
  for (uint8_t i = 0; i < MAX_VOICES; ++i) {
    if (gVoices[i].active) activeCount++;
  }
  float polyTrim = 1.0f;
  if (activeCount > 1) {
    polyTrim = 1.0f / sqrtf((float)activeCount);
  }

  for (size_t i = 0; i < CHUNK_SAMPLES; ++i) {
    float mix = 0.0f;
    for (uint8_t v = 0; v < MAX_VOICES; ++v) {
      if (!gVoices[v].active) continue;
      mix += renderVoice(gVoices[v]);
    }
    mix *= polyTrim;
    mix = softLimit(mix);
    mix = processReverb(mix);
    mix = softLimit(mix * gVolume);

    float pcm = finiteOrZero(mix) * OUTPUT_PCM_SCALE;
    if (pcm > 32767.0f) pcm = 32767.0f;
    if (pcm < -32768.0f) pcm = -32768.0f;
    int16_t s = (int16_t)pcm;
    gOut[i * 2] = s;
    gOut[i * 2 + 1] = s;
  }
}

// Start a note on the current engine. Called only by the audio task.
static void startQueuedNote(uint8_t noteIndex) {
  if (noteIndex >= MAX_NOTE_INDEX) return;
  int slot = allocateVoice();
  configureVoice(gVoices[slot], noteIndex);
}

// Release every active voice that belongs to this pad. Called only by the audio task.
static void releaseQueuedNote(uint8_t noteIndex) {
  for (uint8_t i = 0; i < MAX_VOICES; ++i) {
    if (gVoices[i].active && gVoices[i].noteIndex == noteIndex) {
      startRelease(gVoices[i]);
    }
  }
}

// Consume input events at block boundaries so the input task never edits voices mid-render.
static void processAudioEvents() {
  if (!gAudioEventQueue) return;

  AudioEvent ev{};
  while (xQueueReceive(gAudioEventQueue, &ev, 0) == pdTRUE) {
    if (ev.type == AUDIO_EVENT_NOTE_ON) {
      startQueuedNote(ev.noteIndex);
    } else if (ev.type == AUDIO_EVENT_NOTE_OFF) {
      releaseQueuedNote(ev.noteIndex);
    }
  }
}

// Queue a note event from the input task without blocking the real-time system.
static void queueAudioEvent(AudioEventType type, uint8_t noteIndex) {
  if (noteIndex >= MAX_NOTE_INDEX) return;
  if (!gAudioEventQueue) return;

  AudioEvent ev{type, noteIndex};
  if (xQueueSend(gAudioEventQueue, &ev, 0) != pdTRUE && !gAudioEventOverflowLogged) {
    gAudioEventOverflowLogged = true;
    Serial.println("[Audio] event queue full");
  }
}

} // namespace

// Public setup: clear DSP state and start I2S.
void init() {
  configureScaleFromCode();
  buildSineTable();
  if (!gAudioEventQueue) {
    gAudioEventQueue = xQueueCreateStatic(
      AUDIO_EVENT_QUEUE_LEN,
      sizeof(AudioEvent),
      gAudioEventQueueStorage,
      &gAudioEventQueueState
    );
  } else {
    xQueueReset(gAudioEventQueue);
  }
  gAudioEventOverflowLogged = false;

  for (uint8_t i = 0; i < MAX_VOICES; ++i) {
    clearVoice(gVoices[i]);
  }
  memset(gReverbA, 0, sizeof(gReverbA));
  memset(gReverbB, 0, sizeof(gReverbB));
  gReverbIndexA = 0;
  gReverbIndexB = 0;
  i2sInit();
  Serial.println("[Audio] engine ready");
}

// Audio task entry point: render one block and block until I2S accepts it.
void update() {
  if (!gI2sReady) {
    delay(1);
    return;
  }

  processAudioEvents();
  renderChunk();
  size_t written = 0;
  const size_t expected = sizeof(gOut);
  esp_err_t err = i2s_write(I2S_NUM_0, (const char*)gOut, expected,
                            &written, portMAX_DELAY);
  if (err != ESP_OK || written != expected) {
    Serial.printf("[Audio] I2S write error err=%d bytes=%u/%u\n",
                  (int)err, (unsigned)written, (unsigned)expected);
  }
}

// Reserved low-priority service hook.
void service() {
}

// Queue a note-on for the audio task.
void noteOn(uint8_t noteIndex) {
  queueAudioEvent(AUDIO_EVENT_NOTE_ON, noteIndex);
}

// Queue a note-off for the audio task.
void noteOff(uint8_t noteIndex) {
  queueAudioEvent(AUDIO_EVENT_NOTE_OFF, noteIndex);
}

// Convert pad index to the MIDI note defined by PANORYTHE_SCALE plus transpose.
uint8_t midiNoteFromNoteIndex(uint8_t noteIndex) {
  int note = gScaleBaseMidiNote + scaleStepForPad(noteIndex) + gScaleOctaveOffset * 12;
  if (note < 0) note = 0;
  if (note > 127) note = 127;
  return (uint8_t)note;
}

// Set master volume for future blocks.
void setVolume(float vol) {
  gVolume = clamp01(vol);
}

// Select the engine used by subsequent note-ons.
void setOscillatorEngine(OscillatorEngine engine) {
  if (engine >= OSC_ENGINE_COUNT) engine = OSC_ENGINE_LIGHT;
  gEngine = engine;
}

// Return the current engine selection.
OscillatorEngine getOscillatorEngine() {
  return gEngine;
}

// All enum values are allowed because the firmware only exposes four engines.
bool computeAllowsOscillatorEngine(OscillatorEngine engine) {
  return engine < OSC_ENGINE_COUNT;
}

// Select the active space effect.
void setSpaceFxMode(SpaceFxMode mode) {
  if (mode >= SPACE_FX_COUNT) mode = SPACE_FX_OFF;
  gSpaceFxMode = mode;
}

// Return the active space effect.
SpaceFxMode getSpaceFxMode() {
  return gSpaceFxMode;
}

// Set warm reverb depth.
void setReverbAmount(float amount) {
  gReverbAmount = clamp01(amount);
}

// Return warm reverb depth.
float getReverbAmount() {
  return gReverbAmount;
}

// Reset all effects to a dry state.
void clearAllEffects() {
  gSpaceFxMode = SPACE_FX_OFF;
  gReverbAmount = 0.0f;
}

// Shift the pad map by whole octaves.
void setScaleOctaveOffset(int offset) {
  if (offset < -2) offset = -2;
  if (offset > 2) offset = 2;
  gScaleOctaveOffset = offset;
}

// Return the current octave offset.
int getScaleOctaveOffset() {
  return gScaleOctaveOffset;
}

} // namespace Audio
