/**
 * NFC Activation — Arduino Nano ESP32 (ABX00083)
 * -----------------------------------------------
 * Hardware:
 *   - Elechouse PN532 NFC Module v3 (I2C mode)
 *   - WS2812B 4" LED Ring (40 LEDs)
 *
 * Features:
 *   - Dynamic state machine with unlimited custom states
 *   - Multi-color gradients (up to 8 colors per state)
 *   - Per-state duration with configurable return-to state
 *   - States persist to NVS flash
 *   - JSON config export/import via serial
 *
 * Default States:
 *   - idle:    chase animation (blue) — runs indefinitely
 *   - success: breathe animation (green) — 1500ms → idle
 *   - failure: breathe animation (red) — 1500ms → idle
 *
 * Serial Protocol:
 *   - ESP32 sends: NFC UID as decimal Wave-ID "full forward"
 *   - ESP32 sends: "[hb] <millis>" heartbeat every HEARTBEAT_MS
 *   - ESP32 sends: "[cfg] ..." config dump lines
 *   - ESP32 sends: "[ok] ..." / "[err] ..." command acknowledgements
 *   - ESP32 sends: "[state] <name>" when state changes
 *
 *   - Host sends: "OK" / "FAIL" — trigger success/failure states
 *   - Host sends: "TRIGGER <name>" — trigger any state by name
 *   - Host sends: "STATUS" — re-emit boot banner
 *   - Host sends: "DUMP" — dump all states as JSON
 *   - Host sends: "SAVE" — persist to NVS
 *   - Host sends: "RESET" — wipe NVS, reload defaults
 *   - Host sends: "SET <state> <param> <val>" — modify state param
 *   - Host sends: "ADDSTATE <name>" — create new state
 *   - Host sends: "DELSTATE <name>" — delete state (not default 3)
 *   - Host sends: "ADDCOLOR <state> <r> <g> <b>" — add color to gradient
 *   - Host sends: "DELCOLOR <state> <index>" — remove color from gradient
 *   - Host sends: "SETCOLOR <state> <index> <r> <g> <b>" — modify color
 *   - Host sends: "IMPORT <json>" — import full config (multi-line, ends with ENDIMPORT)
 *   - Host sends: "ADDRESET <uid>" — designate a UID as a reset card (no arg = last scanned)
 *   - Host sends: "DELRESET <uid>" — remove a reset card
 *   - Host sends: "LISTRESET" — list reset card UIDs
 *
 * Reset cards:
 *   - When a reset card is tapped, the ring hard-resets to RESET_HOME_STATE
 *     ("idle"), re-enables tap intake, and clears any in-flight scan. This
 *     bypasses the acceptTaps gate and the success/failure/reading guards, so
 *     it recovers the activation even if it has wedged. Reset cards emit
 *     "[reset] <uid>" instead of "[scan] ..." — the host never sees them as a tap.
 *
 * Libraries: Adafruit PN532, FastLED, ArduinoJson
 * Board: Arduino ESP32 Boards — "Arduino Nano ESP32"
 */

// ── Limits ───────────────────────────────────────────────────────────────────
// Must be defined before #includes so the Arduino IDE's auto-generated function
// prototypes (inserted after the last #include) can resolve LedState/LedColor.
#define MAX_STATES        16
#define MAX_COLORS        8
#define MAX_STATE_NAME    15
#define MAX_BRIGHTNESS    60

// ── Reset cards ────────────────────────────────────────────────────────────────
#define MAX_RESET_UIDS    8
#define MAX_UID_STR       24       // matches nfcPendingUid sizing
#define RESET_HOME_STATE  "idle"   // state a reset card forces the ring back to

// ── Animation Preset Enum ────────────────────────────────────────────────────
enum PresetId : uint8_t {
  P_NOISE=0, P_CHASE=1, P_BREATHE=2, P_WAVE=3,
  P_SPARKLE=4, P_RAINBOW=5, P_SOLID=6, P_GRADIENT=7,
  P_UNKNOWN=255
};

enum TransitionId : uint8_t {
  T_NONE=0, T_CROSSFADE=1, T_FADE_BLACK=2, T_FLASH_WHITE=3
};

// ── State Types ───────────────────────────────────────────────────────────────
struct LedColor {
  uint8_t r, g, b;
  float pos;  // 0.0–1.0 ring position; auto-distributed when not specified
};

struct LedState {
  char name[MAX_STATE_NAME + 1];
  PresetId preset;
  float speed;
  float scale;
  float threshold;
  float gamma;
  uint8_t brightness;
  uint8_t breaths;
  uint16_t duration;              // 0 = indefinite
  char returnTo[MAX_STATE_NAME + 1];  // empty = stay in state
  uint8_t colorCount;
  LedColor colors[MAX_COLORS];
  bool isDefault;                 // true for idle/success/failure (can't delete)
  bool randomColor;               // noise/sparkle: map noise value → gradient instead of ring position
  TransitionId transition;        // how to enter this state (T_NONE = instant)
  uint16_t transitionMs;          // transition duration in ms (0 = type default)
};

#include <Wire.h>
#include <Adafruit_PN532.h>
#include <FastLED.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// ── Pin Definitions ──────────────────────────────────────────────────────────
#define PN532_SDA     A4
#define PN532_SCL     A5
#define PN532_IRQ     255
#define PN532_RESET   255
#define LED_DATA_PIN  5
#define LED_COUNT     40

// ── Timing ───────────────────────────────────────────────────────────────────
#define TAP_COOLDOWN_MS      1500
#define CARD_RELEASE_MS      400    // field must read empty this long before re-arming a tap
#define HEARTBEAT_MS         5000
#define NFC_HEALTH_CHECK_MS  5000   // how often to probe PN532 while connected
#define NFC_RETRY_MS         3000   // how often to attempt re-init when lost
#define READING_MIN_DEFAULT  600    // default minimum reading state display time
#define READ_SETTLE_MS       8      // settle delay after select, before reading pages
#define READ_RETRY_MS        2800   // keep re-reading the NDEF this long after a tap
                                    // (< reading.duration 3000 so a late read can't
                                    //  fire success after the reading timeout flips)

uint16_t readingMinMs = READING_MIN_DEFAULT;
volatile bool acceptTaps = true;       // global gate: when false, NFC polling is paused
bool ledEnabled = true;                // when false, the ring is cleared and FastLED.show() is
                                       // skipped entirely (no LED current, no data signal) --
                                       // diagnostic toggle to test LED interference with NFC reads
bool ringCleared = false;              // tracks whether we've already blanked the ring while disabled

// ── Animation Presets ────────────────────────────────────────────────────────
const char* PRESET_NAMES[] = {"noise", "chase", "breathe", "wave", "sparkle", "rainbow", "solid", "gradient"};
const uint8_t PRESET_COUNT = 8;

const char* TRANSITION_NAMES[] = {"none", "crossfade", "fadeblack", "flashwhite"};
const uint8_t TRANSITION_COUNT = 4;

TransitionId transitionFromString(const char* s) {
  for (uint8_t i = 0; i < TRANSITION_COUNT; i++) {
    if (!strcmp(s, TRANSITION_NAMES[i])) return (TransitionId)i;
  }
  return T_NONE;
}
const char* transitionToString(TransitionId t) {
  if (t < TRANSITION_COUNT) return TRANSITION_NAMES[t];
  return "none";
}

PresetId presetFromString(const char* s) {
  for (uint8_t i = 0; i < PRESET_COUNT; i++) {
    if (!strcmp(s, PRESET_NAMES[i])) return (PresetId)i;
  }
  return P_UNKNOWN;
}

const char* presetToString(PresetId p) {
  if (p < PRESET_COUNT) return PRESET_NAMES[p];
  return "unknown";
}

// ── State Storage ────────────────────────────────────────────────────────────
LedState states[MAX_STATES];
uint8_t stateCount = 0;
volatile int8_t currentStateIdx = -1;  // volatile: read by nfcTask (core 0), written by loop (core 1)
int8_t pendingStateIdx = -1;
unsigned long stateStartMs = 0;
float noiseT = 0.0f;

// ── Hardware Objects ─────────────────────────────────────────────────────────
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);
CRGB leds[LED_COUNT];
Preferences prefs;
const char* PREFS_NS = "nfc_states";

// ── Runtime State ────────────────────────────────────────────────────────────
String serialBuffer = "";
String importBuffer = "";
bool importing = false;
volatile bool nfcAvailable = false;  // written by nfcTask (core 0), read by loop (core 1)
volatile bool nfcCardDetected = false;  // set when card first detected, cleared by loop
volatile bool nfcNewScan = false;    // set when scan complete, cleared by loop
char nfcPendingUid[24] = "";         // written before nfcNewScan=true; read after

// ── NDEF Guest Data (written by nfcTask before nfcNewScan=true) ──────────────
char nfcPendingBadgeId[24] = "";
char nfcPendingName[64] = "";        // "FirstName LastName"
char nfcPendingCompany[48] = "";
bool nfcPendingValid = false;        // true = valid NDEF guest data parsed
bool nfcReadComplete = false;        // true = NDEF read finished (vs failed early)
bool nfcResultPending = false;       // true = waiting for reading min time before triggering
bool nfcResultSuccess = false;       // true = trigger success, false = trigger failure

// ── Reset Card State ─────────────────────────────────────────────────────────
char resetUids[MAX_RESET_UIDS][MAX_UID_STR];  // enrolled reset-card UIDs
uint8_t resetUidCount = 0;
volatile bool nfcResetRequested = false;      // set by nfcTask (core 0), cleared by loop (core 1)
char nfcResetUid[MAX_UID_STR] = "";           // UID that triggered the reset (for logging)

// ── Transition Runtime ───────────────────────────────────────────────────────
bool inTransition = false;
unsigned long transitionStartMs = 0;
uint16_t transitionActiveMs = 300;
TransitionId activeTransition = T_NONE;
CRGB transitionFromLeds[LED_COUNT];
unsigned long lastTapMs = 0;         // only accessed by nfcTask
unsigned long lastHeartbeatMs = 0;
unsigned long lastAnimMs = 0;

// ── Default State Definitions ────────────────────────────────────────────────
void initDefaultStates() {
  stateCount = 0;

  // IDLE state
  LedState& idle = states[stateCount++];
  strcpy(idle.name, "idle");
  idle.preset = P_WAVE;
  idle.speed = 0.026f;
  idle.scale = 0.18f;
  idle.threshold = 1.0f;
  idle.gamma = 1.4f;
  idle.brightness = 210;
  idle.breaths = 0;
  idle.duration = 0;  // indefinite
  idle.returnTo[0] = '\0';
  idle.colorCount = 1;
  idle.colors[0] = {255, 255, 255, 0.0f};  // white
  idle.isDefault = true;
  idle.randomColor = false;
  idle.transition = T_FADE_BLACK;
  idle.transitionMs = 0;

  // SUCCESS state
  LedState& success = states[stateCount++];
  strcpy(success.name, "success");
  success.preset = P_BREATHE;
  success.speed = 0.046f;
  success.scale = 0.18f;
  success.threshold = 1.0f;
  success.gamma = 1.2f;
  success.brightness = 255;
  success.breaths = 2;
  success.duration = 1500;
  strcpy(success.returnTo, "active");
  success.colorCount = 1;
  success.colors[0] = {0, 255, 0, 0.0f};  // green
  success.isDefault = true;
  success.randomColor = false;
  success.transition = T_CROSSFADE;
  success.transitionMs = 0;

  // FAILURE state
  LedState& failure = states[stateCount++];
  strcpy(failure.name, "failure");
  failure.preset = P_BREATHE;
  failure.speed = 0.046f;
  failure.scale = 0.18f;
  failure.threshold = 1.0f;
  failure.gamma = 1.2f;
  failure.brightness = 255;
  failure.breaths = 2;
  failure.duration = 1500;
  strcpy(failure.returnTo, "idle");
  failure.colorCount = 1;
  failure.colors[0] = {255, 0, 0, 0.0f};  // red
  failure.isDefault = true;
  failure.randomColor = false;
  failure.transition = T_CROSSFADE;
  failure.transitionMs = 0;

  // ACTIVE state
  LedState& active = states[stateCount++];
  strcpy(active.name, "active");
  active.preset = P_NOISE;
  active.speed = 0.032f;
  active.scale = 0.18f;
  active.threshold = 1.0f;
  active.gamma = 1.4f;
  active.brightness = 210;
  active.breaths = 0;
  active.duration = 0;  // indefinite
  active.returnTo[0] = '\0';
  active.colorCount = 1;
  active.colors[0] = {0, 20, 255, 0.0f};  // blue
  active.isDefault = true;
  active.randomColor = false;
  active.transition = T_CROSSFADE;
  active.transitionMs = 0;

  // READING state (card detected, waiting for NDEF read)
  LedState& reading = states[stateCount++];
  strcpy(reading.name, "reading");
  reading.preset = P_CHASE;
  reading.speed = 0.12f;
  reading.scale = 0.18f;
  reading.threshold = 1.0f;
  reading.gamma = 1.4f;
  reading.brightness = 255;
  reading.breaths = 0;
  reading.duration = 3000;  // timeout if card removed early
  strcpy(reading.returnTo, "failure");
  reading.colorCount = 1;
  reading.colors[0] = {255, 200, 0, 0.0f};  // amber/yellow
  reading.isDefault = true;
  reading.randomColor = false;
  reading.transition = T_CROSSFADE;
  reading.transitionMs = 0;
}

// ── State Lookup ─────────────────────────────────────────────────────────────
int8_t findStateByName(const char* name) {
  for (uint8_t i = 0; i < stateCount; i++) {
    if (!strcmp(states[i].name, name)) return i;
  }
  return -1;
}

LedState* getStateByName(const char* name) {
  int8_t idx = findStateByName(name);
  return (idx >= 0) ? &states[idx] : nullptr;
}

LedState* getCurrentState() {
  return (currentStateIdx >= 0 && currentStateIdx < stateCount) ? &states[currentStateIdx] : nullptr;
}

// ── Reset Card Lookup ────────────────────────────────────────────────────────
bool isResetUid(const char* uid) {
  for (uint8_t i = 0; i < resetUidCount; i++) {
    if (!strcmp(resetUids[i], uid)) return true;
  }
  return false;
}

// ── Color Position Helpers ───────────────────────────────────────────────────
// Evenly space all color stops from 0.0 to 1.0.
void distributeColorPositions(LedState* s) {
  if (s->colorCount == 0) return;
  if (s->colorCount == 1) { s->colors[0].pos = 0.0f; return; }
  for (uint8_t i = 0; i < s->colorCount; i++) {
    s->colors[i].pos = (float)i / (s->colorCount - 1);
  }
}

// Insertion sort (max MAX_COLORS=8 elements) — keeps stops ordered by pos.
void sortColorsByPos(LedState* s) {
  for (uint8_t i = 1; i < s->colorCount; i++) {
    LedColor tmp = s->colors[i];
    int8_t j = i - 1;
    while (j >= 0 && s->colors[j].pos > tmp.pos) {
      s->colors[j + 1] = s->colors[j];
      j--;
    }
    s->colors[j + 1] = tmp;
  }
}

// ── Color Interpolation (Gradient) ───────────────────────────────────────────
// Colors must be sorted by pos (ascending). Wraps around the ring: the gap
// from the last stop back to the first stop blends across the 0/1 boundary.
CRGB interpolateGradient(const LedState& s, float position) {
  if (s.colorCount == 0) return CRGB::Black;
  if (s.colorCount == 1) return CRGB(s.colors[0].r, s.colors[0].g, s.colors[0].b);

  // Find first stop with pos strictly greater than position
  int8_t hi = -1;
  for (uint8_t i = 0; i < s.colorCount; i++) {
    if (s.colors[i].pos > position) { hi = i; break; }
  }

  // hi == -1: position is at or past last stop
  // hi == 0:  position is before first stop
  // Both cases wrap around the ring: blend last stop → first stop
  if (hi < 0 || hi == 0) {
    const LedColor& cL = s.colors[s.colorCount - 1];
    const LedColor& cF = s.colors[0];
    float span = (1.0f - cL.pos) + cF.pos;
    float t = 0.0f;
    if (span > 0.001f)
      t = (hi < 0) ? (position - cL.pos) / span
                   : (1.0f - cL.pos + position) / span;
    t = constrain(t, 0.0f, 1.0f);
    return CRGB(
      (uint8_t)(cL.r + (int16_t)(cF.r - cL.r) * t),
      (uint8_t)(cL.g + (int16_t)(cF.g - cL.g) * t),
      (uint8_t)(cL.b + (int16_t)(cF.b - cL.b) * t)
    );
  }

  const LedColor& c0 = s.colors[hi - 1];
  const LedColor& c1 = s.colors[hi];
  float span = c1.pos - c0.pos;
  float t = (span > 0.001f) ? (position - c0.pos) / span : 0.0f;
  t = constrain(t, 0.0f, 1.0f);
  return CRGB(
    (uint8_t)(c0.r + (int16_t)(c1.r - c0.r) * t),
    (uint8_t)(c0.g + (int16_t)(c1.g - c0.g) * t),
    (uint8_t)(c0.b + (int16_t)(c1.b - c0.b) * t)
  );
}

// ── Animation Rendering ──────────────────────────────────────────────────────
CRGB getColorForLed(int i, const LedState& s, unsigned long elapsedMs) {
  float angle = (float)i / LED_COUNT * TWO_PI;
  float position = (float)i / LED_COUNT;  // 0-1 around ring

  // Get base color from gradient
  CRGB baseColor = interpolateGradient(s, position);
  float tintR = baseColor.r / 255.0f;
  float tintG = baseColor.g / 255.0f;
  float tintB = baseColor.b / 255.0f;

  float val = 0;

  switch (s.preset) {
    case P_SOLID:
      val = 1.0f;
      break;

    case P_GRADIENT:
      // Pure gradient, no animation modulation
      return CRGB(baseColor.r * s.brightness / 255,
                  baseColor.g * s.brightness / 255,
                  baseColor.b * s.brightness / 255);

    case P_NOISE: {
      int16_t nx = (int16_t)(cos(angle) * s.scale * 2550);
      int16_t ny = (int16_t)(sin(angle) * s.scale * 2550);
      uint32_t tScaled = (uint32_t)(noiseT * 1000.0f);
      uint8_t n = inoise8(nx, ny, tScaled);
      float noiseVal = constrain((float)n / 255.0f, 0.0f, 1.0f);
      if (s.randomColor) {
        // Use noise value as gradient position so colors flow with the texture
        CRGB c = interpolateGradient(s, noiseVal);
        float bri = pow(noiseVal, s.gamma) * s.brightness / 255.0f;
        return CRGB((uint8_t)(c.r * bri), (uint8_t)(c.g * bri), (uint8_t)(c.b * bri));
      }
      val = pow(noiseVal, s.gamma);
      break;
    }

    case P_BREATHE: {
      float breath;
      if (s.breaths > 0 && s.duration > 0) {
        float period = (float)s.duration / s.breaths;
        float phase = (float)elapsedMs / period * TWO_PI - HALF_PI;
        breath = (sin(phase) + 1.0f) / 2.0f;
      } else {
        breath = (sin(noiseT * 2.0f) + 1.0f) / 2.0f;
      }
      val = pow(breath, s.gamma);
      break;
    }

    case P_CHASE: {
      float pos = fmod(noiseT * 2.0f, TWO_PI);
      float dist = abs(angle - pos);
      if (dist > PI) dist = TWO_PI - dist;
      float tailLen = 0.8f;
      if (dist < tailLen) val = 1.0f - dist / tailLen;
      break;
    }

    case P_WAVE: {
      float wave = (sin(angle * 3.0f - noiseT * 3.0f) + 1.0f) / 2.0f;
      val = pow(wave, s.gamma);
      break;
    }

    case P_SPARKLE: {
      float sparkle = (random8() > 235) ? 1.0f : (float)inoise8(i * 128, (uint32_t)(noiseT * 300)) / 255.0f * 0.3f + 0.1f;
      float sv = pow(constrain(sparkle, 0.0f, 1.0f), s.gamma);
      if (s.randomColor) {
        // Each LED gets a slowly-drifting noise position in the gradient
        float colorPos = (float)inoise8(i * 73 + 19, (uint32_t)(noiseT * 50)) / 255.0f;
        CRGB c = interpolateGradient(s, colorPos);
        float bri = sv * s.brightness / 255.0f;
        return CRGB((uint8_t)(c.r * bri), (uint8_t)(c.g * bri), (uint8_t)(c.b * bri));
      }
      val = sv;
      break;
    }

    case P_RAINBOW: {
      uint8_t hue = (uint8_t)(position * 255) + (uint8_t)(noiseT * 25.0f);
      return CHSV(hue, 230, s.brightness);
    }

    default:
      val = 1.0f;
      break;
  }

  uint8_t r = (uint8_t)(val * s.brightness * tintR);
  uint8_t g = (uint8_t)(val * s.brightness * tintG);
  uint8_t b = (uint8_t)(val * s.brightness * tintB);
  return CRGB(r, g, b);
}

// ── State Transitions ────────────────────────────────────────────────────────
void triggerState(const char* name) {
  int8_t idx = findStateByName(name);
  if (idx >= 0) {
    pendingStateIdx = idx;
  } else {
    Serial.print("[err] unknown state: ");
    Serial.println(name);
  }
}

void triggerStateByIndex(int8_t idx) {
  if (idx >= 0 && idx < stateCount) {
    pendingStateIdx = idx;
  }
}

void executeTransition(unsigned long now) {
  if (pendingStateIdx >= 0 && pendingStateIdx != currentStateIdx) {
    LedState& incoming = states[pendingStateIdx];
    TransitionId tid = incoming.transition;

    if (tid != T_NONE) {
      memcpy(transitionFromLeds, leds, sizeof(CRGB) * LED_COUNT);
      inTransition = true;
      transitionStartMs = now;
      // Default durations per type when transitionMs is 0
      if (incoming.transitionMs > 0) {
        transitionActiveMs = incoming.transitionMs;
      } else {
        transitionActiveMs = (tid == T_FADE_BLACK) ? 500 : 300;
      }
      activeTransition = tid;
    } else {
      inTransition = false;
    }

    currentStateIdx = pendingStateIdx;
    pendingStateIdx = -1;
    stateStartMs = now;
    Serial.print("[state] ");
    Serial.println(states[currentStateIdx].name);
  }
  // Always clear the request, even when it was a redundant trigger of the
  // state we're already in. Otherwise pendingStateIdx stays pinned >= 0 and
  // the duration-based auto-advance (which requires pendingStateIdx < 0) never
  // fires -- the root cause of the ring getting stuck in "reading" when a
  // failed/flaky card re-triggers "reading" while it's already active.
  pendingStateIdx = -1;
}

// ── NVS Persistence ──────────────────────────────────────────────────────────
void saveStatesToNvs() {
  prefs.begin(PREFS_NS, false);
  prefs.clear();

  // Store state count and global settings
  prefs.putUChar("count", stateCount);
  prefs.putUShort("readingMs", readingMinMs);
  prefs.putBool("acceptTaps", acceptTaps);
  prefs.putBool("ledEnabled", ledEnabled);

  // Reset card UIDs
  prefs.putUChar("rstc", resetUidCount);
  for (uint8_t i = 0; i < resetUidCount; i++) {
    char rk[8];
    snprintf(rk, sizeof(rk), "rst%d", i);
    prefs.putString(rk, resetUids[i]);
  }

  // Store each state as JSON
  for (uint8_t i = 0; i < stateCount; i++) {
    StaticJsonDocument<896> doc;
    LedState& s = states[i];

    doc["name"] = s.name;
    doc["preset"] = presetToString(s.preset);
    doc["speed"] = s.speed;
    doc["scale"] = s.scale;
    doc["threshold"] = s.threshold;
    doc["gamma"] = s.gamma;
    doc["brightness"] = s.brightness;
    doc["breaths"] = s.breaths;
    doc["duration"] = s.duration;
    doc["returnTo"] = s.returnTo;
    doc["isDefault"] = s.isDefault;
    doc["randomColor"] = s.randomColor;
    doc["transition"] = transitionToString(s.transition);
    doc["transitionMs"] = s.transitionMs;

    JsonArray colors = doc.createNestedArray("colors");
    for (uint8_t c = 0; c < s.colorCount; c++) {
      JsonArray color = colors.createNestedArray();
      color.add(s.colors[c].r);
      color.add(s.colors[c].g);
      color.add(s.colors[c].b);
      color.add(s.colors[c].pos);
    }

    String json;
    serializeJson(doc, json);

    char key[8];
    snprintf(key, sizeof(key), "s%d", i);
    prefs.putString(key, json.c_str());
  }

  prefs.end();
}

void loadStatesFromNvs() {
  prefs.begin(PREFS_NS, true);

  // Load global settings
  readingMinMs = prefs.getUShort("readingMs", READING_MIN_DEFAULT);
  acceptTaps = prefs.getBool("acceptTaps", true);
  ledEnabled = prefs.getBool("ledEnabled", true);

  // Reset card UIDs (loaded before the count==0 early-return below so they
  // survive even on a device that has no custom states saved).
  resetUidCount = prefs.getUChar("rstc", 0);
  if (resetUidCount > MAX_RESET_UIDS) resetUidCount = MAX_RESET_UIDS;
  for (uint8_t i = 0; i < resetUidCount; i++) {
    char rk[8];
    snprintf(rk, sizeof(rk), "rst%d", i);
    String v = prefs.getString(rk, "");
    strncpy(resetUids[i], v.c_str(), MAX_UID_STR - 1);
    resetUids[i][MAX_UID_STR - 1] = '\0';
  }

  uint8_t count = prefs.getUChar("count", 0);
  if (count == 0) {
    prefs.end();
    initDefaultStates();
    return;
  }

  stateCount = 0;

  for (uint8_t i = 0; i < count && stateCount < MAX_STATES; i++) {
    char key[8];
    snprintf(key, sizeof(key), "s%d", i);
    String json = prefs.getString(key, "");

    if (json.length() == 0) continue;

    StaticJsonDocument<896> doc;
    if (deserializeJson(doc, json) != DeserializationError::Ok) continue;

    LedState& s = states[stateCount];

    strncpy(s.name, doc["name"] | "unnamed", MAX_STATE_NAME);
    s.name[MAX_STATE_NAME] = '\0';
    s.preset = presetFromString(doc["preset"] | "solid");
    s.speed = doc["speed"] | 0.06f;
    s.scale = doc["scale"] | 0.18f;
    s.threshold = doc["threshold"] | 1.0f;
    s.gamma = doc["gamma"] | 1.4f;
    s.brightness = doc["brightness"] | 210;
    s.breaths = doc["breaths"] | 0;
    s.duration = doc["duration"] | 0;
    strncpy(s.returnTo, doc["returnTo"] | "", MAX_STATE_NAME);
    s.returnTo[MAX_STATE_NAME] = '\0';
    s.isDefault = doc["isDefault"] | false;
    s.randomColor = doc["randomColor"] | false;
    s.transition = transitionFromString(doc["transition"] | "none");
    s.transitionMs = doc["transitionMs"] | 0;

    JsonArray colors = doc["colors"];
    s.colorCount = 0;
    bool hasPosData = false;
    for (JsonArray color : colors) {
      if (s.colorCount >= MAX_COLORS) break;
      s.colors[s.colorCount].r = color[0] | 0;
      s.colors[s.colorCount].g = color[1] | 0;
      s.colors[s.colorCount].b = color[2] | 0;
      s.colors[s.colorCount].pos = color[3] | -1.0f;
      if (s.colors[s.colorCount].pos >= 0.0f) hasPosData = true;
      s.colorCount++;
    }
    if (s.colorCount == 0) {
      s.colors[0] = {255, 255, 255, 0.0f};
      s.colorCount = 1;
    } else if (!hasPosData) {
      // Old NVS data without pos field — distribute evenly
      distributeColorPositions(&s);
    }

    stateCount++;
  }

  prefs.end();

  // Ensure we have at least the default states
  if (findStateByName("idle") < 0 || findStateByName("success") < 0 ||
      findStateByName("failure") < 0 || findStateByName("reading") < 0 ||
      findStateByName("active") < 0) {
    initDefaultStates();
  }
}

void resetToDefaults() {
  prefs.begin(PREFS_NS, false);
  prefs.clear();
  prefs.end();
  initDefaultStates();
}

// ── JSON Dump ────────────────────────────────────────────────────────────────
void dumpStateJson(const LedState& s) {
  StaticJsonDocument<896> doc;

  doc["name"] = s.name;
  doc["preset"] = presetToString(s.preset);
  doc["speed"] = s.speed;
  doc["scale"] = s.scale;
  doc["threshold"] = s.threshold;
  doc["gamma"] = s.gamma;
  doc["brightness"] = s.brightness;
  doc["breaths"] = s.breaths;
  doc["duration"] = s.duration;
  doc["returnTo"] = s.returnTo;
  doc["isDefault"] = s.isDefault;
  doc["randomColor"] = s.randomColor;
  doc["transition"] = transitionToString(s.transition);
  doc["transitionMs"] = s.transitionMs;

  JsonArray colors = doc.createNestedArray("colors");
  for (uint8_t c = 0; c < s.colorCount; c++) {
    JsonArray color = colors.createNestedArray();
    color.add(s.colors[c].r);
    color.add(s.colors[c].g);
    color.add(s.colors[c].b);
    color.add(s.colors[c].pos);
  }

  Serial.print("[cfg] ");
  serializeJson(doc, Serial);
  Serial.println();
}

void dumpAllStates() {
  Serial.print("[cfg] {\"stateCount\":");
  Serial.print(stateCount);
  Serial.print(",\"currentState\":\"");
  LedState* cs = getCurrentState();
  Serial.print(cs ? cs->name : "");
  Serial.print("\",\"nfcAvailable\":");
  Serial.print(nfcAvailable ? "true" : "false");
  Serial.print(",\"readingMinMs\":");
  Serial.print(readingMinMs);
  Serial.print(",\"acceptTaps\":");
  Serial.print(acceptTaps ? "true" : "false");
  Serial.print(",\"ledEnabled\":");
  Serial.print(ledEnabled ? "true" : "false");
  Serial.println("}");

  for (uint8_t i = 0; i < stateCount; i++) {
    dumpStateJson(states[i]);
  }

  Serial.println("[cfg] END");
}

// ── Serial Command Processing ────────────────────────────────────────────────
bool applySet(const String& line) {
  // SET <state> <param> <value...>
  int p1 = line.indexOf(' ');
  int p2 = line.indexOf(' ', p1 + 1);
  int p3 = line.indexOf(' ', p2 + 1);

  if (p1 < 0 || p2 < 0) {
    Serial.println("[err] SET requires: state param value");
    return false;
  }

  String stateName = line.substring(p1 + 1, p2);
  String param = (p3 < 0) ? line.substring(p2 + 1) : line.substring(p2 + 1, p3);
  String value = (p3 < 0) ? "" : line.substring(p3 + 1);

  LedState* s = getStateByName(stateName.c_str());
  if (!s) {
    Serial.print("[err] unknown state: ");
    Serial.println(stateName);
    return false;
  }

  if (param == "preset") {
    PresetId pid = presetFromString(value.c_str());
    if (pid == P_UNKNOWN) {
      Serial.print("[err] unknown preset: ");
      Serial.println(value);
      return false;
    }
    s->preset = pid;
  }
  else if (param == "speed") s->speed = value.toFloat();
  else if (param == "scale") s->scale = value.toFloat();
  else if (param == "threshold") s->threshold = value.toFloat();
  else if (param == "gamma") s->gamma = value.toFloat();
  else if (param == "brightness") s->brightness = constrain(value.toInt(), 0, 255);
  else if (param == "breaths") s->breaths = constrain(value.toInt(), 0, 255);
  else if (param == "duration") s->duration = constrain(value.toInt(), 0, 65535);
  else if (param == "returnTo") {
    strncpy(s->returnTo, value.c_str(), MAX_STATE_NAME);
    s->returnTo[MAX_STATE_NAME] = '\0';
  }
  else if (param == "randomColor") s->randomColor = (value.toInt() != 0);
  else if (param == "transition") s->transition = transitionFromString(value.c_str());
  else if (param == "transitionms") s->transitionMs = constrain(value.toInt(), 0, 65535);
  else {
    Serial.print("[err] unknown param: ");
    Serial.println(param);
    return false;
  }

  Serial.print("[ok] set ");
  Serial.print(stateName);
  Serial.print(" ");
  Serial.print(param);
  Serial.print(" ");
  Serial.println(value);
  return true;
}

bool addState(const String& name) {
  if (stateCount >= MAX_STATES) {
    Serial.println("[err] max states reached");
    return false;
  }

  if (findStateByName(name.c_str()) >= 0) {
    Serial.print("[err] state exists: ");
    Serial.println(name);
    return false;
  }

  if (name.length() > MAX_STATE_NAME) {
    Serial.println("[err] name too long");
    return false;
  }

  LedState& s = states[stateCount++];
  strncpy(s.name, name.c_str(), MAX_STATE_NAME);
  s.name[MAX_STATE_NAME] = '\0';
  s.preset = P_SOLID;
  s.speed = 0.06f;
  s.scale = 0.18f;
  s.threshold = 1.0f;
  s.gamma = 1.4f;
  s.brightness = 200;
  s.breaths = 0;
  s.duration = 0;
  s.returnTo[0] = '\0';
  s.colorCount = 1;
  s.colors[0] = {255, 255, 255, 0.0f};
  s.isDefault = false;
  s.randomColor = false;
  s.transition = T_NONE;
  s.transitionMs = 0;

  Serial.print("[ok] added state: ");
  Serial.println(name);
  return true;
}

bool deleteState(const String& name) {
  int8_t idx = findStateByName(name.c_str());
  if (idx < 0) {
    Serial.print("[err] state not found: ");
    Serial.println(name);
    return false;
  }

  if (states[idx].isDefault) {
    Serial.print("[err] cannot delete default state: ");
    Serial.println(name);
    return false;
  }

  // Shift remaining states down
  for (uint8_t i = idx; i < stateCount - 1; i++) {
    states[i] = states[i + 1];
  }
  stateCount--;

  // Adjust current state index if needed
  if (currentStateIdx == idx) {
    triggerState("idle");
  } else if (currentStateIdx > idx) {
    currentStateIdx--;
  }

  Serial.print("[ok] deleted state: ");
  Serial.println(name);
  return true;
}

bool addColor(const String& line) {
  // ADDCOLOR <state> <r> <g> <b> [<pos>]
  // Without pos: all positions are evenly redistributed 0.0–1.0.
  // With pos:    color inserted at that position and colors re-sorted.
  int p1 = line.indexOf(' ');
  int p2 = line.indexOf(' ', p1 + 1);
  int p3 = line.indexOf(' ', p2 + 1);
  int p4 = line.indexOf(' ', p3 + 1);
  int p5 = (p4 >= 0) ? line.indexOf(' ', p4 + 1) : -1;  // optional pos

  if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0) {
    Serial.println("[err] ADDCOLOR requires: state r g b [pos]");
    return false;
  }

  String stateName = line.substring(p1 + 1, p2);
  uint8_t r = constrain(line.substring(p2 + 1, p3).toInt(), 0, 255);
  uint8_t g = constrain(line.substring(p3 + 1, p4).toInt(), 0, 255);

  bool hasPos = (p5 >= 0);
  uint8_t b;
  float pos;
  if (hasPos) {
    b = constrain(line.substring(p4 + 1, p5).toInt(), 0, 255);
    pos = constrain(line.substring(p5 + 1).toFloat(), 0.0f, 1.0f);
  } else {
    b = constrain(line.substring(p4 + 1).toInt(), 0, 255);
    pos = 0.0f;
  }

  LedState* s = getStateByName(stateName.c_str());
  if (!s) {
    Serial.print("[err] unknown state: ");
    Serial.println(stateName);
    return false;
  }

  if (s->colorCount >= MAX_COLORS) {
    Serial.println("[err] max colors reached");
    return false;
  }

  s->colors[s->colorCount++] = {r, g, b, pos};

  if (hasPos) {
    sortColorsByPos(s);
  } else {
    distributeColorPositions(s);
  }

  Serial.print("[ok] added color to ");
  Serial.print(stateName);
  Serial.print(": r="); Serial.print(r);
  Serial.print(" g="); Serial.print(g);
  Serial.print(" b="); Serial.print(b);
  if (hasPos) { Serial.print(" pos="); Serial.print(pos); }
  Serial.println();
  return true;
}

bool setColor(const String& line) {
  // SETCOLOR <state> <index> <r> <g> <b> [<pos>]
  int p1 = line.indexOf(' ');
  int p2 = line.indexOf(' ', p1 + 1);
  int p3 = line.indexOf(' ', p2 + 1);
  int p4 = line.indexOf(' ', p3 + 1);
  int p5 = line.indexOf(' ', p4 + 1);
  int p6 = (p5 >= 0) ? line.indexOf(' ', p5 + 1) : -1;  // optional pos

  if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0 || p5 < 0) {
    Serial.println("[err] SETCOLOR requires: state index r g b [pos]");
    return false;
  }

  String stateName = line.substring(p1 + 1, p2);
  uint8_t idx = constrain(line.substring(p2 + 1, p3).toInt(), 0, MAX_COLORS - 1);
  uint8_t r = constrain(line.substring(p3 + 1, p4).toInt(), 0, 255);
  uint8_t g = constrain(line.substring(p4 + 1, p5).toInt(), 0, 255);

  bool hasPos = (p6 >= 0);
  uint8_t b;
  float pos;
  if (hasPos) {
    b = constrain(line.substring(p5 + 1, p6).toInt(), 0, 255);
    pos = constrain(line.substring(p6 + 1).toFloat(), 0.0f, 1.0f);
  } else {
    b = constrain(line.substring(p5 + 1).toInt(), 0, 255);
    pos = 0.0f;
  }

  LedState* s = getStateByName(stateName.c_str());
  if (!s) {
    Serial.print("[err] unknown state: ");
    Serial.println(stateName);
    return false;
  }

  if (idx >= s->colorCount) {
    Serial.println("[err] color index out of range");
    return false;
  }

  float existingPos = s->colors[idx].pos;
  s->colors[idx] = {r, g, b, hasPos ? pos : existingPos};
  if (hasPos) sortColorsByPos(s);

  Serial.print("[ok] set color ");
  Serial.print(idx);
  Serial.print(" on ");
  Serial.println(stateName);
  return true;
}

bool deleteColor(const String& line) {
  // DELCOLOR <state> <index>
  int p1 = line.indexOf(' ');
  int p2 = line.indexOf(' ', p1 + 1);

  if (p1 < 0 || p2 < 0) {
    Serial.println("[err] DELCOLOR requires: state index");
    return false;
  }

  String stateName = line.substring(p1 + 1, p2);
  uint8_t idx = constrain(line.substring(p2 + 1).toInt(), 0, MAX_COLORS - 1);

  LedState* s = getStateByName(stateName.c_str());
  if (!s) {
    Serial.print("[err] unknown state: ");
    Serial.println(stateName);
    return false;
  }

  if (s->colorCount <= 1) {
    Serial.println("[err] cannot remove last color");
    return false;
  }

  if (idx >= s->colorCount) {
    Serial.println("[err] color index out of range");
    return false;
  }

  // Shift remaining colors down
  for (uint8_t i = idx; i < s->colorCount - 1; i++) {
    s->colors[i] = s->colors[i + 1];
  }
  s->colorCount--;

  Serial.print("[ok] deleted color ");
  Serial.print(idx);
  Serial.print(" from ");
  Serial.println(stateName);
  return true;
}

bool setPos(const String& line) {
  // SETPOS <state> <index> <pos>
  int p1 = line.indexOf(' ');
  int p2 = line.indexOf(' ', p1 + 1);
  int p3 = line.indexOf(' ', p2 + 1);

  if (p1 < 0 || p2 < 0 || p3 < 0) {
    Serial.println("[err] SETPOS requires: state index pos");
    return false;
  }

  String stateName = line.substring(p1 + 1, p2);
  uint8_t idx = constrain(line.substring(p2 + 1, p3).toInt(), 0, MAX_COLORS - 1);
  float pos = constrain(line.substring(p3 + 1).toFloat(), 0.0f, 1.0f);

  LedState* s = getStateByName(stateName.c_str());
  if (!s) {
    Serial.print("[err] unknown state: ");
    Serial.println(stateName);
    return false;
  }

  if (idx >= s->colorCount) {
    Serial.println("[err] color index out of range");
    return false;
  }

  s->colors[idx].pos = pos;
  sortColorsByPos(s);

  Serial.print("[ok] set pos on ");
  Serial.print(stateName);
  Serial.print(" color ");
  Serial.print(idx);
  Serial.print(": ");
  Serial.println(pos);
  return true;
}

// ── Reset Card Management ────────────────────────────────────────────────────
bool addResetUid(const String& uid) {
  String u = uid;
  u.trim();
  if (u.length() == 0) {
    Serial.println("[err] empty uid (tap a card first, or use ADDRESET <uid>)");
    return false;
  }
  if (u.length() >= MAX_UID_STR) {
    Serial.println("[err] uid too long");
    return false;
  }
  if (isResetUid(u.c_str())) {
    Serial.print("[err] already a reset card: ");
    Serial.println(u);
    return false;
  }
  if (resetUidCount >= MAX_RESET_UIDS) {
    Serial.println("[err] max reset cards reached");
    return false;
  }
  strncpy(resetUids[resetUidCount], u.c_str(), MAX_UID_STR - 1);
  resetUids[resetUidCount][MAX_UID_STR - 1] = '\0';
  resetUidCount++;
  Serial.print("[ok] added reset card: ");
  Serial.println(u);
  Serial.println("[sys]  send SAVE to persist");
  return true;
}

bool delResetUid(const String& uid) {
  String u = uid;
  u.trim();
  int8_t idx = -1;
  for (uint8_t i = 0; i < resetUidCount; i++) {
    if (!strcmp(resetUids[i], u.c_str())) { idx = i; break; }
  }
  if (idx < 0) {
    Serial.print("[err] not a reset card: ");
    Serial.println(u);
    return false;
  }
  for (uint8_t i = idx; i < resetUidCount - 1; i++) {
    strcpy(resetUids[i], resetUids[i + 1]);
  }
  resetUidCount--;
  Serial.print("[ok] deleted reset card: ");
  Serial.println(u);
  Serial.println("[sys]  send SAVE to persist");
  return true;
}

void listResetUids() {
  Serial.print("[reset] ");
  for (uint8_t i = 0; i < resetUidCount; i++) {
    if (i > 0) Serial.print(",");
    Serial.print(resetUids[i]);
  }
  Serial.println();
}

void processImportLine(const String& line) {
  if (line == "ENDIMPORT") {
    // Parse accumulated JSON
    StaticJsonDocument<4096> doc;
    if (deserializeJson(doc, importBuffer) != DeserializationError::Ok) {
      Serial.println("[err] import parse failed");
      importing = false;
      importBuffer = "";
      return;
    }

    // Clear existing non-default states
    uint8_t newCount = 0;
    for (uint8_t i = 0; i < stateCount; i++) {
      if (states[i].isDefault) {
        if (newCount != i) states[newCount] = states[i];
        newCount++;
      }
    }
    stateCount = newCount;

    // Import states from JSON array
    JsonArray statesArr = doc["states"];
    for (JsonObject stateObj : statesArr) {
      const char* name = stateObj["name"] | "";

      // Find or create state
      LedState* s = getStateByName(name);
      if (!s) {
        if (stateCount >= MAX_STATES) continue;
        s = &states[stateCount++];
        strncpy(s->name, name, MAX_STATE_NAME);
        s->name[MAX_STATE_NAME] = '\0';
        s->isDefault = false;
      }

      s->preset = presetFromString(stateObj["preset"] | "solid");
      s->speed = stateObj["speed"] | 0.06f;
      s->scale = stateObj["scale"] | 0.18f;
      s->threshold = stateObj["threshold"] | 1.0f;
      s->gamma = stateObj["gamma"] | 1.4f;
      s->brightness = stateObj["brightness"] | 210;
      s->breaths = stateObj["breaths"] | 0;
      s->duration = stateObj["duration"] | 0;
      strncpy(s->returnTo, stateObj["returnTo"] | "", MAX_STATE_NAME);
      s->returnTo[MAX_STATE_NAME] = '\0';
      s->randomColor = stateObj["randomColor"] | false;
      s->transition = transitionFromString(stateObj["transition"] | "none");
      s->transitionMs = stateObj["transitionMs"] | 0;

      JsonArray colors = stateObj["colors"];
      s->colorCount = 0;
      bool hasPosData = false;
      for (JsonArray color : colors) {
        if (s->colorCount >= MAX_COLORS) break;
        s->colors[s->colorCount].r = color[0] | 0;
        s->colors[s->colorCount].g = color[1] | 0;
        s->colors[s->colorCount].b = color[2] | 0;
        s->colors[s->colorCount].pos = color[3] | -1.0f;
        if (s->colors[s->colorCount].pos >= 0.0f) hasPosData = true;
        s->colorCount++;
      }
      if (s->colorCount == 0) {
        s->colors[0] = {255, 255, 255, 0.0f};
        s->colorCount = 1;
      } else if (!hasPosData) {
        distributeColorPositions(s);
      }
    }

    Serial.println("[ok] import complete");
    importing = false;
    importBuffer = "";
    return;
  }

  importBuffer += line;
}

void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialBuffer.trim();
      if (serialBuffer.length() == 0) {
        serialBuffer = "";
        continue;
      }

      if (importing) {
        processImportLine(serialBuffer);
        serialBuffer = "";
        continue;
      }

      // Command dispatch
      if (serialBuffer == "OK") {
        triggerState("success");
        Serial.println("[led] Success triggered");
      }
      else if (serialBuffer == "FAIL") {
        triggerState("failure");
        Serial.println("[led] Failure triggered");
      }
      else if (serialBuffer.startsWith("TRIGGER ")) {
        String name = serialBuffer.substring(8);
        triggerState(name.c_str());
      }
      else if (serialBuffer == "STATUS") {
        emitStatusBanner();
      }
      else if (serialBuffer == "DUMP") {
        dumpAllStates();
      }
      else if (serialBuffer == "SAVE") {
        saveStatesToNvs();
        Serial.println("[ok] saved");
      }
      else if (serialBuffer == "RESET") {
        resetToDefaults();
        Serial.println("[ok] reset to defaults");
      }
      else if (serialBuffer.startsWith("SET ")) {
        applySet(serialBuffer);
      }
      else if (serialBuffer.startsWith("ADDSTATE ")) {
        addState(serialBuffer.substring(9));
      }
      else if (serialBuffer.startsWith("DELSTATE ")) {
        deleteState(serialBuffer.substring(9));
      }
      else if (serialBuffer.startsWith("ADDCOLOR ")) {
        addColor(serialBuffer);
      }
      else if (serialBuffer.startsWith("SETCOLOR ")) {
        setColor(serialBuffer);
      }
      else if (serialBuffer.startsWith("DELCOLOR ")) {
        deleteColor(serialBuffer);
      }
      else if (serialBuffer.startsWith("SETPOS ")) {
        setPos(serialBuffer);
      }
      else if (serialBuffer == "IMPORT") {
        importing = true;
        importBuffer = "";
        Serial.println("[ok] ready for import (send JSON then ENDIMPORT)");
      }
      else if (serialBuffer.startsWith("SETDELAY ")) {
        int val = serialBuffer.substring(9).toInt();
        if (val >= 0 && val <= 10000) {
          readingMinMs = val;
          Serial.print("[ok] readingMinMs=");
          Serial.println(readingMinMs);
        } else {
          Serial.println("[err] delay must be 0-10000");
        }
      }
      else if (serialBuffer.startsWith("SETTAPS ")) {
        int val = serialBuffer.substring(8).toInt();
        acceptTaps = (val != 0);
        Serial.print("[cfg] acceptTaps=");
        Serial.println(acceptTaps ? "1" : "0");
      }
      else if (serialBuffer.startsWith("SETLED ")) {
        int val = serialBuffer.substring(7).toInt();
        ledEnabled = (val != 0);
        Serial.print("[cfg] ledEnabled=");
        Serial.println(ledEnabled ? "1" : "0");
      }
      else if (serialBuffer == "LIST") {
        Serial.print("[list] ");
        for (uint8_t i = 0; i < stateCount; i++) {
          if (i > 0) Serial.print(",");
          Serial.print(states[i].name);
        }
        Serial.println();
      }
      else if (serialBuffer.startsWith("ADDRESET ")) {
        addResetUid(serialBuffer.substring(9));
      }
      else if (serialBuffer == "ADDRESET") {
        // No arg: enroll the most recently scanned UID (tap card, then ADDRESET).
        addResetUid(String(nfcPendingUid));
      }
      else if (serialBuffer.startsWith("DELRESET ")) {
        delResetUid(serialBuffer.substring(9));
      }
      else if (serialBuffer == "LISTRESET") {
        listResetUids();
      }
      else {
        Serial.print("[err] unknown command: ");
        Serial.println(serialBuffer);
      }

      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }
}

// ── Status Banner ────────────────────────────────────────────────────────────
void emitStatusBanner() {
  Serial.println("[boot] NFC Activation v2.1 — Dynamic States + NFC Recovery");
  if (nfcAvailable) {
    Serial.println("[nfc]  PN532 present");
  } else {
    Serial.println("[warn] PN532 not detected — LED-only mode");
  }
  Serial.print("[sys]  ");
  Serial.print(stateCount);
  Serial.println(" states loaded");
  Serial.print("[sys]  ");
  Serial.print(resetUidCount);
  Serial.println(" reset cards loaded");
  Serial.println("[sys]  Setup complete.");
}

// ── UID Conversion ───────────────────────────────────────────────────────────
String uidToWaveId(uint8_t *uid, uint8_t uidLen) {
  uint64_t v = 0;
  for (uint8_t i = 0; i < uidLen; i++) v = (v << 8) | uid[i];
  if (v == 0) return String("0");
  char buf[21];
  uint8_t i = sizeof(buf) - 1;
  buf[i] = 0;
  while (v && i > 0) { buf[--i] = '0' + (v % 10); v /= 10; }
  return String(&buf[i]);
}

// ── Badge Format Validation ──────────────────────────────────────────────────
// Our badges carry an NDEF text record beginning with a 20-char badge ID:
// 16 decimal digits followed by a 4-char alphanumeric activation code, then
// caret-delimited guest fields. This check rejects foreign cards (hotel keys,
// transit passes, blank/other NTAGs) that happen to hold an NDEF text record,
// so they never flash success or get forwarded as a guest scan.
bool isOurBadgeFormat(const char* badgeId) {
  if (strlen(badgeId) != 20) return false;
  for (uint8_t i = 0; i < 16; i++) {
    if (badgeId[i] < '0' || badgeId[i] > '9') return false;   // 16-digit numeric ID
  }
  for (uint8_t i = 16; i < 20; i++) {                          // 4-char alphanumeric code
    char c = badgeId[i];
    bool alnum = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
    if (!alnum) return false;
  }
  return true;
}

// ── NDEF Reading ─────────────────────────────────────────────────────────────
// Read NTAG pages and extract NDEF text record payload.
// Writes directly to nfcPending* globals. Returns true if valid guest data was parsed.
bool readNdefGuestData() {
  nfcPendingBadgeId[0] = '\0';
  nfcPendingName[0] = '\0';
  nfcPendingCompany[0] = '\0';
  nfcPendingValid = false;
  nfcReadComplete = false;

  // Read pages 4-19 (64 bytes) which should contain the NDEF message
  // ntag2xx_ReadPage reads 4 bytes (1 page) at a time
  uint8_t buffer[64];
  uint8_t bufLen = 0;

  for (uint8_t page = 4; page < 20 && bufLen < 64; page++) {
    uint8_t pageData[4];
    // Retry up to 3 times per page
    bool success = false;
    for (uint8_t retry = 0; retry < 3 && !success; retry++) {
      success = nfc.ntag2xx_ReadPage(page, pageData);
      if (!success) delay(5);
    }
    if (!success) {
      return false;
    }
    memcpy(buffer + bufLen, pageData, 4);
    bufLen += 4;
  }

  // Pages read successfully - mark read as complete
  nfcReadComplete = true;

  // Find NDEF message TLV (type 0x03)
  uint8_t* ndefStart = nullptr;
  uint8_t ndefLen = 0;
  for (uint8_t i = 0; i < bufLen - 2; i++) {
    if (buffer[i] == 0x03) {  // NDEF Message TLV
      ndefLen = buffer[i + 1];
      if (ndefLen == 0xFF) {
        // 3-byte length format
        ndefLen = (buffer[i + 2] << 8) | buffer[i + 3];
        ndefStart = &buffer[i + 4];
      } else {
        ndefStart = &buffer[i + 2];
      }
      break;
    }
  }

  if (!ndefStart || ndefLen < 10) {
    return false;
  }

  // Parse NDEF record header
  // Byte 0: flags (MB, ME, CF, SR, IL, TNF)
  // Byte 1: type length
  // Byte 2: payload length (if SR=1) or bytes 2-5 for long
  // Then: type, ID (if IL=1), payload
  uint8_t flags = ndefStart[0];
  uint8_t typeLen = ndefStart[1];
  bool sr = (flags & 0x10);  // Short Record flag
  uint8_t payloadLen = sr ? ndefStart[2] : ndefStart[5];
  uint8_t headerLen = sr ? (3 + typeLen) : (6 + typeLen);

  // Get payload - may or may not have language code prefix depending on record type
  uint8_t* payload = ndefStart + headerLen;
  char* text;
  uint8_t textLen;

  // Check if payload starts with a digit (our badge ID format) - if so, no lang prefix
  if (payload[0] >= '0' && payload[0] <= '9') {
    text = (char*)payload;
    textLen = payloadLen;
  } else {
    // Standard Text record: first byte is lang code length (lower 6 bits)
    uint8_t langLen = payload[0] & 0x3F;
    text = (char*)(payload + 1 + langLen);
    textLen = payloadLen - 1 - langLen;
  }

  // Null-terminate for parsing
  char textBuf[128];
  if (textLen >= sizeof(textBuf)) textLen = sizeof(textBuf) - 1;
  memcpy(textBuf, text, textLen);
  textBuf[textLen] = '\0';

  // Parse format: {badgeId}{4-char-code}^{firstName}^{lastName}^{company}^
  // Badge ID is first 20 chars (16 digits + 4-char code), then caret-delimited fields
  if (textLen < 21) {  // Minimum: 20 + at least one ^
    return false;
  }

  // Extract badge ID (first 20 chars: 16-digit ID + 4-char code)
  strncpy(nfcPendingBadgeId, textBuf, 20);
  nfcPendingBadgeId[20] = '\0';

  // Find fields after the code (start at position 20, after badge+code)
  char* fieldStart = textBuf + 20;
  if (*fieldStart != '^') {
    // Try to find first caret if format varies
    fieldStart = strchr(textBuf, '^');
    if (!fieldStart) return false;
  }

  // Parse caret-delimited fields: ^firstName^lastName^company^
  char* fields[4];
  uint8_t fieldCount = 0;
  char* p = fieldStart;

  while (*p && fieldCount < 4) {
    if (*p == '^') {
      p++;
      fields[fieldCount++] = p;
    } else {
      p++;
    }
  }

  // Terminate each field at the next caret
  for (uint8_t i = 0; i < fieldCount; i++) {
    char* end = strchr(fields[i], '^');
    if (end) *end = '\0';
  }

  // Build full name: "FirstName LastName"
  nfcPendingName[0] = '\0';
  if (fieldCount >= 1 && fields[0][0]) {
    strncpy(nfcPendingName, fields[0], sizeof(nfcPendingName) - 1);
  }
  if (fieldCount >= 2 && fields[1][0]) {
    size_t len = strlen(nfcPendingName);
    if (len > 0 && len < sizeof(nfcPendingName) - 2) {
      nfcPendingName[len] = ' ';
      nfcPendingName[len + 1] = '\0';
    }
    strncat(nfcPendingName, fields[1], sizeof(nfcPendingName) - strlen(nfcPendingName) - 1);
  }

  // Copy company
  if (fieldCount >= 3) {
    strncpy(nfcPendingCompany, fields[2], sizeof(nfcPendingCompany) - 1);
    nfcPendingCompany[sizeof(nfcPendingCompany) - 1] = '\0';
  }

  // Format check: only accept our badges (16-digit ID + 4-char activation code).
  // A foreign card whose NDEF happens to parse this far is rejected here, so it
  // never flashes success or has its (bogus) guest data forwarded over serial.
  if (!isOurBadgeFormat(nfcPendingBadgeId)) {
    nfcPendingValid = false;
    return false;
  }

  nfcPendingValid = true;
  return true;
}

// True while the ring is showing a result flash (success/failure) that a fresh
// tap would cut short. Mirrors the Mega sketch's "poll only in idle/session"
// guard, adapted to our dynamic state set: rather than a state whitelist, we
// block just the two feedback states and accept taps everywhere else.
//
// NOTE: "reading" is intentionally NOT blocked. When a card is pulled before its
// NDEF read completes, the firmware stays in "reading" and waits for the guest
// to re-present the card (see loop(): nfcReadComplete handling). Blocking taps
// here would kill that retry path until "reading" times out to "failure".
bool stateBlocksTaps() {
  int8_t idx = currentStateIdx;  // single-byte volatile read; atomic on this MCU
  if (idx < 0 || idx >= stateCount) return false;
  const char* n = states[idx].name;
  return strcmp(n, "success") == 0 ||
         strcmp(n, "failure") == 0;
}

// ── NFC Task (Core 0) ────────────────────────────────────────────────────────
// All I2C / PN532 blocking operations run here so the LED loop on core 1
// is never stalled. Uses getFirmwareVersion() as a health probe because
// readPassiveTargetID() returns false for both "no card" and "reader gone".
void nfcTask(void* pvParameters) {
  Wire.begin(PN532_SDA, PN532_SCL);
  Wire.setTimeOut(50);
  nfc.begin();

  uint32_t ver = nfc.getFirmwareVersion();
  if (ver) {
    nfc.SAMConfig();
    nfcAvailable = true;
    Serial.print("[nfc]  PN532 found. Firmware v");
    Serial.print((ver >> 16) & 0xFF, DEC);
    Serial.print('.');
    Serial.println((ver >> 8) & 0xFF, DEC);
    Serial.println("[nfc]  SAM configured — ready for cards");
  } else {
    nfcAvailable = false;
    Serial.println("[warn] PN532 not found — running in LED-only mode");
  }

  unsigned long lastCheckMs = millis();

  // Presence latch: a held card re-selects on every poll, so we only accept a
  // tap on the rising edge (field empty → card present). The card must then be
  // lifted — read empty for CARD_RELEASE_MS — before another tap can fire. The
  // release window absorbs the PN532's occasional false-empty polls while a
  // card is genuinely held.
  bool cardPresent = false;
  unsigned long lastSeenMs = 0;
  bool readPending = false;   // a guest tap is accepted but its NDEF read hasn't completed yet

  for (;;) {
    unsigned long now = millis();

    if (nfcAvailable) {
      // Always run the I2C poll, even when acceptTaps is off, so reset cards
      // can recover the activation regardless of the tap gate. The gate is
      // applied per-tap below for normal guest taps only.
      uint8_t uid[7] = {};
      uint8_t uidLen = 0;
      bool detected = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 50);
      if (detected) lastSeenMs = now;
      // Re-arm only after the field has been empty long enough.
      if (cardPresent && (now - lastSeenMs) > CARD_RELEASE_MS) cardPresent = false;

      // Rising edge of a presented card (respecting the release latch + cooldown).
      if (detected && !cardPresent && (now - lastTapMs) > TAP_COOLDOWN_MS) {
        String s = uidToWaveId(uid, uidLen);

        if (isResetUid(s.c_str())) {
          // RESET CARD: hard-recover the activation. Deliberately bypasses
          // acceptTaps and stateBlocksTaps() so it works even when the ring is
          // wedged in "reading"/"success"/"failure" or taps are gated off by
          // the host. Does NOT emit a scan -- loop() handles the reset.
          cardPresent = true;
          lastTapMs = now;
          strncpy(nfcResetUid, s.c_str(), sizeof(nfcResetUid) - 1);
          nfcResetUid[sizeof(nfcResetUid) - 1] = '\0';
          nfcResetRequested = true;
        }
        else if (acceptTaps && !stateBlocksTaps()) {
          // Normal guest tap. stateBlocksTaps() keeps the result flash
          // (success/failure) from being interrupted; "reading" stays tappable
          // so an early-pulled card can be re-presented to retry.
          cardPresent = true;
          lastTapMs = now;
          strncpy(nfcPendingUid, s.c_str(), sizeof(nfcPendingUid) - 1);
          nfcPendingUid[sizeof(nfcPendingUid) - 1] = '\0';

          // Signal card detected (triggers "reading" state in loop). The actual
          // NDEF read is owed by the re-read block below: we keep retrying it
          // while the card stays on the reader rather than reading just once
          // here, so a quick/wobbly tap that misses on the first attempt still
          // lands during the "reading" animation window.
          nfcCardDetected = true;
          readPending = true;
        }
      }

      // Re-read the NDEF while a read is owed and the card is still present.
      // A single detection-time read often misses on a fast/imperfectly-coupled
      // tap; the card usually stays put through the "reading" animation, so we
      // keep trying in that window. We stop on the first COMPLETE read (pages
      // read, valid badge or not — that's a definitive result), when the card
      // leaves, or when the retry window closes (then "reading" times out to
      // failure as before).
      if (readPending) {
        if (!cardPresent || (now - lastTapMs) > READ_RETRY_MS) {
          readPending = false;            // gave up — reading state will time out
        } else if (detected) {
          delay(READ_SETTLE_MS);          // let the just-selected card settle
          readNdefGuestData();            // writes nfcPending*, sets nfcReadComplete
          if (nfcReadComplete) {          // pages read → definitive result
            readPending = false;
            nfcNewScan = true;            // hand to loop(): success if valid, else failure
          }
          // else: couldn't read the pages this cycle — retry on the next poll
        }
      }

      // Health check: probe reader every NFC_HEALTH_CHECK_MS
      if (now - lastCheckMs >= NFC_HEALTH_CHECK_MS) {
        lastCheckMs = now;
        if (!nfc.getFirmwareVersion()) {
          nfcAvailable = false;
          Serial.println("[warn] PN532 lost — check connection");
        }
      }
    } else {
      // Recovery: attempt full re-init every NFC_RETRY_MS
      if (now - lastCheckMs >= NFC_RETRY_MS) {
        lastCheckMs = now;
        Wire.begin(PN532_SDA, PN532_SCL);
        Wire.setTimeOut(50);
        nfc.begin();
        uint32_t v = nfc.getFirmwareVersion();
        if (v) {
          nfc.SAMConfig();
          nfcAvailable = true;
          Serial.println("[nfc]  PN532 recovered — resuming card detection");
        }
      } else {
        vTaskDelay(pdMS_TO_TICKS(50));  // yield when idle
      }
    }
  }
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[boot] NFC Activation starting...");

  // Load states from NVS (or init defaults)
  loadStatesFromNvs();
  Serial.print("[cfg]  Loaded ");
  Serial.print(stateCount);
  Serial.println(" states");
  Serial.print("[cfg] acceptTaps=");
  Serial.println(acceptTaps ? "1" : "0");
  Serial.print("[cfg] ledEnabled=");
  Serial.println(ledEnabled ? "1" : "0");

  // Initialize LEDs
  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, LED_COUNT);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  fill_solid(leds, LED_COUNT, CRGB::Black);
  FastLED.show();
  Serial.println("[led]  FastLED ready — 40 LED ring");

  // NFC runs on core 0 so I2C blocking never stalls the LED loop on core 1
  xTaskCreatePinnedToCore(nfcTask, "nfcTask", 4096, NULL, 1, NULL, 0);

  // Start in idle state
  triggerState("idle");
  executeTransition(millis());

  Serial.println("[sys]  Setup complete.");
  Serial.println("[sys]  Send 'OK' for success, 'FAIL' for failure, 'TRIGGER <name>' for custom states");

  lastHeartbeatMs = millis();
  lastAnimMs = millis();
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // Process serial commands
  processSerial();

  // Heartbeat
  if (now - lastHeartbeatMs > HEARTBEAT_MS) {
    lastHeartbeatMs = now;
    Serial.print("[hb] ");
    Serial.println(now);
  }

  // Reset card scanned - hard-recover the activation (see nfcTask). Handled
  // before the card-detected/scan logic so a reset always takes priority.
  if (nfcResetRequested) {
    nfcResetRequested = false;
    Serial.print("[reset] state cleared by card ");
    Serial.println(nfcResetUid);

    // Drop any in-flight scan handshake so no stale success/failure fires.
    nfcCardDetected  = false;
    nfcNewScan       = false;
    nfcResultPending = false;
    nfcResultSuccess = false;

    // Re-enable tap intake -- the whole point of a reset card. Emit the
    // [cfg] line so the host learns taps are back on (matches SETTAPS).
    acceptTaps = true;
    Serial.print("[cfg] acceptTaps=");
    Serial.println(acceptTaps ? "1" : "0");

    // Force straight to the home state, bypassing the pending/transition path
    // so recovery works even if pendingStateIdx is wedged.
    int8_t homeIdx = findStateByName(RESET_HOME_STATE);
    if (homeIdx < 0) homeIdx = 0;
    pendingStateIdx = -1;
    inTransition    = false;
    currentStateIdx = homeIdx;
    stateStartMs    = now;
    Serial.print("[state] ");
    Serial.println(states[currentStateIdx].name);
  }

  // Card detected - trigger "reading" state if it exists
  if (nfcCardDetected) {
    nfcCardDetected = false;
    Serial.println("[card]");
    if (findStateByName("reading") >= 0) {
      triggerState("reading");
    }
  }

  // Forward any scans queued by nfcTask as JSON
  if (nfcNewScan) {
    nfcNewScan = false;
    {
      // Only emit scan and trigger states if read completed (card held long enough)
      if (nfcReadComplete) {
        if (nfcPendingValid) {
          // Our badge (format check passed) — forward the full guest data.
          StaticJsonDocument<256> doc;
          doc["uid"]     = nfcPendingUid;
          doc["badgeId"] = nfcPendingBadgeId;
          doc["name"]    = nfcPendingName;
          doc["company"] = nfcPendingCompany;
          Serial.print("[scan] ");
          serializeJson(doc, Serial);
          Serial.println();
        } else {
          // Not one of our badges. Deliberately do NOT emit a [scan]: the host
          // must never see this as a tap, or a downstream state could flash the
          // ring green (success) off it. We still log the uid so the dev tool
          // can enroll reset cards, and flash failure locally below.
          Serial.print("[warn] unrecognized badge format — rejected (uid ");
          Serial.print(nfcPendingUid);
          Serial.println(")");
        }

        // Queue the result - actual trigger happens after minimum reading time.
        // Only our badges (nfcPendingValid) flash success; anything else fails.
        nfcResultPending = true;
        nfcResultSuccess = nfcPendingValid;
      }
      // If read didn't complete (card removed early), stay in reading state
      // and wait for next tap
    }
  }

  // Trigger success/failure after minimum reading display time
  if (nfcResultPending) {
    unsigned long readingElapsed = now - stateStartMs;
    if (readingElapsed >= readingMinMs) {
      nfcResultPending = false;
      triggerState(nfcResultSuccess ? "success" : "failure");
    }
  }

  // Update animation time
  LedState* cs = getCurrentState();
  if (cs) {
    float dt = (now - lastAnimMs) / 16.0f;
    lastAnimMs = now;
    noiseT += cs->speed * dt;
  }

  // Execute pending transitions
  if (pendingStateIdx >= 0) {
    executeTransition(now);
  }

  // Check for duration-based return
  cs = getCurrentState();
  if (cs && cs->duration > 0 && strlen(cs->returnTo) > 0) {
    unsigned long elapsed = now - stateStartMs;
    if (elapsed >= cs->duration && pendingStateIdx < 0) {
      triggerState(cs->returnTo);
    }
  }

  // LED ring disabled (diagnostic): blank the ring once, then skip rendering and
  // FastLED.show() entirely so there's no LED current draw and no data-line signal
  // on the strip. The state machine + NFC keep running normally above.
  if (!ledEnabled) {
    if (!ringCleared) {
      fill_solid(leds, LED_COUNT, CRGB::Black);
      FastLED.show();
      ringCleared = true;
    }
    delay(16);
    return;
  }
  ringCleared = false;

  // Render LEDs
  cs = getCurrentState();
  if (cs) {
    unsigned long elapsed = now - stateStartMs;
    for (int i = 0; i < LED_COUNT; i++) {
      leds[i] = getColorForLed(i, *cs, elapsed);
    }
  } else {
    fill_solid(leds, LED_COUNT, CRGB::Black);
  }

  // Apply transition blend over the new state
  if (inTransition) {
    unsigned long tElapsed = now - transitionStartMs;
    if (tElapsed >= transitionActiveMs) {
      inTransition = false;
    } else {
      float t = (float)tElapsed / (float)transitionActiveMs;
      switch (activeTransition) {
        case T_CROSSFADE: {
          uint8_t b = (uint8_t)(t * 255.0f);
          for (int i = 0; i < LED_COUNT; i++)
            leds[i] = blend(transitionFromLeds[i], leds[i], b);
          break;
        }
        case T_FADE_BLACK:
          if (t < 0.5f) {
            uint8_t b = (uint8_t)(t * 2.0f * 255.0f);
            for (int i = 0; i < LED_COUNT; i++)
              leds[i] = blend(transitionFromLeds[i], CRGB::Black, b);
          } else {
            uint8_t b = (uint8_t)((t - 0.5f) * 2.0f * 255.0f);
            for (int i = 0; i < LED_COUNT; i++)
              leds[i] = blend(CRGB::Black, leds[i], b);
          }
          break;
        case T_FLASH_WHITE:
          if (t < 0.2f) {
            uint8_t b = (uint8_t)(t / 0.2f * 255.0f);
            for (int i = 0; i < LED_COUNT; i++)
              leds[i] = blend(transitionFromLeds[i], CRGB::White, b);
          } else {
            uint8_t b = (uint8_t)((t - 0.2f) / 0.8f * 255.0f);
            for (int i = 0; i < LED_COUNT; i++)
              leds[i] = blend(CRGB::White, leds[i], b);
          }
          break;
        default:
          break;
      }
    }
  }

  FastLED.show();
  delay(16);
}
