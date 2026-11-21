/*
  ============================================================================
  Arduino Nano + FastLED - 450x WS2811 Serpentine Matrix Sequencer
  ============================================================================
  Hardware:
    - 18 chains x 25 LEDs = 450 LEDs total, wired DOUT->DIN in series
      (one single logical strip of 450 pixels from the Nano's point of view)
    - S-type (serpentine) wiring: physical data direction alternates every
      chain -- chain 1 (odd, 1-indexed) runs top->down, chain 2 (even) runs
      down->top, chain 3 top->down, etc. All animation code below works in
      chain-local "logical" positions (0 = visual top) and calls getIndex()
      to translate into the correct physical LED index, so you never have
      to think about wiring direction inside an animation.
    - Data pin: D4
    - Target: ATmega328P (Nano), 2KB SRAM -> CRGB array below uses
      450 * 3 bytes = 1350 bytes, leaving ~700 bytes for stack/globals.
      Kept lean on purpose -- avoid adding large extra buffers.

  Sequencer:
    A non-blocking state machine in loop() runs 5 animations in turn, and
    each animation is always allowed to play all the way through to its own
    natural finish (a full hue cycle, a full bounce, a full pass through all
    11 palette colors, etc.) before the sequencer hands off to the next one
    and wraps back to animation 1 after animation 5. Nothing in this sketch
    uses delay() -- every animation is driven purely by millis().

  The 4 animations:
    1. Rainbow Diagonal Wave  - smooth scrolling rainbow across the matrix
    2. Curtain Comet          - per-chain bouncing rainbow-tail comets,
                                 alternating direction chain-to-chain
    3. Chasing                - a bright dot with a soft trail sweeping the
                                 whole 450-pixel strip, cycling through all
                                 11 palette colors per turn
    4. Bucket Fill            - each chain fills from the bottom up in
                                 falling batches, then drains and repeats
                                 with the next palette color
    Serial control (9600 baud): type anim<n> (n = 1..number of animations) to jump straight to that
    animation. It plays to its natural end, then the sequencer continues with
    the next one as normal.

    5. Palette Flow           - solid palette-color bands flowing one after
                                 another (no black gaps) top->bottom, then
                                 reversing bottom->top, on every chain
  ============================================================================
*/

// Let the UART receive interrupt run while FastLED is pushing pixels out, so
// serial commands aren't lost during the ~13 ms a 450-LED show() takes. If you
// ever see flicker on the strip, delete this line.
#define FASTLED_ALLOW_INTERRUPTS 1
#include <FastLED.h>

// ---------------------------------------------------------------------------
// CONFIGURATION
// ---------------------------------------------------------------------------
#define DATA_PIN        4
#define LED_TYPE        WS2811
#define COLOR_ORDER     RGB          // change if colors look swapped on your strip

#define NUM_CHAINS      18
#define LEDS_PER_CHAIN  25
#define NUM_LEDS        (NUM_CHAINS * LEDS_PER_CHAIN)   // 450

#define BRIGHTNESS      255          // 0-255: overall show brightness
#define FRAMES_PER_SEC  60           // global animation frame rate cap

// ---------------------------------------------------------------------------
// SHARED COLOR PALETTE
// ---------------------------------------------------------------------------
// All named colors used across the animations are defined once here, so
// Chasing (Animation 3) and Bucket Fill (Animation 4) both draw from the
// exact same palette instead of keeping their own separate copies. Edit
// colors or their order here and every animation that uses them updates.
#define PALETTE_COUNT   11

CRGB palette[PALETTE_COUNT] = {
  CRGB(148, 0, 211),   // Violet
  CRGB::Red,
  CRGB(110, 240, 3),   // Light Green
  CRGB(255, 50, 0),    // Orange
  CRGB::Blue,
  CRGB::Yellow,
  CRGB(255, 0, 80),   // Pink
  CRGB::Green,
  CRGB(255, 60, 40),   // Light Pink
  CRGB(50, 100, 255),  // Sky Blue
  CRGB(60, 255, 80)    // Spring Green
};

// Each animation sets its own *Finished flag once it has completed this many
// full passes (loop() waits for that flag before advancing -- see the
// phase-advance check below). Raise/lower a value to make that animation's
// phase longer/shorter.
#define ANIM1_CYCLES         8        // Rainbow diagonal wave: full 0-255 hue cycles before finishing
#define ANIM2_CYCLES         8        // Curtain comet: full bounce round-trips (0->end->0) before finishing
#define ANIM3_CYCLES         1        // Chasing: full passes through all 11 palette colors
#define ANIM4_CYCLES         1        // Bucket Fill: full fill+drain cycles through all 11 palette colors
#define ANIM5_CYCLES         3        // Palette Flow: full down+up round-trips before finishing

CRGB leds[NUM_LEDS];

// ---------------------------------------------------------------------------
// SEQUENCER STATE
// ---------------------------------------------------------------------------
enum AnimState : uint8_t { ANIM_1 = 0, ANIM_2, ANIM_3, ANIM_4, ANIM_5, ANIM_COUNT };
AnimState currentAnim   = ANIM_1;
uint32_t  animStartTime = 0;     // millis() timestamp when current anim began
uint32_t  lastFrameTime = 0;     // for frame-rate limiting
bool      animInitDone  = false; // true once the current anim's one-time setup has run

// Set by each animation once it has played its configured number of cycles
// (ANIM1_CYCLES..ANIM4_CYCLES above). Declared here, defined near each
// animation's code, so loop() can read them to know when to advance.
extern bool rainbowWaveFinished;
extern bool curtainCometFinished;
extern bool chasingFinished;
extern bool bucketFillFinished;
extern bool paletteFlowFinished;

// ===========================================================================
// SHARED SERPENTINE MAPPING
// ===========================================================================
// logicalPos 0 = visual top of a chain, LEDS_PER_CHAIN-1 = visual bottom.
// Chain 1 (odd, 1-indexed) data runs top->down physically (no flip needed);
// chain 2 (even) runs down->top (flipped), and so on, alternating. Every
// animation below addresses LEDs only through this function, so the
// physical S-type wiring is handled in exactly one place.
int getIndex(int chain, int logicalPos) {
  bool chainIsOdd  = ((chain + 1) % 2 == 1); // chain is 0-indexed; chain 0 = physical chain 1 (odd)
  int  physicalPos = chainIsOdd ? logicalPos : (LEDS_PER_CHAIN - 1 - logicalPos);
  return chain * LEDS_PER_CHAIN + physicalPos;
}

// ---------------------------------------------------------------------------
// FORWARD DECLARATIONS - one function per animation: *_init() runs once when
// the phase begins, *_frame() renders one frame and is called every tick.
// ---------------------------------------------------------------------------
void anim1_RainbowWave_init();
void anim1_RainbowWave_frame(uint32_t elapsedMs);

void anim2_CurtainComet_init();
void anim2_CurtainComet_frame(uint32_t elapsedMs);

void anim3_Chasing_init();
void anim3_Chasing_frame();

void anim4_BucketFill_init();
void anim4_BucketFill_frame();

void anim5_PaletteFlow_init();
void anim5_PaletteFlow_frame();

void nextAnimation();
void handleSerial();

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  delay(1000); // safety startup delay, keeps bootup current draw gentle
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS)
         .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  animStartTime = millis();
  lastFrameTime = millis();
  Serial.print(F("Ready. Type anim1 to anim"));
  Serial.print((int)ANIM_COUNT);
  Serial.print(F(" to jump to an animation."));
  Serial.print('\n');
}

// ---------------------------------------------------------------------------
// LOOP - master non-blocking sequencer / state machine
// ---------------------------------------------------------------------------
void loop() {
  handleSerial(); // polled every pass, before the frame limiter, so input stays responsive

  uint32_t now = millis();

  // --- Frame rate limiter: only render a new frame FRAMES_PER_SEC times/sec.
  if (now - lastFrameTime < (1000UL / FRAMES_PER_SEC)) {
    return;
  }
  lastFrameTime = now;

  uint32_t elapsed = now - animStartTime;

  // --- One-time setup when a phase first becomes active ---
  if (!animInitDone) {
    switch (currentAnim) {
      case ANIM_1: anim1_RainbowWave_init();  break;
      case ANIM_2: anim2_CurtainComet_init(); break;
      case ANIM_3: anim3_Chasing_init();      break;
      case ANIM_4: anim4_BucketFill_init();   break;
      case ANIM_5: anim5_PaletteFlow_init();  break;
      default: break;
    }
    animInitDone = true;
  }

  // --- Dispatch to the active animation's per-frame render function ---
  switch (currentAnim) {
    case ANIM_1: anim1_RainbowWave_frame(elapsed);  break;
    case ANIM_2: anim2_CurtainComet_frame(elapsed); break;
    case ANIM_3: anim3_Chasing_frame();             break;
    case ANIM_4: anim4_BucketFill_frame();          break;
    case ANIM_5: anim5_PaletteFlow_frame();         break;
    default: break;
  }

  FastLED.show();

  // --- Phase-advance check: each animation signals completion itself
  //     (see ANIM1_CYCLES..ANIM4_CYCLES above). ---
  bool phaseDone = false;
  switch (currentAnim) {
    case ANIM_1: phaseDone = rainbowWaveFinished;  break;
    case ANIM_2: phaseDone = curtainCometFinished; break;
    case ANIM_3: phaseDone = chasingFinished;      break;
    case ANIM_4: phaseDone = bucketFillFinished;   break;
    case ANIM_5: phaseDone = paletteFlowFinished;  break;
    default: break;
  }

  if (phaseDone) {
    nextAnimation();
  }
}

// ---------------------------------------------------------------------------
// Advances the sequencer to the next animation (wraps from 5 back to 1),
// and resets the phase timer / clears the buffer for a clean handoff.
// ---------------------------------------------------------------------------
void nextAnimation() {
  currentAnim    = (AnimState)((currentAnim + 1) % ANIM_COUNT);
  animStartTime  = millis();
  animInitDone   = false;
  FastLED.clear();   // clean slate so the previous animation doesn't "bleed" into the next
}

// ---------------------------------------------------------------------------
// SERIAL COMMANDS: "anim<n>" (n = 1..ANIM_COUNT) jumps straight to that
// animation. Because currentAnim is simply set and the normal sequencer keeps
// running, the jumped-to animation finishes on its own and then hands off to
// the next one (n+1, wrapping 5 -> 1) as usual.
// Line ending is optional: a command is also accepted after 200 ms of silence.
// ---------------------------------------------------------------------------
char     serialBuf[12];
uint8_t  serialLen    = 0;
uint32_t serialLastRx = 0;

void jumpToAnimation(uint8_t n) {
  currentAnim   = (AnimState)(n - 1);
  animStartTime = millis();
  animInitDone  = false;      // runs the new animation's init (also resets its finished flag)
  FastLED.clear();
}

// The valid range is always 1..ANIM_COUNT, so adding a new animation only
// needs a new entry in the AnimState enum (before ANIM_COUNT) -- nothing in
// the serial code has to change.
void processSerialCommand() {
  serialBuf[serialLen] = 0;
  const char* p = serialBuf;
  while (*p == ' ') p++;
  if (*p == 0) return;        // blank / whitespace-only input: ignore silently

  if (strncmp(p, "anim", 4) == 0) {
    p += 4;
    while (*p == ' ') p++;
    if (*p >= '0' && *p <= '9') {
      int n = atoi(p);
      if (n >= 1 && n <= ANIM_COUNT) {
        jumpToAnimation((uint8_t)n);
        Serial.print(F("Successfully Set to Animation "));
        Serial.print(n);
        Serial.print('\n');
        return;
      }
    }
  }
  Serial.print(F("Unknown command. Use anim1 to anim"));
  Serial.print((int)ANIM_COUNT);
  Serial.print('\n');
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    serialLastRx = millis();
    if (c == '\n' || c == '\r') {
      if (serialLen > 0) { processSerialCommand(); serialLen = 0; }
    } else if (c >= ' ' && c <= '~' && serialLen < sizeof(serialBuf) - 1) {
      serialBuf[serialLen++] = tolower(c);   // ignores stray control/non-ASCII bytes
    }
  }
  // No newline sent? Run what we have after a short pause.
  if (serialLen > 0 && millis() - serialLastRx > 200) {
    processSerialCommand();
    serialLen = 0;
  }
}

// ===========================================================================
// ANIMATION 1: Rainbow diagonal wave across the matrix.
// Uses getIndex() so the diagonal sweeps visually correctly despite the
// serpentine wiring and the per-chain alternating travel direction.
// ===========================================================================
#define WAVE_SPEED_DIVISOR   8   // controls scroll speed; SMALLER = faster scroll, LARGER = slower
#define WAVE_HUE_PER_COLUMN   6   // hue change per LED along a chain (controls how tight/diagonal the bands look)
#define WAVE_HUE_PER_CHAIN   10   // hue change per chain (controls the diagonal "tilt" of the wave)
#define WAVE_SATURATION     255   // 0-255: color saturation, 255 = fully vivid, lower = more pastel/white
#define WAVE_BRIGHTNESS     255   // 0-255: per-pixel brightness (separate from the global BRIGHTNESS setting)

bool rainbowWaveFinished;

void anim1_RainbowWave_init() {
  rainbowWaveFinished = false;
}

void anim1_RainbowWave_frame(uint32_t elapsedMs) {
  uint32_t timeHueFull = elapsedMs / WAVE_SPEED_DIVISOR; // 0-255 wraps = one full hue cycle
  uint8_t  timeHue     = (uint8_t)timeHueFull;

  for (int chain = 0; chain < NUM_CHAINS; chain++) {
    for (int pos = 0; pos < LEDS_PER_CHAIN; pos++) {
      uint8_t hue = (pos * WAVE_HUE_PER_COLUMN) + (chain * WAVE_HUE_PER_CHAIN) + timeHue;
      leds[getIndex(chain, pos)] = CHSV(hue, WAVE_SATURATION, WAVE_BRIGHTNESS);
    }
  }

  // Finishes once the hue has wrapped the 0-255 range ANIM1_CYCLES times.
  if (timeHueFull >= (uint32_t)256 * ANIM1_CYCLES) {
    rainbowWaveFinished = true;
  }
}

// ===========================================================================
// ANIMATION 2: "Curtain Comet" -- each chain runs its own independent
// bouncing comet with a soft rainbow tail. Odd chains travel one way, even
// chains the other, so the whole curtain breathes in alternating waves.
// Each chain keeps a fixed, distinct base hue so neighboring comets never
// look the same color, while the tail itself still sweeps through hues.
// ===========================================================================
#define COMET_LEN        10     // length of the glowing comet tail (LEDs)
#define COMET_HUE_SPAN    60     // hue shift across one comet's tail
#define COMET_SPEED        4     // comet movement speed (higher = faster)
#define COMET_HUE_DRIFT     1     // how fast the base palette slowly cycles
#define COMET_FADE         60     // trailing fade behind the comets
#define COMET_HUE_STEP   (256 / NUM_CHAINS)   // spreads each chain's base hue evenly

// One full round trip (0 -> end -> 0) in 8x sub-steps, for smooth slow motion.
#define COMET_SPAN   ((LEDS_PER_CHAIN - 1) * 8)
#define COMET_CYCLE  (COMET_SPAN * 2)

uint16_t cometChainPos[NUM_CHAINS];
uint8_t  cometBaseHue = 0;
bool     curtainCometFinished;

void anim2_CurtainComet_init() {
  for (int c = 0; c < NUM_CHAINS; c++) cometChainPos[c] = 0;
  cometBaseHue = 0;
  curtainCometFinished = false;
}

bool cometChainGoesUp(int chain) {
  return (chain % 2 == 0); // chain 0 (physical chain 1) travels up, chain 1 down, alternating
}

void drawChainComet(int chain) {
  uint16_t cyclePos = cometChainPos[chain] % COMET_CYCLE;

  // Triangle wave: travels 0->end then end->0, for a back-and-forth bounce.
  uint16_t linPos = (cyclePos <= COMET_SPAN) ? cyclePos : (COMET_CYCLE - cyclePos);
  int headPos = linPos / 8; // logical position (0..LEDS_PER_CHAIN-1), direction-agnostic

  uint8_t chainHue = cometBaseHue + (chain * COMET_HUE_STEP);

  for (int t = 0; t < COMET_LEN; t++) {
    int logicalPos = headPos - t;
    if (logicalPos < 0 || logicalPos >= LEDS_PER_CHAIN) continue;

    // Flip logical -> visual travel direction per chain.
    int visualPos = cometChainGoesUp(chain) ? logicalPos : (LEDS_PER_CHAIN - 1 - logicalPos);

    uint8_t brightness = 255 - (t * (255 / COMET_LEN));          // tail fades out
    uint8_t hue        = chainHue + (t * COMET_HUE_SPAN / COMET_LEN); // tail shifts hue

    leds[getIndex(chain, visualPos)] += CHSV(hue, 255, brightness); // blend for smooth overlaps
  }
}

void anim2_CurtainComet_frame(uint32_t elapsedMs) {
  fadeToBlackBy(leds, NUM_LEDS, COMET_FADE); // soft trailing glow behind every comet

  for (int c = 0; c < NUM_CHAINS; c++) {
    drawChainComet(c);
  }

  for (int c = 0; c < NUM_CHAINS; c++) {
    cometChainPos[c] += COMET_SPEED;
  }
  cometBaseHue += COMET_HUE_DRIFT;

  // Finishes once every chain's comet has completed ANIM2_CYCLES full
  // round-trips. All chains move in lockstep, so checking chain 0 suffices.
  if (cometChainPos[0] >= (uint32_t)COMET_CYCLE * ANIM2_CYCLES) {
    curtainCometFinished = true;
  }
}

// ===========================================================================
// ANIMATION 3: "Chasing" -- a bright dot with a soft trail sweeps the
// entire 450-pixel strip end to end, cycling through all colors in palette[]
// in order, looping back to the start.
//
// How many colors are in flight at once is not fixed -- it's derived from
// CHASE_GAP_LEDS and the strip length. A new color launches the instant the
// gap between the previous color's faint tail end and LED 0 reaches exactly
// CHASE_GAP_LEDS. Smaller CHASE_GAP_LEDS on a long strip lets more colors
// run simultaneously; a larger gap keeps fewer in flight together.
// ===========================================================================
#define CHASE_SPEED_MS       14   // ms between each step of a moving dot
#define CHASE_GLOW_WIDTH      2   // solid LEDs lit on either side of the chase head
#define CHASE_TRAIL_LEN       10   // length of the soft fading trail behind the head
#define CHASE_GAP_LEDS       50  // gap (LEDs) between previous color's last lit pixel and next color's first lit pixel

// Uses the shared palette[] defined near the top of the file.
#define CHASE_COLOR_COUNT PALETTE_COUNT

// Max simultaneous runners the array can hold. One full pass only ever
// launches CHASE_COLOR_COUNT colors total, so this is never exceeded
// regardless of CHASE_GAP_LEDS/NUM_LEDS.
#define CHASE_MAX_RUNNERS    CHASE_COLOR_COUNT

struct ChaseRunner {
  bool     active;
  uint8_t  colorIndex;
  int16_t  headPos;     // -CHASE_TRAIL_LEN..NUM_LEDS-1 (starts just off-strip so the trail eases in)
};
ChaseRunner chaseRunners[CHASE_MAX_RUNNERS];
// Reset every time Animation 3 starts (anim3_Chasing_init()).
uint8_t  chaseNextColorIndex;
uint32_t chaseLastStep;
uint8_t  chaseLaunchesRemaining; // how many more new colors still need to be launched this turn
bool     chasingFinished;

// A runner's "last illuminated pixel" -- the far end of its faint trail,
// however dim -- is this many LEDs behind its head.
#define CHASE_TAIL_END_OFFSET (CHASE_GLOW_WIDTH + CHASE_TRAIL_LEN)

void chaseLaunch(uint8_t slot, uint8_t colorIndex) {
  chaseRunners[slot].active     = true;
  chaseRunners[slot].colorIndex = colorIndex;
  chaseRunners[slot].headPos    = -CHASE_TRAIL_LEN;
}

// Index of the single most-recently-launched active runner (the one
// currently "at the back" of the pack) -- the gap to the NEXT color is
// always measured from this one, since it's the only runner whose tail end
// is still approaching LED 0 from behind.
int8_t chaseFindNewestActive() {
  int8_t newest = -1;
  for (uint8_t i = 0; i < CHASE_MAX_RUNNERS; i++) {
    if (chaseRunners[i].active) {
      if (newest == -1 || chaseRunners[i].headPos < chaseRunners[newest].headPos) {
        newest = i; // smallest headPos = launched most recently = furthest back
      }
    }
  }
  return newest;
}

int8_t chaseFindFreeSlot() {
  for (uint8_t i = 0; i < CHASE_MAX_RUNNERS; i++) {
    if (!chaseRunners[i].active) return i;
  }
  return -1; // shouldn't happen -- array sized to CHASE_COLOR_COUNT
}

uint8_t chaseActiveRunnerCount() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < CHASE_MAX_RUNNERS; i++) if (chaseRunners[i].active) n++;
  return n;
}

void anim3_Chasing_init() {
  for (uint8_t i = 0; i < CHASE_MAX_RUNNERS; i++) chaseRunners[i].active = false;
  chaseLastStep   = millis();
  chasingFinished = false;

  // Start a fresh pass through the whole palette from index 0 every time.
  chaseNextColorIndex    = 0;
  chaseLaunchesRemaining = CHASE_COLOR_COUNT * ANIM3_CYCLES;

  chaseLaunch(0, chaseNextColorIndex);
  chaseNextColorIndex = (chaseNextColorIndex + 1) % CHASE_COLOR_COUNT;
  chaseLaunchesRemaining--;
}

void anim3_Chasing_frame() {
  uint32_t now = millis();
  if (now - chaseLastStep < CHASE_SPEED_MS) return; // pace the chase independently of the frame cap
  chaseLastStep = now;

  FastLED.clear(); // redraw fresh each step -- no global fade, so colors stay clean/solid

  for (uint8_t s = 0; s < CHASE_MAX_RUNNERS; s++) {
    if (!chaseRunners[s].active) continue;

    CRGB color = palette[chaseRunners[s].colorIndex];

    // Solid head + small glow on either side.
    if (chaseRunners[s].headPos >= 0 && chaseRunners[s].headPos < NUM_LEDS) {
      leds[chaseRunners[s].headPos] = color;
    }
    for (int g = 1; g <= CHASE_GLOW_WIDTH; g++) {
      int p1 = chaseRunners[s].headPos + g;
      int p2 = chaseRunners[s].headPos - g;
      if (p1 >= 0 && p1 < NUM_LEDS) leds[p1] = color;
      if (p2 >= 0 && p2 < NUM_LEDS) leds[p2] = color;
    }

    // Soft fading trail behind the head (local only -- not a strip-wide fade).
    for (int t = 1; t <= CHASE_TRAIL_LEN; t++) {
      int p = chaseRunners[s].headPos - CHASE_GLOW_WIDTH - t;
      if (p < 0 || p >= NUM_LEDS) continue;
      uint8_t trailBrightness = 255 - (t * (255 / CHASE_TRAIL_LEN));
      leds[p] += color.nscale8_video(trailBrightness);
    }

    chaseRunners[s].headPos++;

    // Retire a runner once its whole trail has left the strip, freeing its
    // slot for a future launch.
    if (chaseRunners[s].headPos - CHASE_TAIL_END_OFFSET >= NUM_LEDS) {
      chaseRunners[s].active = false;
    }
  }

  // Launch the next color once the gap between the newest active runner's
  // tail end (headPos - CHASE_TAIL_END_OFFSET) and LED 0 equals
  // CHASE_GAP_LEDS, i.e. headPos >= CHASE_GAP_LEDS + CHASE_TAIL_END_OFFSET.
  // Checked each frame against whichever runner is furthest back.
  if (chaseLaunchesRemaining > 0) {
    int8_t newest = chaseFindNewestActive();
    bool gapReached = (newest == -1) ||
      (chaseRunners[newest].headPos >= (int16_t)(CHASE_GAP_LEDS + CHASE_TAIL_END_OFFSET));

    if (gapReached) {
      int8_t freeSlot = chaseFindFreeSlot();
      if (freeSlot != -1) {
        chaseLaunch(freeSlot, chaseNextColorIndex);
        chaseNextColorIndex = (chaseNextColorIndex + 1) % CHASE_COLOR_COUNT;
        chaseLaunchesRemaining--;
      }
    }
  }

  // All colors launched and no runner still in flight -> full pass done.
  if (chaseLaunchesRemaining == 0 && chaseActiveRunnerCount() == 0) {
    chasingFinished = true;
  }
  // Safety net: if every runner went dark but colors are still left to
  // show, launch the next one immediately instead of stalling dark.
  else if (chaseLaunchesRemaining > 0 && chaseActiveRunnerCount() == 0) {
    chaseLaunch(0, chaseNextColorIndex);
    chaseNextColorIndex = (chaseNextColorIndex + 1) % CHASE_COLOR_COUNT;
    chaseLaunchesRemaining--;
  }
}

// ===========================================================================
// ANIMATION 4: "Bucket Fill" -- every chain fills from the bottom up in
// falling batches (like water pouring in), holds full, then drains dark
// and refills with the next rainbow color. Repeats for ANIM4_CYCLES cycles,
// then signals the sequencer to move on via bucketFillFinished.
// ===========================================================================
#define BUCKET_BATCH_SIZE        3     // how many LEDs drop/stack together per "pour"
#define BUCKET_FALL_STEP_MS     17     // ms between each step of a falling batch
#define BUCKET_PAUSE_FULL_MS  1000     // how long to hold the fully-lit bucket
#define BUCKET_PAUSE_DARK_MS    200    // how long to stay dark before the next pour

// Uses the shared palette[] defined near the top of the file.

// Bucket fill runs its own little internal state machine (filling a batch,
// pausing full, pausing dark) so it stays fully non-blocking too.
enum BucketPhase : uint8_t { BF_DROP, BF_HOLD_FULL, BF_HOLD_DARK };
BucketPhase bucketPhase;
uint8_t  bucketColorIndex;
uint8_t  bucketFilledCount;     // LEDs filled at the bottom (shared across all chains)
uint8_t  bucketBatchCount;      // size of the batch currently falling
int      bucketBatchStep;       // current fall step of the active batch
int      bucketBatchTravelSteps;
uint32_t bucketPhaseStart;
uint8_t  bucketCyclesDone;
bool     bucketFillFinished;    // set true once ANIM4_CYCLES fills+drains are complete

void bucketStartNewBatch() {
  int landingBottom = LEDS_PER_CHAIN - 1 - bucketFilledCount;
  bucketBatchCount = min((uint8_t)BUCKET_BATCH_SIZE, (uint8_t)(LEDS_PER_CHAIN - bucketFilledCount));
  bucketBatchTravelSteps = landingBottom - (bucketBatchCount - 1);
  bucketBatchStep  = 0;
  bucketPhaseStart = millis();
}

void anim4_BucketFill_init() {
  bucketColorIndex   = 0;
  bucketFilledCount  = 0;
  bucketCyclesDone   = 0;
  bucketFillFinished = false;
  bucketPhase        = BF_DROP;
  bucketStartNewBatch();
}

// Draws the current frame of a falling batch, plus the permanently-filled
// stack beneath it, identically on every chain (via getIndex()).
void bucketDrawFallingFrame() {
  int landingBottom = LEDS_PER_CHAIN - 1 - bucketFilledCount;
  CRGB color = palette[bucketColorIndex];

  for (int c = 0; c < NUM_CHAINS; c++) {
    // Re-draw the permanently filled stack at the bottom.
    for (int f = 0; f < bucketFilledCount; f++) {
      int logicalPos = LEDS_PER_CHAIN - 1 - f;
      leds[getIndex(c, logicalPos)] = color;
    }
    // Clear the falling region above the stack, then draw the falling batch.
    for (int logicalPos = 0; logicalPos <= landingBottom; logicalPos++) {
      bool isFallingLED = (logicalPos >= bucketBatchStep) &&
                           (logicalPos < bucketBatchStep + bucketBatchCount);
      leds[getIndex(c, logicalPos)] = isFallingLED ? color : CRGB::Black;
    }
  }
}

void anim4_BucketFill_frame() {
  uint32_t now = millis();

  switch (bucketPhase) {
    case BF_DROP: {
      if (now - bucketPhaseStart < BUCKET_FALL_STEP_MS) {
        // still waiting for this fall-step's timer -- just redraw current frame
        bucketDrawFallingFrame();
        return;
      }
      bucketPhaseStart = now;
      bucketDrawFallingFrame();

      bucketBatchStep++;
      if (bucketBatchStep > bucketBatchTravelSteps) {
        // batch has landed -- add it to the permanent stack
        bucketFilledCount += bucketBatchCount;
        if (bucketFilledCount >= LEDS_PER_CHAIN) {
          bucketPhase      = BF_HOLD_FULL;
          bucketPhaseStart = now;
        } else {
          bucketStartNewBatch();
        }
      }
      break;
    }

    case BF_HOLD_FULL: {
      if (now - bucketPhaseStart >= BUCKET_PAUSE_FULL_MS) {
        FastLED.clear();
        bucketPhase      = BF_HOLD_DARK;
        bucketPhaseStart = now;
      }
      break;
    }

    case BF_HOLD_DARK: {
      if (now - bucketPhaseStart >= BUCKET_PAUSE_DARK_MS) {
        bucketColorIndex = (bucketColorIndex + 1) % PALETTE_COUNT;
        bucketFilledCount = 0;
        bucketCyclesDone++;

        // Runs through the whole palette ANIM4_CYCLES times before finishing.
        if (bucketCyclesDone >= (uint8_t)(PALETTE_COUNT * ANIM4_CYCLES)) {
          bucketFillFinished = true; // tells the master sequencer we're done
        } else {
          bucketPhase = BF_DROP;
          bucketStartNewBatch();
        }
      }
      break;
    }
  }
}

// ===========================================================================
// ANIMATION 5: "Palette Flow" -- palette[] colors are laid out as solid
// bands, edge to edge with no black between them, and the whole pattern
// slides top->bottom along every chain, then reverses bottom->top. Each
// chain uses the same logical positions via getIndex(), so S-type wiring is
// handled automatically and all 18 chains show matching horizontal bands.
// One cycle = one sweep down + one sweep up. Repeats ANIM5_CYCLES times.
// ===========================================================================
#define FLOW_BAND_LEN      5      // LEDs per color band (visible colors ~ LEDS_PER_CHAIN / FLOW_BAND_LEN)
#define FLOW_STEP_MS       40    // ms per 1-LED shift (smaller = faster)
#define FLOW_SWEEP_STEPS   70     // LED shifts per direction before reversing

#define FLOW_PERIOD  (PALETTE_COUNT * FLOW_BAND_LEN)  // pattern length before it repeats

int16_t  flowOffset;        // how far the pattern has slid (grows going down, shrinks going up)
int16_t  flowStepsDone;     // shifts completed in the current direction
bool     flowGoingDown;
uint8_t  flowCyclesDone;
uint32_t flowLastStep;
bool     paletteFlowFinished;

void anim5_PaletteFlow_init() {
  flowOffset          = 0;
  flowStepsDone       = 0;
  flowGoingDown       = true;
  flowCyclesDone      = 0;
  flowLastStep        = millis();
  paletteFlowFinished = false;
}

void anim5_PaletteFlow_frame() {
  // Draw current state: every LED always gets a palette color (no black).
  for (int chain = 0; chain < NUM_CHAINS; chain++) {
    for (int pos = 0; pos < LEDS_PER_CHAIN; pos++) {
      int16_t p = (int16_t)(pos - flowOffset) % FLOW_PERIOD;
      if (p < 0) p += FLOW_PERIOD;
      leds[getIndex(chain, pos)] = palette[p / FLOW_BAND_LEN];
    }
  }

  uint32_t now = millis();
  if (now - flowLastStep < FLOW_STEP_MS) return;
  flowLastStep = now;

  // Shift pattern: +1 moves colors toward the bottom, -1 toward the top.
  flowOffset += flowGoingDown ? 1 : -1;
  flowStepsDone++;

  if (flowStepsDone >= FLOW_SWEEP_STEPS) {
    flowStepsDone = 0;
    if (flowGoingDown) {
      flowGoingDown = false;            // bottom reached -> reverse
    } else {
      flowGoingDown = true;             // back at top -> one full cycle done
      if (++flowCyclesDone >= ANIM5_CYCLES) paletteFlowFinished = true;
    }
  }
}
