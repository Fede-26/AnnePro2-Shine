/* Algorithm controlling the LEDs. */

#include "matrix.h"
#include "ch.h"
#include "board.h"
#include "hal.h"
#include "settings.h"

/* LED Matrix state */
led_t ledColors[KEY_COUNT];
led_t ledMask[KEY_COUNT];
led_t ledSticky[KEY_COUNT];

led_t noColor = {.p.red = 0, .p.green = 0, .p.blue = 0, .p.alpha = 0};

volatile bool needToCallbackProfile = false;
bool matrixEnabled;

/* Animations */
/* If non-zero the animation is enabled; 1 is full speed */
volatile uint16_t animationSkipTicks = 0;

/* Animation tick counter used to slow down animations */
uint16_t animationTicks = 0;

/* Internal function prototypes */
static void mainCallback(GPTDriver *_driver);
static void pwmRowDimmer(void);
static void pwmNextColumn(void);

const ioline_t ledColumns[NUM_COLUMN] = {
    LINE_LED_COL_1,  LINE_LED_COL_2,  LINE_LED_COL_3,  LINE_LED_COL_4,
    LINE_LED_COL_5,  LINE_LED_COL_6,  LINE_LED_COL_7,  LINE_LED_COL_8,
    LINE_LED_COL_9,  LINE_LED_COL_10, LINE_LED_COL_11, LINE_LED_COL_12,
    LINE_LED_COL_13, LINE_LED_COL_14};

const ioline_t ledRows[NUM_ROW * 3] = {
    LINE_LED_ROW_1_R, LINE_LED_ROW_1_G, LINE_LED_ROW_1_B,

    LINE_LED_ROW_2_R, LINE_LED_ROW_2_G, LINE_LED_ROW_2_B,

    LINE_LED_ROW_3_R, LINE_LED_ROW_3_G, LINE_LED_ROW_3_B,

    LINE_LED_ROW_4_R, LINE_LED_ROW_4_G, LINE_LED_ROW_4_B,

    LINE_LED_ROW_5_R, LINE_LED_ROW_5_G, LINE_LED_ROW_5_B,
};

static mutex_t mtx;

/*
 * Profile callbacks run here rather than in the PWM interrupt. NORMALPRIO
 * keeps them below the thread draining the UART in main().
 */
static THD_WORKING_AREA(waAnimation, 384);
static binary_semaphore_t animationSem;

/*
  Original firmware reference:
  ~9.81ms column cycle measured -> 100Hz.

  During each column cycle the row is "strobed" 3 - 32 times (measured on white
  with different intensity settings). This wastes some shining time, but limits
  the current.

  The row strobing is done at around 70kHz.

  We would like to achieve more colors than the original firmware.

  80kHz with pwmCounterLimit=80 will scan 14 columns at 71.4Hz - enough for
  smooth animations, and allows for 6bit LED brightness control.

  Tests on oscilloscope suggest it can reach 100kHz.
*/
static const GPTConfig bftm0Config = {.frequency = 80000,
                                      .callback = mainCallback};

/* Currently scanned column */
static uint8_t currentColumn = 0;

/*
 * Time each row has left to shine within the current column cycle.
 * Row1 R-G-B, Row2 R-G-B, Row3 R-G-B, ...
 */
static uint8_t rowTimes[NUM_ROW * 3];

/* Number of still enabled rows in current column */
static uint8_t rowsEnabled;

/*
 * pwmCounter which counts time of lit rows within each column cycle.
 */
uint16_t pwmCounter;

/*
 * pwmCounter goes over 64 a bit to limit the current of completely white
 * board (0.5A) vs <0.3A for original firmware.
 *
 * You can get brighter LEDs if you set this to 64. And possibly burn the
 * board in the longer period of time.
 */
const uint16_t pwmCounterLimit = 80;

/*
 * Per channel scaling factor folded out of color_correction and
 * color_temperature, indexed like led_t.pv (0 - blue, 1 - green, 2 - red).
 *
 * Both inputs are configuration, so the factor only changes when they do.
 * It used to be recomputed for all 15 channels of every column, which is
 * ~15k divisions per second inside the PWM interrupt - and this core has no
 * divide instruction, so each one is a call into __aeabi_uidiv.
 */
static uint8_t colorAdjust[3] = {0xFF, 0xFF, 0xFF};

void matrixUpdateColorAdjust(void) {
  for (size_t i = 0; i < 3; i++) {
    const uint8_t cc = color_correction.pv[i];
    const uint8_t ct = color_temperature.pv[i];
    if (cc > 0 && ct > 0) {
      uint32_t work = (((uint32_t)cc) + 1) * (((uint32_t)ct) + 1) * 0xFF;
      work /= 0x10000L;
      colorAdjust[i] = work & 0xFF;
    } else {
      colorAdjust[i] = 0xFF;
    }
  }
}

/* Disable timeouted LEDs */
static inline void pwmRowDimmer() {
  for (size_t ledRow = 0; ledRow < NUM_ROW * 3; ledRow++) {
    const uint8_t time = rowTimes[ledRow];
    if (pwmCounter == time) {
      palClearLine(ledRows[ledRow]);
      rowsEnabled--;
    }
  }
  if (rowsEnabled == 0) {
    /* Limit color bleed by disabling the column as early as possible */
    palClearLine(ledColumns[currentColumn]);
  }
}

/* Start new PWM cycle */
static inline void pwmNextColumn() {
  /* Disable previously lit column */
  palClearLine(ledColumns[currentColumn]);

  /* Not "% NUM_COLUMN": 14 is not a power of two, so that compiled into a
   * call to __aeabi_idivmod on every column cycle. */
  if (++currentColumn >= NUM_COLUMN)
    currentColumn = 0;

  /* Prepare the PWM data and enable leds for non-zero colors */
  rowsEnabled = 0;
  for (size_t keyRow = 0; keyRow < NUM_ROW; keyRow++) {
    const uint8_t ledIndex = ROWCOL2IDX(keyRow, currentColumn);
    led_t cl;
    /* TODO: Maybe... weight with alpha? */
    if (ledSticky[ledIndex].p.alpha) {
      cl = ledSticky[ledIndex];
    } else if (ledMask[ledIndex].p.alpha && !backlightDisabled) {
      cl = ledMask[ledIndex];
    } else if (!backlightDisabled) {
      cl = ledColors[ledIndex];
    } else {
      // user disabled backlight, but sticky keys are keeping
      // it alive, unless a sticky key exists, led should
      // be turned off
      cl = noColor;
    }

    for (size_t colorIdx = 0; colorIdx < 3; colorIdx++) {
      const uint8_t ledRow = 3 * keyRow + colorIdx;

      /* Apply the precomputed correction.
       *
       * (x + 1 + (x >> 8)) >> 8 equals x / 0xFF for every x a uint8_t pair
       * can produce, so this drops the division without touching a single
       * output value. */
      const uint16_t work = (uint16_t)cl.pv[2 - colorIdx] *
                            (uint16_t)colorAdjust[2 - colorIdx];
      uint8_t color = (work + 1 + (work >> 8)) >> 8;

      /* >>2 to decrease the color resolution from 0-255 to 0-63 */
      color = color >> 2;
      if (color > 0) {
        /* Each led is enabled for color>0 even for a short while. */
        palSetLine(ledRows[ledRow]);
        rowsEnabled++;
      }
      rowTimes[ledRow] = color;
    }
  }

  /* Enable the current LED column if at least one row needs this. Limit bleed
     and maybe power consumption on reactive profiles. */
  if (rowsEnabled) {
    palSetLine(ledColumns[currentColumn]);
  }
}

/*
 * Update lighting table as per animation
 */
static THD_FUNCTION(animationThread, arg) {
  (void)arg;
  while (true) {
    chBSemWait(&animationSem);
    if (manualControl)
      continue;
    needToCallbackProfile = false;
    profiles[currentProfile].callback(ledColors);
  }
}

/*
 * mainCallback is called by GPT timer periodically
 * and is responsible for 2 things:
 * - software PWM
 * - waking the thread that repaints animated profiles
 */
void mainCallback(GPTDriver *_driver) {
  (void)_driver;

  pwmCounter += 1;
  if (pwmCounter < pwmCounterLimit) {
    /* Once every row has timed out there is nothing left to switch off, and
     * the remaining ticks of the cycle have no work to do. */
    if (rowsEnabled)
      pwmRowDimmer();
    return;
  }

  bool repaint = false;

  /* Update profile if required before starting new cycle */
  if (!manualControl && needToCallbackProfile) {
    repaint = true;
  }

  /* Animation can be updated after each full column cycle. On
   * pwmCounterLimit=80 + 80kHz timer this refreshes at 80kHz/80/14 = 71Hz and
   * should be a sensible maximum speed for a fluent smooth animation.
   */
  if (!manualControl && animationSkipTicks > 0 && currentColumn == 13) {
    animationTicks++;
    if (animationTicks >= animationSkipTicks) {
      animationTicks = 0;
      repaint = true;
    }
  }

  /* The profile callbacks walk all 70 keys and are far too long to run at
   * this priority: this interrupt fires every 12.5us and cannot be preempted
   * by the UART, so the time spent here is time the UART receive register is
   * not being read. Hand the work to a thread instead. The semaphore is
   * binary, so a frame we are too late for is dropped rather than queued. */
  if (repaint) {
    chSysLockFromISR();
    chBSemSignalI(&animationSem);
    chSysUnlockFromISR();
  }

  /* We start a new PWM column cycle. */
  pwmCounter = 0;

  pwmNextColumn();
}

/*
 * Turn off LEDs and PWM interrupt.
 */
void matrixDisable() {
  backlightDisabled = 1;
  if (stickyKeysExist) {
    return;
  }

  chMtxLock(&mtx);

  // stop timer, clock is still enabled
  if (GPTD_BFTM0.state == GPT_CONTINUOUS) {
    gptStopTimer(&GPTD_BFTM0);
  }
  // enter low power mode
  if (GPTD_BFTM0.state == GPT_READY) {
    gptStop(&GPTD_BFTM0);
  }

  palClearLine(LINE_LED_PWR);

  for (int ledRow = 0; ledRow < NUM_ROW * 3; ledRow++) {
    palClearLine(ledRows[ledRow]);
  }
  for (int i = 0; i < NUM_COLUMN; i++) {
    palClearLine(ledColumns[i]);
  }
  matrixEnabled = false;
  chMtxUnlock(&mtx);
}

/*
 * Turn on LED power and start PWM interrupt.
 */
void matrixEnable(void) {
  backlightDisabled = 0;
  chMtxLock(&mtx);

  palSetLine(LINE_LED_PWR);

  // start PWM handling interval
  gptStart(&GPTD_BFTM0, &bftm0Config);
  gptStartContinuous(&GPTD_BFTM0, 1);

  matrixEnabled = true;
  chMtxUnlock(&mtx);
}

/* Initialize matrix module */
void matrixInit() {
  chMtxObjectInit(&mtx);
  matrixUpdateColorAdjust();

  chBSemObjectInit(&animationSem, true /* taken */);
  chThdCreateStatic(waAnimation, sizeof(waAnimation), NORMALPRIO,
                    animationThread, NULL);

  matrixEnabled = false;
}
