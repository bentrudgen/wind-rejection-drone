/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Wind-disturbance companion firmware for the Team 10
  *                   Companion Board v2 (STM32F411CEU6)
  ******************************************************************************
  * Sits between the pilot and the Betaflight flight controller:
  *
  *   pilot input (LoRa from the controller PCB)  ->  this MCU  ->  MSP  ->  FC
  *                                                     |
  *                                   reads attitude / gyro / motor outputs back from the FC,
  *                                   adds a small wind-rejection bias to roll and pitch
  *
  * PIN MAP (from SeniorDesign_CompanionBoard_v2.SchDoc)
  * ---------------------------------------------------------------------------
  *  FC UART   (J2)  USART1  PA9  = TX (MCU TX -> FC RX)   PA10 = RX (FC TX -> MCU RX)   115200 8N1
  *  DEBUG UART(J7)  USART6  PA11 = TX                      PA12 = RX                      115200 8N1
  *  LED link        PB5  (yellow DS2, via R13)   "CRSF/LoRa link LED"
  *  LED correction  PB6  (red    DS1, via R12)   "Correction active LED"
  *  LoRa RFM95W     SPI1: PA5 SCK, PA6 MISO, PA7 MOSI, PA4 NSS, PB0 RESET, PB1 DIO0, PA0 DIO1, PA2 DIO2
  *  SD card         SPI2: PB13 SCK, PB14 MISO, PB15 MOSI, PA8 CS, PB10 card-detect
  *  ESP32-CAM       PA1 = CAM_EN
  *  Programming     J6 (SWD): PA13 SWDIO, PA14 SWCLK, NRST
  *  Spare / unused  PA3 PB2 PB3 PB4 PB7 PB8 PB9 PB12 PA15
  *  (LoRa and SD pins are owned by the controller-link / logging code, not configured here.)
  *
  * FILE MAP - press Ctrl+F and search for "PART 1", "PART 2" ... to jump (the file now reads top to bottom in this order):
  *   PART 1  SETTINGS      every #define you might change: switches, radio, packet layout, sticks/buttons, correction tuning, LEDs
  *   PART 2  BOARD SETUP   how the MCU connects to the custom board: debug printing, clock, FC UART, debug UART, LEDs
  *   PART 3  FC LINK       MSP functions that talk to the Betaflight flight controller
  *   PART 4  RECEIVER      LoRa radio driver, packet decoding, ARM / correction buttons (and the CRSF option)
  *   PART 5  CORRECTION    the wind-correction math and the off-vs-on statistics
  *   PART 6  MAIN PROGRAM  boot sequence, then the 50 Hz flight loop in order:
  *                         1 pilot input, 2 flight test, 3 failsafe, 4 sensors + baseline + correction,
  *                         5 motor read-back, 6 send to FC, 7 logging, 8 loop timing
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

#include "sd_logger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* >>>>>> 1A. SWITCHES: input source, flight test, logging <<<<<< */
/* =====================  BUILD-TIME SWITCHES  ================================ */

/* Where pilot input comes from:
 *   INPUT_LORA   - the custom controller over the RFM95W (fill in pilot_lora_poll() below)
 *   INPUT_CRSF   - an ExpressLRS receiver on USART6 RX (J7 pin 2). The debug UART is disabled in this mode.
 *   INPUT_NONE   - no pilot (bench tests / the flight test below only)                               */
#define INPUT_LORA   0
#define INPUT_CRSF   1
#define INPUT_NONE   2
#define INPUT_SOURCE INPUT_LORA

#define AUTO_ARM_AT_BOOT          0   /* 1 = firmware raises AUX1 itself at boot (bench only, props off!) */
#define CORRECTION_ALWAYS_ON      0   /* 1 = ignore the correction switch (bench testing with no controller) */
#define FC_PIN_TEST               0   /* 1 = wiring test: checks whether the FC's TX pad is driving the board's RX pin (PA10).
                                         * Does NOTHING else (no MSP, no radio). Read pa10PullDownHigh / pa10PullUpHigh
                                         * in Live Expressions (each 0..100) and compare with the table in the notes. */
#define FC_CLI_TEST               0   /* 1 = round-trip test: sends '#' to the FC (Betaflight answers with its CLI banner),
                                         * stores the reply in cliRx[] / cliRxCount, then sends "exit" so the FC reboots
                                         * back to normal. Runs once, then idles. Nothing else (no MSP, no radio) runs. */
#define FC_TX_TEST                0   /* 1 = "does the FC hear the board?" test: streams MSP RC frames at 50 Hz with distinctive
                                         * values (roll 1700, pitch 1300, yaw 1600) and never waits for a reply. Watch the
                                         * Betaflight Receiver tab (receiver mode MSP): if those values show up and the
                                         * failsafe banner goes away, the board's TX reaches the FC. Nothing else runs. */
#define FC_INV_TEST               0   /* 1 = like FC_CLI_TEST, but the '#' is sent with INVERTED polarity (bit-banged on PA9).
                                         * If the FC's R2 pad has a hardware inverter in front of its UART, this is the one
                                         * that gets an answer. Text in cliRx = inverted input. Runs once, then idles. */
#define FC_HELLO_TEST             0   /* 1 = the board just prints "HELLO FROM COMPANION BOARD" out of its FC UART (PA9) every
                                         * 200 ms at 115200. Used with Betaflight's serialpassthrough command to read the text
                                         * on the PC and prove the FC's R2 pad receives it. Nothing else runs. */
#define UART_LOOPBACK_TEST        0   /* 1 = FC-link UART talks to ITSELF inside the chip (half-duplex), no wires needed.
                                         * Bench check of the board's baud rate. After a run, mspRxLast should repeat
                                         * 24 4D 3C 00 6C 6C. Set back to 0 for real use. */

/* Flight test (strapped-down thrust check). To DISABLE it, leave the next line commented out. OFF by default.
 * It waits for the baseline, counts down, arms, ramps throttle to FLIGHT_TEST_THROTTLE_US, holds,
 * ramps down and disarms. If a pilot link is present, its ARM switch (AUX1) is a kill switch. */
//#define FLIGHT_TEST_ENABLE
#define FLIGHT_TEST_THROTTLE_US       1050  /* target throttle, 1000 = idle, 2000 = full. Keep small, raise slowly */
#define FLIGHT_TEST_MAX_THROTTLE_US   1070  /* hard cap - the test can never exceed this */
#define FLIGHT_TEST_START_DELAY_MS    10000 /* countdown after the baseline locks */
#define FLIGHT_TEST_HOLD_MS           5000  /* time at the target throttle */
#define FLIGHT_TEST_RAMP_US_PER_SEC   50    /* ramp speed, us of throttle per second */
#define FLIGHT_TEST_ANGLE_AUX_CHANNEL 6     /* rc index driven high during the test to request ANGLE mode (6 = AUX3). -1 = off */
#if FLIGHT_TEST_THROTTLE_US > FLIGHT_TEST_MAX_THROTTLE_US
#error FLIGHT_TEST_THROTTLE_US is above FLIGHT_TEST_MAX_THROTTLE_US
#endif

/* Debug output on the DEBUG UART (J7, USART6 TX, 115200 8N1) - plug a USB-serial adapter into J7 pin 3.
 * Plain UART output, so unlike semihosting it never hangs the MCU when no debugger is attached. */
#define LOG_CSV_ENABLE            1   /* 1 = stream a CSV line for graphing */
#define LOG_CSV_EVERY_N_LOOPS     5
#define STATS_ENABLE              1   /* 1 = print RMS/peak attitude deviation, correction OFF vs ON */
#define STATS_PRINT_MS            5000
#define STATUS_PRINT_EVERY_N_LOOPS 50

#if INPUT_SOURCE == INPUT_CRSF
#define DEBUG_UART_ENABLE         0   /* USART6 is busy receiving CRSF at 420000 baud */
#else
#define DEBUG_UART_ENABLE         1
#endif

#define SD_LOG_ENABLE             1   /* 1 = log to the SD card */
#define SD_LOG_EVERY_N_LOOPS      1   /* 1 = a row every loop (50 Hz) */


/* >>>>>> 1B. RECEIVER SETTINGS: radio, packet layout, sticks and buttons  <<<<<< */
/* ==========================================================================================
 *  LoRa LINK TO THE CONTROLLER  (RFM95W / SX1276 on SPI1)
 *  Filled in from the teammate's receiver sketch (915 MHz, SF7, 125 kHz, CR 4/5, CRC on, 8-byte packet). Re-check if the transmitter changes.
 * ========================================================================================== */

/* ---- 1. Radio settings: MUST match the transmitter's lora.h exactly ---- */
#define LORA_FREQ_HZ              915000000UL  /* 915 MHz (from the receiver sketch: LORA_FREQUENCY 915E6) */
#define LORA_BW_CODE              7            /* 7 = 125 kHz (sketch: 125E3). 8 = 250 kHz, 9 = 500 kHz */
#define LORA_SF                   7            /* spreading factor 7 (sketch: LORA_SPREADING_FACTOR 7) */
#define LORA_CR_CODE              1            /* 1 = 4/5 (sketch: LORA_CODING_RATE 5). 2 = 4/6, 3 = 4/7, 4 = 4/8 */
#define LORA_SYNC_WORD            0x12         /* Arduino LoRa library default (sketch never changes it) */
#define LORA_PREAMBLE_LEN         8            /* Arduino LoRa library default (sketch never changes it) */
#define LORA_PAYLOAD_CRC_ON       1            /* sketch calls LoRa.enableCrc() */

/* ---- 2. Packet layout: 8 bytes, from the receiver sketch ("must match the ESP32 transmitter") ----
 *   byte 0     protocol version (always 1)
 *   byte 1-2   sequence number, uint16 little-endian
 *   byte 3     joystick 1 X, int8, -100..+100
 *   byte 4     joystick 1 Y, int8, -100..+100
 *   byte 5     joystick 2 X, int8, -100..+100
 *   byte 6     joystick 2 Y, int8, -100..+100
 *   byte 7     button flags: bit 0 = button 1, bit 1 = button 2 (1 = pressed)
 * There is no CRC byte; the radio's own payload CRC (enabled above) protects the packet.            */
#define PKT_LEN                   8
#define PKT_USE_MAGIC             1            /* byte 0 is the protocol version: packets with another value are dropped */
#define PKT_MAGIC_OFF             0
#define PKT_MAGIC_VALUE           0x01
#define PKT_USE_SEQ               1            /* low byte of the 16-bit sequence number drives the lost-packet count */
#define PKT_SEQ_OFF               1
#define PKT_USE_CRC8              0

/* Axis encoding. Offsets are byte positions in the payload. */
#define AXIS_BYTES                1            /* one byte per axis */
#define AXIS_SIGNED               1            /* 1 = the byte is a signed int8 (-100..+100), 0 = unsigned */
#define AXIS_BIG_ENDIAN           0            /* unused for 1-byte axes */
#define AXIS_LX_OFF               3            /* joystick 1 X  (left stick, left/right)  */
#define AXIS_LY_OFF               4            /* joystick 1 Y  (left stick, up/down)     */
#define AXIS_RX_OFF               5            /* joystick 2 X  (right stick, left/right) */
#define AXIS_RY_OFF               6            /* joystick 2 Y  (right stick, up/down)    */
#define AXIS_RAW_MIN              (-100)       /* value with the stick fully one way  */
#define AXIS_RAW_CENTER           0            /* value with the stick released       */
#define AXIS_RAW_MAX              100          /* value with the stick fully the other way */
#define AXIS_LX_INVERT            1            /* YAW inverted (left stick left/right). flip direction: set 1 if the drone moves the wrong way (check stickUs[] / Betaflight Receiver tab) */
#define AXIS_LY_INVERT            0
#define AXIS_RX_INVERT            1            /* ROLL inverted (right stick left/right) */
#define AXIS_RY_INVERT            0
#define STICK_DEADBAND_US         25           /* ignore tiny stick offsets (stick drift) */

/* Stick scaling: how much of the full +/-500 us range a full stick deflection sends to the FC.
 * 100 = full range. 30 = full stick sends +/-150 us (1350..1650), for small, controlled movements.
 * Applies to every pilot input source (LoRa or CRSF). Throttle is not scaled (it has THROTTLE_MAX_US).
 * The correction still decides "is the pilot using this axis?" from the stick BEFORE scaling,
 * so CORRECTION_STICK_DEADBAND_US keeps meaning the same physical stick movement at any scale. */
#define STICK_SCALE_ROLL_PCT      20
#define STICK_SCALE_PITCH_PCT     20
#define STICK_SCALE_YAW_PCT       20
/* Stick hold: when 1, letting a stick spring back to centre keeps the last push instead of returning to 0.
 *   - Each push sets that axis to the FURTHEST point reached during the push (so the spring-back on release
 *     doesn't change it). Pushing harder raises it; a new push in the same direction replaces it.
 *   - A push in the OPPOSITE direction clears the axis back to centre (0). Push again to set a new value.
 *   - Pushes smaller than STICK_HOLD_MIN_PUSH_US are ignored, so stick jitter and the overshoot when the stick
 *     snaps back can't set or clear anything.
 *   - Holds only apply while armed, and are cleared on disarm and on link loss. While disarmed the sticks
 *     pass through live (so the Betaflight Receiver tab still shows them).
 *   - An axis with a held value counts as "pilot is using this axis", so the correction leaves it alone.
 *   - LoRa controller only (a CRSF radio has its own sticks/trims).
 * Careful: a held roll/pitch in ANGLE mode is a held tilt, so the drone keeps drifting that way.
 * A held yaw is a constant spin rate in every flight mode. */
#define STICK_HOLD_ENABLE         1            /* 1 = hold mode on, 0 = sticks return to centre (normal) */
#define STICK_HOLD_ROLL           1            /* per-axis: which axes hold when STICK_HOLD_ENABLE is 1 */
#define STICK_HOLD_PITCH          1
#define STICK_HOLD_YAW            1
#define STICK_HOLD_MIN_PUSH_US    100          /* a push must reach this far (us, before scaling; 500 = full stick) to count */

#if STICK_SCALE_ROLL_PCT < 0 || STICK_SCALE_ROLL_PCT > 100 || STICK_SCALE_PITCH_PCT < 0 || STICK_SCALE_PITCH_PCT > 100 || STICK_SCALE_YAW_PCT < 0 || STICK_SCALE_YAW_PCT > 100
#error STICK_SCALE_*_PCT must be 0..100
#endif

/* Buttons (called L3 / R3 in this file). Button 1 = L3, button 2 = R3. */
#define BTN_BYTE_OFF              7            /* byte holding the button bits */
#define BTN_L3_MASK               0x01         /* button 1 */
#define BTN_R3_MASK               0x02         /* button 2 */
#define BTN_PRESSED_IS_HIGH       1            /* sketch: pressed = bit set */

/* ---- 3. What each stick / button does (choose to taste, no packet knowledge needed) ---- */
enum { AX_LX = 0, AX_LY = 1, AX_RX = 2, AX_RY = 3 };
#define MAP_THROTTLE_AXIS         AX_LY        /* left stick up/down  */
#define MAP_YAW_AXIS              AX_LX        /* left stick left/right */
#define MAP_ROLL_AXIS             AX_RX        /* right stick left/right */
#define MAP_PITCH_AXIS            AX_RY        /* right stick up/down */
#define BTN_ARM_IS_L3             1            /* 1: L3 toggles ARM and R3 toggles correction; 0: swapped */

/* Game-style thumbsticks spring back to centre, so a centred throttle stick can't mean "hover at half power".
 * THROTTLE_INTEGRATE 1: pushing the stick up raises throttle, pulling down lowers it, centre holds it (like a trigger).
 * THROTTLE_INTEGRATE 0: the stick position IS the throttle (use only if the throttle axis does not spring back). */
#define THROTTLE_INTEGRATE        1
#define THROTTLE_RATE_US_PER_SEC  75          /* full stick deflection changes throttle by this many us per second */
#define THROTTLE_MAX_US           1500         /* hard cap on throttle from the controller. Raise slowly during testing */
#define THROTTLE_ARMED_START_US   1200         /* after arming, throttle jumps here so the stick works in the useful range.
                                                * Keep it below hover so the drone stays on the ground. 1000 = start from idle */
#define THROTTLE_ARM_HOLD_MS      500          /* throttle is held at 1000 this long after the ARM press, because Betaflight
                                                * refuses to arm while throttle is above min_check (default 1050) */
#if THROTTLE_ARMED_START_US >= THROTTLE_MAX_US || THROTTLE_MAX_US > 2000 || THROTTLE_ARMED_START_US < 1000
#error check THROTTLE_ARMED_START_US / THROTTLE_MAX_US (need 1000 <= start < max <= 2000)
#endif

/* Roll / pitch / yaw start values: what is sent to the FC right after arming (at the same moment the throttle
 * jumps to THROTTLE_ARMED_START_US). Values are what the FC receives, 1000..2000 us, so 1500 = centre (no input).
 *   - Works through STICK_HOLD: the value is loaded as the held value, so it stays until you change it with the
 *     stick (a push the same way replaces it, a push the opposite way returns that axis to centre).
 *   - Needs STICK_HOLD_ENABLE 1 and that axis's STICK_HOLD_<axis> 1; otherwise the stick would snap it back
 *     to centre straight away, so it is ignored.
 *   - Must be reachable with the stick at the current STICK_SCALE: within 1500 +/- 5 x STICK_SCALE_<axis>_PCT
 *     (at 20%: 1400..1600). The build stops if it isn't.
 *   - An axis with a non-centre start value counts as "pilot is using it", so the correction leaves that axis alone
 *     until it is back at centre.
 *   - Correction (roll/pitch bias) and the controller's live sticks still work on top as before. */
#define ARMED_START_ROLL_US       1500         /* 1500 = centre */
#define ARMED_START_PITCH_US      1500
#define ARMED_START_YAW_US        1500

/* start value (FC us) -> held stick value (before scaling) */
#define ARMED_START_HELD(us, pct) ((pct) ? (int16_t)((((int32_t)(us) - 1500) * 100) / (pct)) : (int16_t)0)
#if (ARMED_START_ROLL_US  - 1500) * 100 >  500 * STICK_SCALE_ROLL_PCT  || (ARMED_START_ROLL_US  - 1500) * 100 < -500 * STICK_SCALE_ROLL_PCT
#error ARMED_START_ROLL_US is out of reach at this STICK_SCALE_ROLL_PCT (limit 1500 +/- 5 x scale)
#endif
#if (ARMED_START_PITCH_US - 1500) * 100 >  500 * STICK_SCALE_PITCH_PCT || (ARMED_START_PITCH_US - 1500) * 100 < -500 * STICK_SCALE_PITCH_PCT
#error ARMED_START_PITCH_US is out of reach at this STICK_SCALE_PITCH_PCT (limit 1500 +/- 5 x scale)
#endif
#if (ARMED_START_YAW_US   - 1500) * 100 >  500 * STICK_SCALE_YAW_PCT   || (ARMED_START_YAW_US   - 1500) * 100 < -500 * STICK_SCALE_YAW_PCT
#error ARMED_START_YAW_US is out of reach at this STICK_SCALE_YAW_PCT (limit 1500 +/- 5 x scale)
#endif
#if !STICK_HOLD_ENABLE && (ARMED_START_ROLL_US != 1500 || ARMED_START_PITCH_US != 1500 || ARMED_START_YAW_US != 1500)
#warning ARMED_START_*_US are ignored because STICK_HOLD_ENABLE is 0
#endif


/* >>>>>> 1C. FC LINK AND SAFETY: MSP constants, loop timing, failsafe <<<<<< */
/* =====================  MSP / protocol constants  =========================== */
#define MSP_API_VERSION           1
#define MSP_STATUS                101
#define MSP_RAW_IMU               102
#define MSP_MOTOR                 104
#define MSP_ATTITUDE              108
#define MSP_SET_RAW_RC            200
#define MSP_SET_ARMING_DISABLED   239
#define MSP_RC                    105
#define MSP_ANALOG                110

/* ---- Extra FC readings: armed state (MSP_STATUS), the channels the FC received (MSP_RC), battery (MSP_ANALOG) ---- */
#define FC_EXTRA_POLL_ENABLE        1    /* 1 = read these three from the FC; 0 = switch them off */
#define FC_EXTRA_POLL_EVERY_N_LOOPS 3    /* one extra MSP request every N loops (20 ms each), cycling STATUS -> RC -> ANALOG, so each is
                                          * refreshed every 3*N loops (180 ms at N=3). Each request costs a few ms on the 115200 baud link:
                                          * watch loopWorkMsMax and loopOverrunCount, and raise N if the loop starts overrunning */
#define FC_RC_ECHO_TOLERANCE_US     3    /* the FC's echo of a channel may differ from what we sent by this much before it counts as a mismatch */
#define BATT_CUTOFF_ENABLE          0    /* 1 = force disarm and minimum throttle when the battery stays below BATT_CUTOFF_DECIV */
#define BATT_CUTOFF_DECIV           0    /* cutoff in 0.1 V units (180 = 18.0 V). Set it for your pack BEFORE enabling; 0 = never trips */
#define BATT_CUTOFF_READS           5    /* consecutive low readings needed before it trips (each reading is ~180 ms apart) */
#define RC_CHANNEL_COUNT          8

#define CRSF_SYNC_BYTE            0xC8
#define CRSF_FRAMETYPE_CHANNELS   0x16
#define CRSF_CHANNEL_COUNT        16

#define LOOP_PERIOD_MS            20    /* main loop pacing = 50 Hz */

/* =====================  Pass-through safety  ================================ */
#define PILOT_FAILSAFE_TIMEOUT_MS 500   /* link lost this long -> throttle min + disarm */

/* >>>>>> 1D. FLIGHT TUNING: level baseline and wind-correction gains <<<<<< */
/* =====================  Baseline (the "level" reference)  =================== */
#define ATTITUDE_DISTURBANCE_THRESHOLD_DECIDEG   60   /* 6 deg - used for status/stats only */
#define ATTITUDE_BASELINE_SAMPLES                10
#define BASELINE_SETTLE_MS                       3000 /* wait for the FC attitude estimate to settle after power-up */
#define BASELINE_STABLE_BAND_DECIDEG             15    /* samples must agree within 1.5 deg or we restart */

/* =====================  Wind-disturbance correction  ======================== */
#define CORRECTION_SWITCH_CHANNEL     5      /* rcChannels[5] = AUX2 */
#define CORRECTION_SWITCH_ON_US       1500   /* above this = correction ON */
#define CORRECTION_KP_US_PER_DECIDEG  0.8f   /* attitude term: us of stick per 0.1 deg of error */
#define CORRECTION_KD_US_PER_DPS      0.3f   /* rate term: us of stick per deg/s of body rotation */
#define CORRECTION_ATT_DEADBAND_DECIDEG 20   /* ignore attitude error under 2 deg */
#define CORRECTION_RATE_DEADBAND_DPS    15   /* ignore body rate under 15 deg/s */
#define GYRO_FILTER_PCT                 30   /* gyro low-pass (lower = smoother) */
#define CORRECTION_MAX_US             100    /* correction can move a stick at most +/-100 us */
#define CORRECTION_STICK_DEADBAND_US  40     /* pilot stick beyond +/-40 us of centre = pilot owns that axis (measured before STICK_SCALE) */
/* Sign conventions verified on the bench: tilt right -> roll bias negative, nose up -> pitch bias positive */
#define CORRECTION_ROLL_SIGN          (-1)
#define CORRECTION_PITCH_SIGN         (-1)
#define GYRO_ROLL_SIGN                (1)
#define GYRO_PITCH_SIGN               (1)

/* >>>>>> 1E. LEDS <<<<<< */
/* =====================  LEDs  =============================================== */
#define LED_LINK_PORT   GPIOB
#define LED_LINK_PIN    GPIO_PIN_5    /* yellow DS2: pilot link alive */
#define LED_CORR_PORT   GPIOB
#define LED_CORR_PIN    GPIO_PIN_6    /* red DS1: correction active */
#define LED_ON(port, pin)   HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define LED_OFF(port, pin)  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)

typedef struct {
  uint8_t direction;   /* '>' = ok, '!' = FC reported error */
  uint8_t size;
  uint8_t cmd;
  uint8_t payload[64];
} MspResponse;

/* One frame of pilot input, RC microseconds (1000-2000):
 * ch[0] roll, ch[1] pitch, ch[2] throttle, ch[3] yaw, ch[4] AUX1 arm,
 * ch[5] AUX2 correction toggle, ch[6] AUX3 (ANGLE), ch[7] AUX4 */
typedef struct {
  uint16_t ch[RC_CHANNEL_COUNT];
} PilotFrame;

typedef struct {
  uint32_t n;
  uint64_t sumSqRoll;
  uint64_t sumSqPitch;
  int16_t  peakRoll;
  int16_t  peakPitch;
} AttStats;

#ifdef FLIGHT_TEST_ENABLE
typedef enum { FT_WAIT_BASELINE = 0, FT_COUNTDOWN, FT_ARM, FT_RAMP_UP, FT_HOLD, FT_RAMP_DOWN, FT_DONE } FlightTestState;
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* Debug-visible globals: add these to Live Expressions while the program runs */
volatile uint8_t  correctionActive = 0;
volatile int16_t  biasRoll = 0, biasPitch = 0;
volatile int16_t  deltaRoll = 0, deltaPitch = 0;
volatile int16_t  gyroRoll = 0, gyroPitch = 0;
volatile int16_t  attRoll = 0, attPitch = 0;          /* live attitude, decidegrees */
volatile int16_t  attYaw = 0;                         /* live heading from the FC, whole degrees (0..359) */
volatile int16_t  baseRollDbg = 0, basePitchDbg = 0;  /* locked baseline, decidegrees */
/* The roll / pitch / yaw command currently in effect, BEFORE stick scaling, -500..+500 us from centre.
 * With STICK_HOLD on this is the held value (or the live stick while disarmed); with it off, the live stick. */
volatile int16_t  stickRawRollUs = 0, stickRawPitchUs = 0, stickRawYawUs = 0;
volatile uint8_t  rezeroBaseline = 0;                 /* set to 1 to re-capture the level baseline (hold flat and still) */
volatile uint8_t  statsReset = 0;                     /* set to 1 to clear the OFF/ON attitude statistics */
volatile uint16_t motorOut[4] = {0, 0, 0, 0};         /* motor outputs the FC is commanding, 1000-2000 */
volatile uint16_t txRollDbg = 1500, txPitchDbg = 1500, txThrottleDbg = 1000, txArmDbg = 1000; /* RC we send to the FC */
volatile uint16_t txYawDbg = 1500;                    /* yaw we send to the FC (after scaling) */
volatile uint32_t loopCount = 0;
volatile uint32_t attitudeFailCount = 0;
volatile uint32_t gyroFailCount = 0;
volatile uint32_t motorFailCount = 0;
volatile uint8_t  pilotLinkAlive = 0;
volatile uint32_t mspRxByteCount = 0;    /* any byte received from the FC on USART1 (0 = nothing arrives at all) */
volatile uint32_t mspReplyOkCount = 0;    /* complete, checksum-valid MSP replies received */
volatile uint8_t  mspRxLast[32];          /* the last 32 bytes received from the FC (view as hex in Live Expressions) */
volatile uint8_t  mspRxLastIdx = 0;       /* next write position in mspRxLast */
volatile uint8_t  pa10PullDownHigh = 0;   /* FC_PIN_TEST: of 100 samples with PA10 pulled DOWN, how many read high */
volatile uint8_t  pa10PullUpHigh = 0;     /* FC_PIN_TEST: of 100 samples with PA10 pulled UP, how many read high */
volatile uint32_t txTestFrames = 0;       /* FC_TX_TEST: RC frames sent so far */
volatile char     cliRx[64];              /* FC_CLI_TEST: what the FC sent back after the '#' (shows as text) */
volatile uint8_t  cliRxCount = 0;         /* FC_CLI_TEST: number of bytes received */
volatile uint8_t  cliDone = 0;            /* FC_CLI_TEST: 1 when the test has finished */
volatile uint8_t  pa9PullDownHigh = 0;    /* FC_PIN_TEST: same two readings for PA9 (J2 slot 3, the board's TX pin) */
volatile uint8_t  pa9PullUpHigh = 0;
volatile uint32_t pinTestPass = 0;        /* FC_PIN_TEST: completed passes */
volatile uint8_t  loraReady = 0;          /* 1 once the SX1276 answered and was configured */
volatile uint32_t loraPktCount = 0;       /* good packets received */
volatile uint32_t loraCrcErrCount = 0;    /* packets rejected (radio CRC, length, magic or our CRC-8) */
volatile uint32_t loraLostCount = 0;      /* packets skipped, from sequence-number gaps */
volatile int16_t  loraRssiDbm = 0;        /* RSSI of the last good packet */
#if INPUT_SOURCE == INPUT_LORA
SPI_HandleTypeDef hspi1;                  /* RFM95W */
#endif
#ifdef FLIGHT_TEST_ENABLE
volatile FlightTestState ftState = FT_WAIT_BASELINE;
volatile uint16_t ftThrottle = 1000;
#endif

static AttStats statsOff, statsOn;

/* Loop timing instrumentation */
volatile uint16_t loopWorkMs = 0;        /* last loop's work time in ms (excludes the pacing wait) */
volatile uint16_t loopWorkMsMax = 0;     /* worst loop since baseline lock */
volatile uint32_t loopOverrunCount = 0;  /* loops whose work exceeded LOOP_PERIOD_MS */
volatile uint32_t loopWorkSumMs = 0;     /* sum of work times since baseline lock */
volatile uint32_t loopWorkN = 0;         /* loops counted since baseline lock */

/* Extra FC readings (MSP_STATUS / MSP_RC / MSP_ANALOG): add these to Live Expressions */
volatile uint8_t  fcArmed = 0;                        /* the FC's own armed state (MSP_STATUS flight-mode bit 0) */
volatile uint16_t fcCycleTimeUs = 0;                  /* the FC's PID loop time in microseconds */
volatile uint16_t fcSensors = 0;                      /* MSP_STATUS sensor-present bits */
volatile uint32_t fcModeFlags = 0;                    /* MSP_STATUS flight-mode flags */
volatile uint16_t fcRc[RC_CHANNEL_COUNT] = {0};       /* the channels the FC says it received (MSP_RC) */
volatile uint32_t fcRcMismatchCount = 0;              /* echoes that did not match what we last sent */
volatile uint16_t fcVbatDeci = 0;                     /* battery voltage in 0.1 V (MSP_ANALOG) */
volatile uint16_t fcMahDrawn = 0;                     /* mAh drawn */
volatile int16_t  fcAmpsCenti = 0;                    /* current in 0.01 A (0 unless the FC has a current sensor set up) */
volatile uint32_t fcStatusOkCount = 0, fcStatusFailCount = 0;
volatile uint32_t fcRcOkCount = 0,     fcRcFailCount = 0;
volatile uint32_t fcAnalogOkCount = 0, fcAnalogFailCount = 0;
volatile uint8_t  battLowLatched = 0;                 /* 1 = the battery cutoff has tripped (clears only on reset) */
static uint16_t   lastSentRc[RC_CHANNEL_COUNT];       /* what we sent to the FC last loop, for the MSP_RC comparison */
static uint8_t    lastSentRcValid = 0;

static log_record_t g_log;
volatile logger_status_t sdLogStatus = LOG_ERR_NO_CARD;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ##########################################################################################
 *  PART 2  -  BOARD SETUP (custom board connection)
 *  Pin map is in the header comment. Order: debug printing, clock, USART1 -> FC (J2), USART6 -> debug (J7), LEDs.
 *  The LoRa SPI1 pins are set up inside lora_init() in PART 4.
 ########################################################################################## */

/* ---------------------------------------------------------------------------
 * Debug output on USART6 TX (J7). No-op when the debug UART is disabled.
 * ------------------------------------------------------------------------- */
static void dbg_printf(const char *fmt, ...)
{
#if DEBUG_UART_ENABLE
  char buf[200];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n > 0)
  {
    if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
    HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)n, 50);
  }
#else
  (void)fmt;
#endif
}


/* ##########################################################################################
 *  PART 3  -  FC LINK (MSP to Betaflight)
 *  Talks to the flight controller over USART1: sends RC channels, reads attitude, gyro and motor outputs.
 ########################################################################################## */

/* ---------------------------------------------------------------------------
 * MSP (MultiWii Serial Protocol) v1 helpers
 * ------------------------------------------------------------------------- */
/* Same as HAL_UART_Receive, but counts every byte that arrives (for the Live Expressions diagnostics) */
static HAL_StatusTypeDef msp_rx(UART_HandleTypeDef *huart, uint8_t *dst, uint16_t len, uint32_t timeout_ms)
{
  HAL_StatusTypeDef r = HAL_UART_Receive(huart, dst, len, timeout_ms);
  if (r == HAL_OK)
  {
    mspRxByteCount += len;
    for (uint16_t i = 0; i < len; i++)
    {
      mspRxLast[mspRxLastIdx & 31] = dst[i];
      mspRxLastIdx = (uint8_t)((mspRxLastIdx + 1) & 31);
    }
  }
  return r;
}

static void msp_send_request(UART_HandleTypeDef *huart, uint8_t cmd, uint8_t *data, uint8_t size)
{
  uint8_t header[5] = { '$', 'M', '<', size, cmd };
  uint8_t checksum = size ^ cmd;

  HAL_UART_Transmit(huart, header, 5, 100);

  if (size > 0 && data != NULL)
  {
    HAL_UART_Transmit(huart, data, size, 100);
    for (uint8_t i = 0; i < size; i++)
    {
      checksum ^= data[i];
    }
  }
  HAL_UART_Transmit(huart, &checksum, 1, 100);
}

static HAL_StatusTypeDef msp_read_response(UART_HandleTypeDef *huart, MspResponse *resp, uint32_t timeout_ms)
{
  uint8_t byte, checksum;
  uint16_t sync_attempts = 0;

  do
  {
    if (msp_rx(huart, &byte, 1, timeout_ms) != HAL_OK) return HAL_TIMEOUT;
    if (++sync_attempts > 64) return HAL_TIMEOUT;
  } while (byte != '$');

  if (msp_rx(huart, &byte, 1, timeout_ms) != HAL_OK || byte != 'M') return HAL_ERROR;
  if (msp_rx(huart, &resp->direction, 1, timeout_ms) != HAL_OK) return HAL_ERROR;
  if (msp_rx(huart, &resp->size, 1, timeout_ms) != HAL_OK) return HAL_ERROR;
  if (msp_rx(huart, &resp->cmd, 1, timeout_ms) != HAL_OK) return HAL_ERROR;

  if (resp->size > sizeof(resp->payload)) return HAL_ERROR;

  checksum = resp->size ^ resp->cmd;
  for (uint8_t i = 0; i < resp->size; i++)
  {
    if (msp_rx(huart, &resp->payload[i], 1, timeout_ms) != HAL_OK) return HAL_ERROR;
    checksum ^= resp->payload[i];
  }

  if (msp_rx(huart, &byte, 1, timeout_ms) != HAL_OK) return HAL_ERROR;
  if (byte != checksum) return HAL_ERROR;

  mspReplyOkCount++;
  return HAL_OK;
}

static void msp_flush_rx(UART_HandleTypeDef *huart)
{
  uint8_t dummy;
  /* An overrun/framing error left set can make later reads fail, so clear them first */
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);

  /* Discard stale bytes (e.g. late acks) so the next request starts on a clean frame boundary.
   * Bounded so a chatty line can never trap us here. */
  for (uint16_t i = 0; i < 256; i++)
  {
    if (msp_rx(huart, &dummy, 1, 2) != HAL_OK) break;
  }
}

static void msp_set_raw_rc(UART_HandleTypeDef *huart, uint16_t *channels, uint8_t count)
{
  uint8_t payload[RC_CHANNEL_COUNT * 2];
  MspResponse ackResp;

  for (uint8_t i = 0; i < count; i++)
  {
    payload[i * 2]     = (uint8_t)(channels[i] & 0xFF);
    payload[i * 2 + 1] = (uint8_t)(channels[i] >> 8);
  }

  msp_send_request(huart, MSP_SET_RAW_RC, payload, (uint8_t)(count * 2));
  /* Betaflight acks every SET_* command. Read it so it can't be mistaken for the next reply. */
  msp_read_response(huart, &ackResp, 20);

  /* Companion computers must keep granting arming permission (0 = allow arming) */
  uint8_t armPayload[2] = { 0, 0 };
  msp_send_request(huart, MSP_SET_ARMING_DISABLED, armPayload, sizeof(armPayload));
  msp_read_response(huart, &ackResp, 20);
}

static HAL_StatusTypeDef msp_get_attitude(UART_HandleTypeDef *huart, int16_t *roll, int16_t *pitch, int16_t *yaw)
{
  MspResponse resp;

  msp_flush_rx(huart);
  msp_send_request(huart, MSP_ATTITUDE, NULL, 0);

  if (msp_read_response(huart, &resp, 100) != HAL_OK) return HAL_ERROR;
  if (resp.direction != '>' || resp.cmd != MSP_ATTITUDE || resp.size < 6) return HAL_ERROR;

  /* roll/pitch are in 0.1-degree units (decidegrees), yaw is whole degrees */
  *roll  = (int16_t)(resp.payload[0] | (resp.payload[1] << 8));
  *pitch = (int16_t)(resp.payload[2] | (resp.payload[3] << 8));
  *yaw   = (int16_t)(resp.payload[4] | (resp.payload[5] << 8));

  return HAL_OK;
}

/* MSP_RAW_IMU payload: acc xyz, gyro xyz, mag xyz (int16 each). Gyro is deg/s: X = roll rate, Y = pitch rate */
static HAL_StatusTypeDef msp_get_gyro(UART_HandleTypeDef *huart, int16_t *gyroX, int16_t *gyroY)
{
  MspResponse resp;

  msp_flush_rx(huart);
  msp_send_request(huart, MSP_RAW_IMU, NULL, 0);

  if (msp_read_response(huart, &resp, 100) != HAL_OK) return HAL_ERROR;
  if (resp.direction != '>' || resp.cmd != MSP_RAW_IMU || resp.size < 18) return HAL_ERROR;

  *gyroX = (int16_t)(resp.payload[6] | (resp.payload[7] << 8));
  *gyroY = (int16_t)(resp.payload[8] | (resp.payload[9] << 8));

  return HAL_OK;
}

/* Motor outputs the FC is commanding (MSP_MOTOR: up to 8 x uint16, we keep the first 4) */
static HAL_StatusTypeDef msp_get_motors(UART_HandleTypeDef *huart, uint16_t *out4)
{
  MspResponse resp;

  msp_flush_rx(huart);
  msp_send_request(huart, MSP_MOTOR, NULL, 0);

  if (msp_read_response(huart, &resp, 100) != HAL_OK) return HAL_ERROR;
  if (resp.direction != '>' || resp.cmd != MSP_MOTOR || resp.size < 8) return HAL_ERROR;

  for (uint8_t i = 0; i < 4; i++)
  {
    out4[i] = (uint16_t)(resp.payload[i * 2] | (resp.payload[i * 2 + 1] << 8));
  }

  return HAL_OK;
}

/* MSP_STATUS: loop time, sensors and the flight-mode flags. Bit 0 of the flags is ARM (the FC's own armed state). */
static HAL_StatusTypeDef msp_get_status(UART_HandleTypeDef *huart, uint16_t *cycleUs, uint16_t *sensors, uint32_t *modeFlags)
{
  MspResponse resp;

  msp_flush_rx(huart);
  msp_send_request(huart, MSP_STATUS, NULL, 0);

  if (msp_read_response(huart, &resp, 100) != HAL_OK) return HAL_ERROR;
  if (resp.direction != '>' || resp.cmd != MSP_STATUS || resp.size < 10) return HAL_ERROR;

  *cycleUs   = (uint16_t)(resp.payload[0] | (resp.payload[1] << 8));
  *sensors   = (uint16_t)(resp.payload[4] | (resp.payload[5] << 8));
  *modeFlags = (uint32_t)resp.payload[6] | ((uint32_t)resp.payload[7] << 8) |
               ((uint32_t)resp.payload[8] << 16) | ((uint32_t)resp.payload[9] << 24);
  return HAL_OK;
}

/* MSP_RC: the channel values the FC currently holds (uint16 each). We keep the first RC_CHANNEL_COUNT. */
static HAL_StatusTypeDef msp_get_rc(UART_HandleTypeDef *huart, uint16_t *out)
{
  MspResponse resp;

  msp_flush_rx(huart);
  msp_send_request(huart, MSP_RC, NULL, 0);

  if (msp_read_response(huart, &resp, 100) != HAL_OK) return HAL_ERROR;
  if (resp.direction != '>' || resp.cmd != MSP_RC || resp.size < (RC_CHANNEL_COUNT * 2)) return HAL_ERROR;

  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++)
  {
    out[i] = (uint16_t)(resp.payload[i * 2] | (resp.payload[i * 2 + 1] << 8));
  }
  return HAL_OK;
}

/* MSP_ANALOG: battery voltage (uint8, 0.1 V), mAh drawn (uint16), RSSI (uint16), current (int16, 0.01 A) */
static HAL_StatusTypeDef msp_get_analog(UART_HandleTypeDef *huart, uint16_t *vbatDeci, uint16_t *mAh, int16_t *ampsCenti)
{
  MspResponse resp;

  msp_flush_rx(huart);
  msp_send_request(huart, MSP_ANALOG, NULL, 0);

  if (msp_read_response(huart, &resp, 100) != HAL_OK) return HAL_ERROR;
  if (resp.direction != '>' || resp.cmd != MSP_ANALOG || resp.size < 7) return HAL_ERROR;

  *vbatDeci  = resp.payload[0];
  *mAh       = (uint16_t)(resp.payload[1] | (resp.payload[2] << 8));
  *ampsCenti = (int16_t)(resp.payload[5] | (resp.payload[6] << 8));
  return HAL_OK;
}


/* ##########################################################################################
 *  PART 4  -  RECEIVER (controller -> this board)
 *  LoRa radio driver, packet decoding, ARM / correction button latches.
 *  Fill-in settings for this part are in PART 1B. Search for FILL IN.
 ########################################################################################## */

/* ---------------------------------------------------------------------------
 * Pilot input sources
 * ------------------------------------------------------------------------- */
#if INPUT_SOURCE == INPUT_LORA
#ifndef HAL_SPI_MODULE_ENABLED
#error "INPUT_LORA needs the HAL SPI module: uncomment #define HAL_SPI_MODULE_ENABLED in Core/Inc/stm32f4xx_hal_conf.h"
#endif

/* --- SX1276 registers (LoRa mode) --- */
#define SX_REG_FIFO               0x00
#define SX_REG_OPMODE             0x01
#define SX_REG_FRF_MSB            0x06
#define SX_REG_LNA                0x0C
#define SX_REG_FIFO_ADDR_PTR      0x0D
#define SX_REG_FIFO_RX_BASE       0x0F
#define SX_REG_FIFO_RX_CURRENT    0x10
#define SX_REG_IRQ_FLAGS          0x12
#define SX_REG_RX_NB_BYTES        0x13
#define SX_REG_PKT_RSSI           0x1A
#define SX_REG_MODEM_CONFIG1      0x1D
#define SX_REG_MODEM_CONFIG2      0x1E
#define SX_REG_PREAMBLE_MSB       0x20
#define SX_REG_MODEM_CONFIG3      0x26
#define SX_REG_DETECT_OPTIMIZE    0x31
#define SX_REG_DETECTION_THRESH   0x37
#define SX_REG_SYNC_WORD          0x39
#define SX_REG_VERSION            0x42
#define SX_MODE_LORA_SLEEP        0x80
#define SX_MODE_LORA_STDBY        0x81
#define SX_MODE_LORA_RX_CONT      0x85
#define SX_IRQ_RX_DONE            0x40
#define SX_IRQ_CRC_ERROR          0x20

#define LORA_NSS_LOW()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define LORA_NSS_HIGH()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

static void sx_write(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = { (uint8_t)(reg | 0x80), value };
  LORA_NSS_LOW();
  HAL_SPI_Transmit(&hspi1, tx, 2, 10);
  LORA_NSS_HIGH();
}

static uint8_t sx_read(uint8_t reg)
{
  uint8_t tx[2] = { (uint8_t)(reg & 0x7F), 0 };
  uint8_t rx[2] = { 0, 0 };
  LORA_NSS_LOW();
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
  LORA_NSS_HIGH();
  return rx[1];
}

static void sx_read_burst(uint8_t reg, uint8_t *dst, uint8_t len)
{
  uint8_t addr = (uint8_t)(reg & 0x7F);
  LORA_NSS_LOW();
  HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
  HAL_SPI_Receive(&hspi1, dst, len, 10);
  LORA_NSS_HIGH();
}

#if PKT_USE_CRC8
static uint8_t lora_crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
    {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}
#endif

/* Brings up SPI1 and the RFM95W in continuous-receive mode. Returns HAL_OK if the radio answered.
 * Pins: PA5 SCK, PA6 MISO, PA7 MOSI (AF5), PA4 NSS (software), PB0 RESET. DIO0 (PB1) is not needed: we poll the IRQ flags. */
static HAL_StatusTypeDef lora_init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

  g.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &g);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  g.Pin = GPIO_PIN_4;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &g);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  g.Pin = GPIO_PIN_0;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;   /* 100 MHz / 16 = 6.25 MHz (SX127x max 10 MHz) */
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) return HAL_ERROR;

  /* Hardware reset */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(10);

  uint8_t version = sx_read(SX_REG_VERSION);
  dbg_printf("LoRa: SX127x version register = 0x%02X (expect 0x12)\r\n", version);
  if (version != 0x12) return HAL_ERROR;

  sx_write(SX_REG_OPMODE, SX_MODE_LORA_SLEEP);   /* LoRa mode can only be selected from sleep */
  HAL_Delay(10);

  uint32_t frf = (uint32_t)(((uint64_t)LORA_FREQ_HZ << 19) / 32000000ULL);   /* 32 MHz radio crystal */
  sx_write(SX_REG_FRF_MSB,     (uint8_t)(frf >> 16));
  sx_write(SX_REG_FRF_MSB + 1, (uint8_t)(frf >> 8));
  sx_write(SX_REG_FRF_MSB + 2, (uint8_t)(frf));

  sx_write(SX_REG_LNA, 0x23);                                           /* max LNA gain, boost on */
  sx_write(SX_REG_MODEM_CONFIG1, (uint8_t)((LORA_BW_CODE << 4) | (LORA_CR_CODE << 1)));   /* explicit header */
  sx_write(SX_REG_MODEM_CONFIG2, (uint8_t)((LORA_SF << 4) | (LORA_PAYLOAD_CRC_ON ? 0x04 : 0x00)));
  sx_write(SX_REG_MODEM_CONFIG3, (uint8_t)(0x04 | ((LORA_SF >= 11 && LORA_BW_CODE == 7) ? 0x08 : 0x00)));   /* AGC auto, low-data-rate-optimise if needed */
  if (LORA_SF == 6)
  {
    sx_write(SX_REG_DETECT_OPTIMIZE, 0xC5);
    sx_write(SX_REG_DETECTION_THRESH, 0x0C);
  }
  else
  {
    sx_write(SX_REG_DETECT_OPTIMIZE, 0xC3);
    sx_write(SX_REG_DETECTION_THRESH, 0x0A);
  }
  sx_write(SX_REG_PREAMBLE_MSB,     (uint8_t)(LORA_PREAMBLE_LEN >> 8));
  sx_write(SX_REG_PREAMBLE_MSB + 1, (uint8_t)(LORA_PREAMBLE_LEN));
  sx_write(SX_REG_SYNC_WORD, LORA_SYNC_WORD);
  sx_write(SX_REG_FIFO_RX_BASE, 0x00);
  sx_write(SX_REG_FIFO_ADDR_PTR, 0x00);
  sx_write(SX_REG_IRQ_FLAGS, 0xFF);                                     /* clear all flags */
  sx_write(SX_REG_OPMODE, SX_MODE_LORA_RX_CONT);

  loraReady = 1;
  return HAL_OK;
}

volatile uint8_t  armLatched = 0;         /* ARM state toggled by the controller button */
volatile uint8_t  corrLatched = 0;        /* correction state toggled by the controller button */
volatile int16_t  stickUs[4] = {0, 0, 0, 0};   /* decoded stick offsets, -500..+500: LX, LY, RX, RY (live, before any hold) */
volatile int16_t  stickHeldUs[4] = {0, 0, 0, 0}; /* STICK_HOLD: value each axis is holding, -500..+500 (0 = centred) */

#if STICK_HOLD_ENABLE
/* One axis of stick hold. live = this packet's stick (-500..+500), returns the value to send. */
static int16_t stick_hold_axis(uint8_t axis, int16_t live)
{
  static uint8_t pushActive[4] = {0, 0, 0, 0};
  static int8_t  pushDir[4]    = {0, 0, 0, 0};   /* +1 / -1 = setting a value, 0 = this push is a "clear" push */
  int16_t held = stickHeldUs[axis];
  int16_t mag  = (int16_t)(live < 0 ? -live : live);

  if (mag < STICK_HOLD_MIN_PUSH_US)
  {
    pushActive[axis] = 0;                        /* stick (nearly) centred: keep whatever is held */
    return held;
  }

  int8_t dir = (live > 0) ? 1 : -1;
  if (!pushActive[axis])
  {
    /* A new push. Opposite to the held value = clear; otherwise it starts a new held value. */
    pushActive[axis] = 1;
    if (held != 0 && dir != ((held > 0) ? 1 : -1))
    {
      pushDir[axis] = 0;
      held = 0;
    }
    else
    {
      pushDir[axis] = dir;
      held = live;
    }
  }
  else if (pushDir[axis] == dir)
  {
    /* Same push, same direction: keep the furthest point reached */
    if (mag > (held < 0 ? -held : held)) held = live;
  }
  /* (a clear push, or a swing past centre without releasing, changes nothing more) */

  stickHeldUs[axis] = held;
  return held;
}

static void stick_hold_clear(void)
{
  for (uint8_t i = 0; i < 4; i++) stickHeldUs[i] = 0;
}

/* Called once per arming, when the throttle jumps to its start value: loads the roll/pitch/yaw start values */
static void stick_hold_load_armed_start(void)
{
#if STICK_HOLD_ROLL
  stickHeldUs[MAP_ROLL_AXIS]  = ARMED_START_HELD(ARMED_START_ROLL_US,  STICK_SCALE_ROLL_PCT);
#endif
#if STICK_HOLD_PITCH
  stickHeldUs[MAP_PITCH_AXIS] = ARMED_START_HELD(ARMED_START_PITCH_US, STICK_SCALE_PITCH_PCT);
#endif
#if STICK_HOLD_YAW
  stickHeldUs[MAP_YAW_AXIS]   = ARMED_START_HELD(ARMED_START_YAW_US,   STICK_SCALE_YAW_PCT);
#endif
}
#endif

static int32_t pkt_u(const uint8_t *buf, uint8_t off)
{
#if AXIS_BYTES == 1
#if AXIS_SIGNED
  return (int32_t)(int8_t)buf[off];
#else
  return (int32_t)buf[off];
#endif
#elif AXIS_BIG_ENDIAN
  return (int32_t)(uint16_t)((buf[off] << 8) | buf[off + 1]);
#else
  return (int32_t)(uint16_t)(buf[off] | (buf[off + 1] << 8));
#endif
}

/* raw stick value -> -500..+500 us around centre */
static int16_t stick_offset_us(int32_t raw, uint8_t invert)
{
  int32_t o;
  if (raw >= AXIS_RAW_CENTER) o = ((raw - AXIS_RAW_CENTER) * 500) / (int32_t)(AXIS_RAW_MAX - AXIS_RAW_CENTER);
  else                        o = -(((AXIS_RAW_CENTER - raw) * 500) / (int32_t)(AXIS_RAW_CENTER - AXIS_RAW_MIN));
  if (o > 500) o = 500;
  if (o < -500) o = -500;
  if (o < STICK_DEADBAND_US && o > -STICK_DEADBAND_US) o = 0;
  return (int16_t)(invert ? -o : o);
}

/* Turns one validated packet into the 8 RC channels. Also runs the ARM / correction button latches. */
static void lora_decode_packet(const uint8_t *buf, PilotFrame *frame)
{
  static uint8_t  haveBtn = 0, prevArmBtn = 0, prevCorrBtn = 0;
  static uint32_t lastPktTick = 0, lastThrTick = 0;
  static uint16_t thr = 1000;
  static uint32_t armPressTick = 0;
  static uint8_t  armStartDone = 0;     /* 1 once the throttle has jumped to THROTTLE_ARMED_START_US for this arming */

  uint32_t now = HAL_GetTick();

  /* After a link gap, forget the latches so the drone can never re-arm by itself when the link returns */
  if (lastPktTick != 0 && (now - lastPktTick) > PILOT_FAILSAFE_TIMEOUT_MS)
  {
    armLatched = 0;
    corrLatched = 0;
    thr = 1000;
    haveBtn = 0;
  }
  uint32_t dtMs = (lastPktTick == 0) ? 0 : (now - lastThrTick);
  lastPktTick = now;
  lastThrTick = now;

  stickUs[AX_LX] = stick_offset_us(pkt_u(buf, AXIS_LX_OFF), AXIS_LX_INVERT);
  stickUs[AX_LY] = stick_offset_us(pkt_u(buf, AXIS_LY_OFF), AXIS_LY_INVERT);
  stickUs[AX_RX] = stick_offset_us(pkt_u(buf, AXIS_RX_OFF), AXIS_RX_INVERT);
  stickUs[AX_RY] = stick_offset_us(pkt_u(buf, AXIS_RY_OFF), AXIS_RY_INVERT);

  /* Buttons: act on the press (rising edge) only, and ignore whatever state they were already in on the first packet */
  uint8_t l3 = ((buf[BTN_BYTE_OFF] & BTN_L3_MASK) != 0) == (BTN_PRESSED_IS_HIGH != 0);
  uint8_t r3 = ((buf[BTN_BYTE_OFF] & BTN_R3_MASK) != 0) == (BTN_PRESSED_IS_HIGH != 0);
  uint8_t armBtn  = BTN_ARM_IS_L3 ? l3 : r3;
  uint8_t corrBtn = BTN_ARM_IS_L3 ? r3 : l3;
  if (!haveBtn)
  {
    prevArmBtn = armBtn;
    prevCorrBtn = corrBtn;
    haveBtn = 1;
  }
  if (armBtn && !prevArmBtn)
  {
    armLatched = !armLatched;      /* a press while armed disarms (acts as a kill) */
    armPressTick = now;
    armStartDone = 0;
  }
  if (corrBtn && !prevCorrBtn) corrLatched = !corrLatched;
  prevArmBtn = armBtn;
  prevCorrBtn = corrBtn;

  /* Throttle */
  int16_t tOff = stickUs[MAP_THROTTLE_AXIS];
  if (!armLatched)
  {
    thr = 1000;
    armStartDone = 0;
  }
  else if ((now - armPressTick) < THROTTLE_ARM_HOLD_MS)
  {
    thr = 1000;                    /* let Betaflight see ARM with throttle low, or it refuses to arm */
  }
  else if (!armStartDone)
  {
    thr = THROTTLE_ARMED_START_US; /* jump once to the start value; the stick takes it from here */
    armStartDone = 1;
#if STICK_HOLD_ENABLE
    stick_hold_load_armed_start(); /* and roll / pitch / yaw to theirs (held until changed with the stick) */
#endif
  }
  else
  {
#if THROTTLE_INTEGRATE
    int32_t t = (int32_t)thr + ((int32_t)tOff * THROTTLE_RATE_US_PER_SEC * (int32_t)dtMs) / (500 * 1000);
#else
    int32_t t = 1500 + tOff;
#endif
    if (t < 1000) t = 1000;
    if (t > THROTTLE_MAX_US) t = THROTTLE_MAX_US;
    thr = (uint16_t)t;
  }

  int16_t rollUs  = stickUs[MAP_ROLL_AXIS];
  int16_t pitchUs = stickUs[MAP_PITCH_AXIS];
  int16_t yawUs   = stickUs[MAP_YAW_AXIS];
#if STICK_HOLD_ENABLE
  if (!armLatched)
  {
    stick_hold_clear();           /* disarmed (or link was lost): no holds, sticks pass through live */
  }
  else
  {
#if STICK_HOLD_ROLL
    rollUs  = stick_hold_axis(MAP_ROLL_AXIS,  rollUs);
#endif
#if STICK_HOLD_PITCH
    pitchUs = stick_hold_axis(MAP_PITCH_AXIS, pitchUs);
#endif
#if STICK_HOLD_YAW
    yawUs   = stick_hold_axis(MAP_YAW_AXIS,   yawUs);
#endif
  }
#endif

  frame->ch[0] = (uint16_t)(1500 + rollUs);
  frame->ch[1] = (uint16_t)(1500 + pitchUs);
  frame->ch[2] = thr;
  frame->ch[3] = (uint16_t)(1500 + yawUs);
  frame->ch[4] = armLatched  ? 2000 : 1000;    /* AUX1 = ARM */
  frame->ch[5] = corrLatched ? 2000 : 1000;    /* AUX2 = correction toggle */
  frame->ch[6] = 1000;                         /* AUX3 */
  frame->ch[7] = 1000;                         /* AUX4 */
}

/* Non-blocking: checks the radio once. HAL_OK only when a valid, new controller packet was decoded into frame->ch[]. */
static HAL_StatusTypeDef pilot_lora_poll(PilotFrame *frame)
{
  if (!loraReady) return HAL_TIMEOUT;

  uint8_t irq = sx_read(SX_REG_IRQ_FLAGS);
  if (!(irq & SX_IRQ_RX_DONE)) return HAL_TIMEOUT;

  sx_write(SX_REG_IRQ_FLAGS, 0xFF);   /* clear flags so the next packet can be signalled */

  if (irq & SX_IRQ_CRC_ERROR)
  {
    loraCrcErrCount++;
    return HAL_TIMEOUT;
  }

  uint8_t len = sx_read(SX_REG_RX_NB_BYTES);
  if (len != PKT_LEN)
  {
    loraCrcErrCount++;
    return HAL_TIMEOUT;
  }

  uint8_t buf[PKT_LEN];
  sx_write(SX_REG_FIFO_ADDR_PTR, sx_read(SX_REG_FIFO_RX_CURRENT));
  sx_read_burst(SX_REG_FIFO, buf, PKT_LEN);

#if PKT_USE_MAGIC
  if (buf[PKT_MAGIC_OFF] != PKT_MAGIC_VALUE) { loraCrcErrCount++; return HAL_TIMEOUT; }
#endif
#if PKT_USE_CRC8
  if (lora_crc8(buf, PKT_LEN - 1) != buf[PKT_LEN - 1]) { loraCrcErrCount++; return HAL_TIMEOUT; }
#endif
#if PKT_USE_SEQ
  {
    static uint8_t lastSeq = 0, haveSeq = 0;
    if (haveSeq)
    {
      uint8_t gap = (uint8_t)(buf[PKT_SEQ_OFF] - lastSeq - 1);
      if (gap > 0 && gap < 128) loraLostCount += gap;
    }
    lastSeq = buf[PKT_SEQ_OFF];
    haveSeq = 1;
  }
#endif

  lora_decode_packet(buf, frame);
  loraRssiDbm = (int16_t)sx_read(SX_REG_PKT_RSSI) - 157;   /* HF port formula */
  loraPktCount++;
  return HAL_OK;
}
#endif

#if INPUT_SOURCE == INPUT_CRSF
static uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
    {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static HAL_StatusTypeDef crsf_read_channels(UART_HandleTypeDef *huart, uint16_t *chOut)
{
  uint8_t syncByte, len, frameType, payload[22], crc;

  if (HAL_UART_Receive(huart, &syncByte, 1, 5) != HAL_OK) return HAL_TIMEOUT;
  if (syncByte != CRSF_SYNC_BYTE) return HAL_ERROR;

  if (HAL_UART_Receive(huart, &len, 1, 5) != HAL_OK) return HAL_ERROR;
  if (HAL_UART_Receive(huart, &frameType, 1, 5) != HAL_OK) return HAL_ERROR;
  if (frameType != CRSF_FRAMETYPE_CHANNELS) return HAL_ERROR;

  uint8_t payloadLen = (uint8_t)(len - 2);
  if (payloadLen != 22) return HAL_ERROR;

  if (HAL_UART_Receive(huart, payload, payloadLen, 10) != HAL_OK) return HAL_ERROR;
  if (HAL_UART_Receive(huart, &crc, 1, 5) != HAL_OK) return HAL_ERROR;

  uint8_t crcData[23];
  crcData[0] = frameType;
  memcpy(&crcData[1], payload, payloadLen);
  if (crsf_crc8(crcData, (uint8_t)(payloadLen + 1)) != crc) return HAL_ERROR;

  uint32_t bitBuffer = 0;
  uint8_t bitCount = 0;
  uint8_t byteIndex = 0;

  for (uint8_t ch = 0; ch < CRSF_CHANNEL_COUNT; ch++)
  {
    while (bitCount < 11)
    {
      bitBuffer |= ((uint32_t)payload[byteIndex++]) << bitCount;
      bitCount += 8;
    }
    chOut[ch] = (uint16_t)(bitBuffer & 0x7FF);
    bitBuffer >>= 11;
    bitCount -= 11;
  }

  return HAL_OK;
}

static uint16_t crsf_to_rc(uint16_t crsfVal)
{
  /* CRSF raw range ~172-1811 maps to RC 1000-2000 */
  return (uint16_t)(((crsfVal - 172) * (2000 - 1000)) / (1811 - 172) + 1000);
}
#endif

/* One call per main-loop pass. Returns HAL_OK when a fresh frame arrived. */
static HAL_StatusTypeDef pilot_input_poll(PilotFrame *frame)
{
#if INPUT_SOURCE == INPUT_LORA
  return pilot_lora_poll(frame);
#elif INPUT_SOURCE == INPUT_CRSF
  uint16_t raw[CRSF_CHANNEL_COUNT];
  if (crsf_read_channels(&huart6, raw) != HAL_OK) return HAL_TIMEOUT;
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++)
  {
    frame->ch[i] = crsf_to_rc(raw[i]);
  }
  return HAL_OK;
#else
  (void)frame;
  return HAL_TIMEOUT;
#endif
}


/* ##########################################################################################
 *  PART 5  -  CORRECTION (wind-disturbance rejection)
 *  The math that turns attitude error and body rotation into a roll/pitch bias, plus the statistics for your results.
 ########################################################################################## */

/* ---------------------------------------------------------------------------
 * Correction math
 * ------------------------------------------------------------------------- */
static int16_t apply_deadband(int16_t value, int16_t deadband)
{
  if (value > deadband)  return (int16_t)(value - deadband);
  if (value < -deadband) return (int16_t)(value + deadband);
  return 0;
}

static int16_t clamp_i16(int32_t value, int16_t limit)
{
  if (value > limit)  return limit;
  if (value < -limit) return (int16_t)(-limit);
  return (int16_t)value;
}

static uint16_t clamp_rc(int32_t value)
{
  if (value < 1000) return 1000;
  if (value > 2000) return 2000;
  return (uint16_t)value;
}

/* Shrinks a stick channel around centre: 1500 + (ch - 1500) * pct / 100 */
static uint16_t scale_stick(uint16_t ch, int32_t pct)
{
  return clamp_rc(1500 + (((int32_t)ch - 1500) * pct) / 100);
}

/* Expected-vs-actual comparison -> correction bias in RC microseconds.
 * Expected (sticks centred) = the locked level baseline and zero rotation.
 * Actual = live attitude and gyro rate.
 * bias = sign * ( Kp * attitude error + Kd * body rate ), deadbanded and clamped. */
static int16_t compute_correction_bias(int16_t attDeltaDecideg, int16_t gyroDps, int8_t attSign, int8_t gyroSign)
{
  float attTerm  = CORRECTION_KP_US_PER_DECIDEG * (float)apply_deadband(attDeltaDecideg, CORRECTION_ATT_DEADBAND_DECIDEG);
  float rateTerm = CORRECTION_KD_US_PER_DPS     * (float)apply_deadband(gyroDps, CORRECTION_RATE_DEADBAND_DPS);

  float bias = (float)attSign * attTerm + (float)attSign * (float)gyroSign * rateTerm;
  return clamp_i16((int32_t)bias, CORRECTION_MAX_US);
}

/* ---------------------------------------------------------------------------
 * Attitude statistics: how far the drone strays from level, correction OFF vs ON
 * ------------------------------------------------------------------------- */
static void stats_add(AttStats *s, int16_t dRoll, int16_t dPitch)
{
  int16_t ar = (int16_t)(dRoll  < 0 ? -dRoll  : dRoll);
  int16_t ap = (int16_t)(dPitch < 0 ? -dPitch : dPitch);

  s->n++;
  s->sumSqRoll  += (uint64_t)((int32_t)dRoll  * (int32_t)dRoll);
  s->sumSqPitch += (uint64_t)((int32_t)dPitch * (int32_t)dPitch);
  if (ar > s->peakRoll)  s->peakRoll  = ar;
  if (ap > s->peakPitch) s->peakPitch = ap;
}

static int32_t stats_rms(uint64_t sumSq, uint32_t n)
{
  if (n == 0) return 0;
  return (int32_t)sqrtf((float)sumSq / (float)n);
}

static void stats_print(void)
{
  dbg_printf("STATS (0.1 deg) OFF: n=%lu rms roll=%ld pitch=%ld peak roll=%d pitch=%d | ON: n=%lu rms roll=%ld pitch=%ld peak roll=%d pitch=%d\r\n",
             (unsigned long)statsOff.n, (long)stats_rms(statsOff.sumSqRoll, statsOff.n), (long)stats_rms(statsOff.sumSqPitch, statsOff.n),
             statsOff.peakRoll, statsOff.peakPitch,
             (unsigned long)statsOn.n, (long)stats_rms(statsOn.sumSqRoll, statsOn.n), (long)stats_rms(statsOn.sumSqPitch, statsOn.n),
             statsOn.peakRoll, statsOn.peakPitch);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  LED_OFF(LED_LINK_PORT, LED_LINK_PIN);
  LED_OFF(LED_CORR_PORT, LED_CORR_PIN);

#if FC_PIN_TEST
  /* Wiring test. PA10 is the board's RX pin (J2 slot 4, should be wired to the FC pad T2) and PA9 is the board's
   * TX pin (J2 slot 3, should be wired to the FC pad R2). Each is turned into a plain input, first with an internal
   * pull-DOWN and then with a pull-UP, and the level is sampled 100 times each way.
   *   FC drives the line high   : pull-down reads ~100, pull-up reads ~100
   *   nothing drives the line   : pull-down reads   0, pull-up reads ~100   (wire broken / FC port not active)
   *   FC (or a short) holds low : pull-down reads   0, pull-up reads   0
   * Expected with correct wiring: PA10 reads 100/100 (FC drives it), PA9 reads 0/100 (FC's RX is an input).
   * Runs forever, so the normal firmware never starts in this mode. */
  {
    GPIO_InitTypeDef pt = {0};
    pt.Mode = GPIO_MODE_INPUT;
    pt.Speed = GPIO_SPEED_FREQ_LOW;
    for (;;)
    {
      uint8_t n, c;
      pt.Pin = GPIO_PIN_10;
      pt.Pull = GPIO_PULLDOWN;  HAL_GPIO_Init(GPIOA, &pt);  HAL_Delay(20);
      for (n = 0, c = 0; n < 100; n++) { if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) c++; HAL_Delay(1); }
      pa10PullDownHigh = c;
      pt.Pull = GPIO_PULLUP;    HAL_GPIO_Init(GPIOA, &pt);  HAL_Delay(20);
      for (n = 0, c = 0; n < 100; n++) { if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) c++; HAL_Delay(1); }
      pa10PullUpHigh = c;

      pt.Pin = GPIO_PIN_9;
      pt.Pull = GPIO_PULLDOWN;  HAL_GPIO_Init(GPIOA, &pt);  HAL_Delay(20);
      for (n = 0, c = 0; n < 100; n++) { if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET) c++; HAL_Delay(1); }
      pa9PullDownHigh = c;
      pt.Pull = GPIO_PULLUP;    HAL_GPIO_Init(GPIOA, &pt);  HAL_Delay(20);
      for (n = 0, c = 0; n < 100; n++) { if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET) c++; HAL_Delay(1); }
      pa9PullUpHigh = c;

      pinTestPass++;
    }
  }
#endif

#if FC_HELLO_TEST
  {
    static const uint8_t hello[] = "HELLO FROM COMPANION BOARD\r\n";
    HAL_Delay(1000);
    for (;;)
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)hello, (uint16_t)(sizeof(hello) - 1), 100);
      HAL_Delay(200);
    }
  }
#endif

#if FC_INV_TEST
  /* Inverted-polarity round trip. PA9 becomes a plain output and sends "#" the way a UART with INVERTED lines would:
   * idle = LOW, start bit = HIGH, a 1 bit = LOW, a 0 bit = HIGH, stop bit = LOW. The reply is read normally on PA10
   * (the FC's TX is not inverted). Timing comes from the DWT cycle counter; interrupts are off during each byte. */
  {
    GPIO_InitTypeDef bb = {0};
    const uint8_t msg[7] = { '#', 'e', 'x', 'i', 't', '\r', '\n' };
    uint8_t b, n = 0, k;
    uint32_t bitCycles = SystemCoreClock / 115200U;

    bb.Pin = GPIO_PIN_9;
    bb.Mode = GPIO_MODE_OUTPUT_PP;
    bb.Pull = GPIO_NOPULL;
    bb.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &bb);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);       /* inverted idle = low */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    HAL_Delay(3000);                                           /* let the FC finish booting */
    while (HAL_UART_Receive(&huart1, &b, 1, 2) == HAL_OK) { }   /* drop anything already waiting */

    for (k = 0; k < 7; k++)
    {
      uint8_t v = msg[k], i;
      uint32_t t;
      __disable_irq();
      t = DWT->CYCCNT;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);       /* start bit (logic 0, inverted = high) */
      t += bitCycles; while ((int32_t)(DWT->CYCCNT - t) < 0) { }
      for (i = 0; i < 8; i++)
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (v & 1) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        v >>= 1;
        t += bitCycles; while ((int32_t)(DWT->CYCCNT - t) < 0) { }
      }
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);     /* stop bit (logic 1, inverted = low) */
      t += 2U * bitCycles; while ((int32_t)(DWT->CYCCNT - t) < 0) { }
      __enable_irq();
      if (k == 0)
      {
        /* listen for the banner right after the '#', before the "exit" goes out */
        while (n < (uint8_t)(sizeof(cliRx) - 1))
        {
          if (HAL_UART_Receive(&huart1, &b, 1, 300) != HAL_OK) break;
          cliRx[n++] = (char)b;
          cliRxCount = n;
        }
        cliRx[n] = 0;
      }
    }
    cliDone = 1;
    for (;;) { HAL_Delay(1000); }
  }
#endif

#if FC_TX_TEST
  /* One-way test: only the board -> FC direction. Streams SET_RAW_RC with values the FC's default can never match.
   * Any byte the FC sends back (it acks every SET command) is captured in mspRxLast, but nothing waits for it. */
  {
    uint16_t testCh[RC_CHANNEL_COUNT] = { 1700, 1300, 1000, 1600, 1000, 1000, 1000, 1000 };   /* R P T Y A1..A4 */
    uint8_t payload[RC_CHANNEL_COUNT * 2];
    uint8_t d;
    for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++)
    {
      payload[i * 2]     = (uint8_t)(testCh[i] & 0xFF);
      payload[i * 2 + 1] = (uint8_t)(testCh[i] >> 8);
    }
    HAL_Delay(3000);                              /* let the FC finish booting */
    for (;;)
    {
      msp_send_request(&huart1, MSP_SET_RAW_RC, payload, (uint8_t)(RC_CHANNEL_COUNT * 2));
      txTestFrames++;
      for (d = 0; d < 16; d++)                    /* capture any ack, without ever blocking for long */
      {
        uint8_t b;
        if (msp_rx(&huart1, &b, 1, 2) != HAL_OK) break;
      }
      HAL_Delay(15);
    }
  }
#endif

#if FC_CLI_TEST
  /* Round-trip test. Betaflight treats a '#' on an MSP-enabled serial port as "enter the CLI" and answers with
   * "Entering CLI Mode, type 'exit' to reboot, or 'help'" followed by a "# " prompt.
   *   Text comes back : our TX reaches the FC AND the FC's TX reaches us -> the wiring is fine
   *   Nothing comes back : the FC is not hearing us (TX wire / R2 pad) or its UART2 is not serving MSP */
  {
    uint8_t hash = '#', b, n = 0;
    uint8_t i;
    HAL_Delay(3000);                              /* let the FC finish booting */
    while (HAL_UART_Receive(&huart1, &b, 1, 2) == HAL_OK) { }   /* drop anything already waiting */
    HAL_UART_Transmit(&huart1, &hash, 1, 100);
    while (n < (uint8_t)(sizeof(cliRx) - 1))
    {
      if (HAL_UART_Receive(&huart1, &b, 1, 300) != HAL_OK) break;
      cliRx[n++] = (char)b;
      cliRxCount = n;
    }
    cliRx[n] = 0;
    /* leave the CLI again so the FC reboots back to normal MSP operation */
    {
      const uint8_t ex[6] = { 'e', 'x', 'i', 't', '\r', '\n' };
      for (i = 0; i < 6; i++) HAL_UART_Transmit(&huart1, (uint8_t *)&ex[i], 1, 100);
    }
    cliDone = 1;
    for (;;) { HAL_Delay(1000); }
  }
#endif

  dbg_printf("\r\n--- Companion board boot, starting MSP handshake ---\r\n");

#if INPUT_SOURCE == INPUT_LORA
  if (lora_init() == HAL_OK)
  {
    dbg_printf("LoRa radio OK, listening on %lu Hz\r\n", (unsigned long)LORA_FREQ_HZ);
  }
  else
  {
    dbg_printf("LoRa radio FAILED - check SPI1 wiring (PA4/5/6/7, PB0 reset) and the RFM95W 3.3 V supply\r\n");
  }
#endif

#if SD_LOG_ENABLE
  sdLogStatus = Logger_Init();
  dbg_printf("SD log: status=%d file=%s\r\n", (int)sdLogStatus, Logger_FileName());
#endif

  /* Give the FC time to boot, then check the MSP link */
  HAL_Delay(3000);

  MspResponse resp;
  msp_send_request(&huart1, MSP_API_VERSION, NULL, 0);
  if (msp_read_response(&huart1, &resp, 500) == HAL_OK && resp.direction == '>')
  {
    dbg_printf("MSP handshake OK\r\n");
  }
  else
  {
    dbg_printf("MSP handshake FAILED - check J2 wiring and the Ports tab (MSP on the matching UART, 115200)\r\n");
    LED_ON(LED_LINK_PORT, LED_LINK_PIN);   /* both LEDs on for a moment = handshake failed */
    LED_ON(LED_CORR_PORT, LED_CORR_PIN);
    HAL_Delay(1000);
    LED_OFF(LED_LINK_PORT, LED_LINK_PIN);
    LED_OFF(LED_CORR_PORT, LED_CORR_PIN);
  }
  HAL_Delay(1000);

  /* Channel order: Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4 */
  uint16_t rcChannels[RC_CHANNEL_COUNT] = {
    1500, /* Roll     - centred */
    1500, /* Pitch    - centred */
    1000, /* Throttle - minimum */
    1500, /* Yaw      - centred */
    1000, /* AUX1     - LOW (disarmed) so the FC sees a real off->on edge */
    1000, /* AUX2     - correction off */
    1000, /* AUX3 */
    1000  /* AUX4 */
  };

  /* Keep the FC link alive and disarmed for a moment */
  for (uint8_t i = 0; i < 20; i++)
  {
    msp_set_raw_rc(&huart1, rcChannels, RC_CHANNEL_COUNT);
    HAL_Delay(50);
  }

#if AUTO_ARM_AT_BOOT
  /* Bench only (props off!): raise AUX1 into the ARM range ourselves */
  dbg_printf("AUTO_ARM_AT_BOOT: raising AUX1 to arm...\r\n");
  rcChannels[4] = 2000;
  for (uint8_t i = 0; i < 40; i++)
  {
    msp_set_raw_rc(&huart1, rcChannels, RC_CHANNEL_COUNT);
    HAL_Delay(50);
  }
#else
  dbg_printf("Pass-through mode: arm with the controller's ARM switch.\r\n");
#endif

  dbg_printf("Entering main loop...\r\n");

  PilotFrame pilot;
  uint16_t txChannels[RC_CHANNEL_COUNT];   /* what goes to the FC = pilot input + correction */
  uint32_t lastPilotTick = 0;
  uint8_t  pilotSeen = 0;
  uint16_t pilotAux1 = 1000;               /* the pilot's ARM switch, before any test override */
  (void)pilotAux1;
#ifdef FLIGHT_TEST_ENABLE
  uint32_t ftStateStart = 0;
#endif

  int16_t baselineRoll = 0, baselinePitch = 0;
  int32_t baselineRollSum = 0, baselinePitchSum = 0;
  uint8_t baselineSampleCount = 0;
  uint8_t baselineCaptured = 0;
  int16_t rollMin = 0, rollMax = 0, pitchMin = 0, pitchMax = 0;
  uint32_t debugLoopCounter = 0;
  uint32_t lastStatsPrint = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t loopStart = HAL_GetTick();
    debugLoopCounter++;
    loopCount++;

    if (rezeroBaseline)
    {
      rezeroBaseline = 0;
      baselineCaptured = 0;
      baselineSampleCount = 0;
      biasRoll = 0;
      biasPitch = 0;
      dbg_printf("Re-zero requested: hold the board flat and still...\r\n");
    }
    if (statsReset)
    {
      statsReset = 0;
      memset(&statsOff, 0, sizeof(statsOff));
      memset(&statsOn, 0, sizeof(statsOn));
      dbg_printf("Stats cleared\r\n");
    }

    /* ---- 1. Pilot input ---------------------------------------------------- */
    if (pilot_input_poll(&pilot) == HAL_OK)
    {
      for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++)
      {
        rcChannels[i] = pilot.ch[i];
      }
      /* Remember the real stick position for the correction's "pilot owns this axis" test, then scale it down */
      stickRawRollUs  = (int16_t)((int32_t)rcChannels[0] - 1500);
      stickRawPitchUs = (int16_t)((int32_t)rcChannels[1] - 1500);
      stickRawYawUs   = (int16_t)((int32_t)rcChannels[3] - 1500);
      rcChannels[0] = scale_stick(rcChannels[0], STICK_SCALE_ROLL_PCT);
      rcChannels[1] = scale_stick(rcChannels[1], STICK_SCALE_PITCH_PCT);
      rcChannels[3] = scale_stick(rcChannels[3], STICK_SCALE_YAW_PCT);
      pilotAux1 = rcChannels[4];
      lastPilotTick = HAL_GetTick();
      pilotSeen = 1;
    }

    pilotLinkAlive = (pilotSeen && (HAL_GetTick() - lastPilotTick) <= PILOT_FAILSAFE_TIMEOUT_MS);
    if (pilotLinkAlive) LED_ON(LED_LINK_PORT, LED_LINK_PIN); else LED_OFF(LED_LINK_PORT, LED_LINK_PIN);

#ifdef FLIGHT_TEST_ENABLE
    /* ---- 2. Flight test (strapped-down thrust check) ------------------------ */
    {
      uint32_t now = HAL_GetTick();

      /* Kill switch: if a pilot link is up and its ARM switch is low, abort immediately */
      if (ftState >= FT_ARM && ftState < FT_DONE && pilotSeen && pilotAux1 < 1500)
      {
        dbg_printf("FLIGHT TEST: ABORTED by the ARM switch\r\n");
        ftState = FT_DONE;
      }

      switch (ftState)
      {
        case FT_WAIT_BASELINE:
          if (baselineCaptured)
          {
            ftStateStart = now;
            ftState = FT_COUNTDOWN;
            dbg_printf("FLIGHT TEST: starting in %lu s - get clear!\r\n", (unsigned long)(FLIGHT_TEST_START_DELAY_MS / 1000));
          }
          break;

        case FT_COUNTDOWN:
          if ((now - ftStateStart) >= FLIGHT_TEST_START_DELAY_MS)
          {
            ftStateStart = now;
            ftThrottle = 1000;
            ftState = FT_ARM;
            dbg_printf("FLIGHT TEST: arming\r\n");
          }
          break;

        case FT_ARM:
          rcChannels[4] = 2000;
          rcChannels[2] = 1000;
          if ((now - ftStateStart) >= 1000)
          {
            ftStateStart = now;
            ftState = FT_RAMP_UP;
            dbg_printf("FLIGHT TEST: ramping up to %u\r\n", (unsigned)FLIGHT_TEST_THROTTLE_US);
          }
          break;

        case FT_RAMP_UP:
        {
          uint32_t rise = ((now - ftStateStart) * FLIGHT_TEST_RAMP_US_PER_SEC) / 1000;
          uint32_t target = 1000 + rise;
          if (target >= FLIGHT_TEST_THROTTLE_US)
          {
            target = FLIGHT_TEST_THROTTLE_US;
            ftStateStart = now;
            ftState = FT_HOLD;
            dbg_printf("FLIGHT TEST: holding\r\n");
          }
          ftThrottle = (uint16_t)target;
          rcChannels[4] = 2000;
          rcChannels[2] = ftThrottle;
          break;
        }

        case FT_HOLD:
          ftThrottle = FLIGHT_TEST_THROTTLE_US;
          rcChannels[4] = 2000;
          rcChannels[2] = ftThrottle;
          if ((now - ftStateStart) >= FLIGHT_TEST_HOLD_MS)
          {
            ftStateStart = now;
            ftState = FT_RAMP_DOWN;
            dbg_printf("FLIGHT TEST: ramping down\r\n");
          }
          break;

        case FT_RAMP_DOWN:
        {
          uint32_t drop = ((now - ftStateStart) * FLIGHT_TEST_RAMP_US_PER_SEC) / 1000;
          uint32_t span = FLIGHT_TEST_THROTTLE_US - 1000;
          uint32_t target = (drop >= span) ? 1000 : (FLIGHT_TEST_THROTTLE_US - drop);
          ftThrottle = (uint16_t)target;
          rcChannels[4] = 2000;
          rcChannels[2] = ftThrottle;
          if (target <= 1000)
          {
            ftState = FT_DONE;
            dbg_printf("FLIGHT TEST: done, disarmed\r\n");
          }
          break;
        }

        case FT_DONE:
        default:
          ftThrottle = 1000;
          rcChannels[2] = 1000;
          rcChannels[4] = 1000;   /* disarmed */
          break;
      }

#if FLIGHT_TEST_ANGLE_AUX_CHANNEL >= 0
      if (ftState >= FT_ARM && ftState < FT_DONE)
      {
        rcChannels[FLIGHT_TEST_ANGLE_AUX_CHANNEL] = 2000;   /* ask Betaflight for ANGLE (self-level) mode */
      }
#endif
    }
#endif /* FLIGHT_TEST_ENABLE */

    /* ---- 3. Failsafe: once a pilot link has been seen, losing it cuts throttle and disarms ---- */
    if (pilotSeen && !pilotLinkAlive)
    {
      rcChannels[2] = 1000;
      rcChannels[4] = 1000;
    }

    /* Pilot input is passed through untouched unless the correction adds a bias below */
    memcpy(txChannels, rcChannels, sizeof(txChannels));

    /* ---- 4. Read gyro and attitude from the FC ----------------------------- */
    int16_t roll, pitch, yaw;
    int16_t gx = 0, gy = 0;
    if (msp_get_gyro(&huart1, &gx, &gy) == HAL_OK)
    {
      /* low-pass filter so hand shake / sensor noise doesn't become big bias swings */
      int16_t gyroRollNew  = (int16_t)(GYRO_ROLL_SIGN  * gx);
      int16_t gyroPitchNew = (int16_t)(GYRO_PITCH_SIGN * gy);
      gyroRoll  = (int16_t)(gyroRoll  + ((gyroRollNew  - gyroRoll)  * GYRO_FILTER_PCT) / 100);
      gyroPitch = (int16_t)(gyroPitch + ((gyroPitchNew - gyroPitch) * GYRO_FILTER_PCT) / 100);
    }
    else
    {
      gyroRoll = 0;
      gyroPitch = 0;
      gyroFailCount++;
    }

    if (msp_get_attitude(&huart1, &roll, &pitch, &yaw) == HAL_OK)
    {
      attRoll  = roll;
      attPitch = pitch;
      attYaw   = yaw;

      if (!baselineCaptured)
      {
        /* Take the "level" baseline once the FC has settled and the board is held still */
        if (HAL_GetTick() < BASELINE_SETTLE_MS)
        {
          if ((debugLoopCounter % STATUS_PRINT_EVERY_N_LOOPS) == 0)
          {
            dbg_printf("Waiting for the FC to settle before the baseline... roll=%d pitch=%d\r\n", roll, pitch);
          }
        }
        else
        {
          if (baselineSampleCount == 0)
          {
            baselineRollSum = 0;
            baselinePitchSum = 0;
            rollMin = rollMax = roll;
            pitchMin = pitchMax = pitch;
          }
          if (roll  < rollMin)  rollMin  = roll;
          if (roll  > rollMax)  rollMax  = roll;
          if (pitch < pitchMin) pitchMin = pitch;
          if (pitch > pitchMax) pitchMax = pitch;

          if ((rollMax - rollMin) > BASELINE_STABLE_BAND_DECIDEG || (pitchMax - pitchMin) > BASELINE_STABLE_BAND_DECIDEG)
          {
            dbg_printf("Baseline unsettled (roll %d..%d pitch %d..%d) - restarting, hold still and level\r\n",
                       rollMin, rollMax, pitchMin, pitchMax);
            baselineSampleCount = 0;
          }
          else
          {
            baselineRollSum  += roll;
            baselinePitchSum += pitch;
            baselineSampleCount++;

            if (baselineSampleCount >= ATTITUDE_BASELINE_SAMPLES)
            {
              baselineRoll  = (int16_t)(baselineRollSum  / baselineSampleCount);
              baselinePitch = (int16_t)(baselinePitchSum / baselineSampleCount);
              baselineCaptured = 1;
              baseRollDbg  = baselineRoll;
              basePitchDbg = baselinePitch;
              dbg_printf("BASELINE LOCKED: roll=%d pitch=%d\r\n", baselineRoll, baselinePitch);
            }
          }
        }
      }
      else
      {
        deltaRoll  = (int16_t)(roll  - baselineRoll);
        deltaPitch = (int16_t)(pitch - baselinePitch);

        /* Correction is ON when the controller's AUX2 switch is high (or forced on for bench testing) */
        correctionActive = (CORRECTION_ALWAYS_ON || rcChannels[CORRECTION_SWITCH_CHANNEL] > CORRECTION_SWITCH_ON_US);

        biasRoll  = 0;
        biasPitch = 0;
        if (correctionActive)
        {
          /* The pilot owns an axis whenever its stick is off-centre; correction only acts on a neutral stick.
           * Uses the stick position BEFORE scaling, so the threshold is the same physical movement at any STICK_SCALE. */
          int16_t rollStick  = stickRawRollUs;
          int16_t pitchStick = stickRawPitchUs;

          if (rollStick < CORRECTION_STICK_DEADBAND_US && rollStick > -CORRECTION_STICK_DEADBAND_US)
          {
            biasRoll = compute_correction_bias(deltaRoll, gyroRoll, CORRECTION_ROLL_SIGN, 1);
          }
          if (pitchStick < CORRECTION_STICK_DEADBAND_US && pitchStick > -CORRECTION_STICK_DEADBAND_US)
          {
            biasPitch = compute_correction_bias(deltaPitch, gyroPitch, CORRECTION_PITCH_SIGN, 1);
          }

          txChannels[0] = clamp_rc((int32_t)rcChannels[0] + biasRoll);
          txChannels[1] = clamp_rc((int32_t)rcChannels[1] + biasPitch);
        }

        if (correctionActive) LED_ON(LED_CORR_PORT, LED_CORR_PIN); else LED_OFF(LED_CORR_PORT, LED_CORR_PIN);

#if STATS_ENABLE
        stats_add(correctionActive ? &statsOn : &statsOff, deltaRoll, deltaPitch);
#endif
      }
    }
    else
    {
      /* No valid attitude this loop: don't leave a stale bias behind */
      attitudeFailCount++;
      biasRoll = 0;
      biasPitch = 0;
    }

    /* ---- 5. Motor outputs the FC is actually commanding --------------------- */
    {
      uint16_t m[4];
      if (msp_get_motors(&huart1, m) == HAL_OK)
      {
        motorOut[0] = m[0];
        motorOut[1] = m[1];
        motorOut[2] = m[2];
        motorOut[3] = m[3];
      }
      else
      {
        motorFailCount++;
      }
    }

    /* ---- 5b. Extra FC readings: its armed state, the channels it received, and the battery ---- */
#if FC_EXTRA_POLL_ENABLE
    if ((debugLoopCounter % FC_EXTRA_POLL_EVERY_N_LOOPS) == 0)
    {
      switch ((debugLoopCounter / FC_EXTRA_POLL_EVERY_N_LOOPS) % 3)
      {
        case 0:   /* MSP_STATUS */
        {
          uint16_t cyc = 0, sens = 0;
          uint32_t flags = 0;
          if (msp_get_status(&huart1, &cyc, &sens, &flags) == HAL_OK)
          {
            fcCycleTimeUs = cyc;
            fcSensors     = sens;
            fcModeFlags   = flags;
            fcArmed       = (uint8_t)(flags & 1u);
            fcStatusOkCount++;
          }
          else
          {
            fcStatusFailCount++;
          }
          break;
        }
        case 1:   /* MSP_RC: compare the FC's echo with what we sent last loop */
        {
          uint16_t rc[RC_CHANNEL_COUNT];
          if (msp_get_rc(&huart1, rc) == HAL_OK)
          {
            uint8_t mismatch = 0;
            for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++)
            {
              fcRc[i] = rc[i];
              if (lastSentRcValid)
              {
                int32_t d = (int32_t)rc[i] - (int32_t)lastSentRc[i];
                if (d < 0) d = -d;
                if (d > FC_RC_ECHO_TOLERANCE_US) mismatch = 1;
              }
            }
            if (mismatch) fcRcMismatchCount++;
            fcRcOkCount++;
          }
          else
          {
            fcRcFailCount++;
          }
          break;
        }
        default:  /* MSP_ANALOG, and the optional low-battery cutoff */
        {
          static uint8_t battLowCount = 0;
          uint16_t vb = 0, mah = 0;
          int16_t amps = 0;
          if (msp_get_analog(&huart1, &vb, &mah, &amps) == HAL_OK)
          {
            fcVbatDeci  = vb;
            fcMahDrawn  = mah;
            fcAmpsCenti = amps;
            fcAnalogOkCount++;
#if BATT_CUTOFF_ENABLE
            if (BATT_CUTOFF_DECIV > 0 && vb > 0 && vb < BATT_CUTOFF_DECIV)
            {
              if (battLowCount < 255) battLowCount++;
              if (battLowCount >= BATT_CUTOFF_READS) battLowLatched = 1;
            }
            else
            {
              battLowCount = 0;
            }
#else
            (void)battLowCount;
#endif
          }
          else
          {
            fcAnalogFailCount++;
          }
          break;
        }
      }
    }
#endif

#if BATT_CUTOFF_ENABLE
    if (battLowLatched)
    {
      txChannels[2] = 1000;   /* minimum throttle */
      txChannels[4] = 1000;   /* disarm */
    }
#endif

    /* ---- 6. Send to the FC: pilot input + correction bias ------------------- */
    txRollDbg     = txChannels[0];
    txPitchDbg    = txChannels[1];
    txYawDbg      = txChannels[3];
    txThrottleDbg = txChannels[2];
    txArmDbg      = txChannels[4];
    msp_set_raw_rc(&huart1, txChannels, RC_CHANNEL_COUNT);
    for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) lastSentRc[i] = txChannels[i];
    lastSentRcValid = 1;

    /* ---- 7. Logging --------------------------------------------------------- */
#if LOG_CSV_ENABLE
    if (baselineCaptured && (debugLoopCounter % LOG_CSV_EVERY_N_LOOPS) == 0)
    {
      /* CSV,ms,attRoll,attPitch,deltaRoll,deltaPitch,gyroRoll,gyroPitch,biasRoll,biasPitch,corr,thr,arm,m1,m2,m3,m4 */
      dbg_printf("CSV,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u\r\n",
                 (unsigned long)HAL_GetTick(), attRoll, attPitch, deltaRoll, deltaPitch,
                 gyroRoll, gyroPitch, biasRoll, biasPitch, (unsigned)correctionActive,
                 (unsigned)txThrottleDbg, (unsigned)txArmDbg,
                 (unsigned)motorOut[0], (unsigned)motorOut[1], (unsigned)motorOut[2], (unsigned)motorOut[3]);
    }
#endif

    if ((debugLoopCounter % STATUS_PRINT_EVERY_N_LOOPS) == 0 && baselineCaptured)
    {
      dbg_printf("STATUS: link=%u corr=%s | att roll=%d pitch=%d | delta roll=%d pitch=%d | bias roll=%d pitch=%d | thr=%u arm=%u | motors %u %u %u %u | fails att=%lu gyro=%lu mot=%lu%s\r\n",
                 (unsigned)pilotLinkAlive, correctionActive ? "ON" : "OFF",
                 attRoll, attPitch, deltaRoll, deltaPitch, biasRoll, biasPitch,
                 (unsigned)txThrottleDbg, (unsigned)txArmDbg,
                 (unsigned)motorOut[0], (unsigned)motorOut[1], (unsigned)motorOut[2], (unsigned)motorOut[3],
                 (unsigned long)attitudeFailCount, (unsigned long)gyroFailCount, (unsigned long)motorFailCount,
                 ((deltaRoll > ATTITUDE_DISTURBANCE_THRESHOLD_DECIDEG || deltaRoll < -ATTITUDE_DISTURBANCE_THRESHOLD_DECIDEG ||
                   deltaPitch > ATTITUDE_DISTURBANCE_THRESHOLD_DECIDEG || deltaPitch < -ATTITUDE_DISTURBANCE_THRESHOLD_DECIDEG) ? " DISTURBANCE" : ""));
    }

#if FC_EXTRA_POLL_ENABLE
    if ((debugLoopCounter % STATUS_PRINT_EVERY_N_LOOPS) == 0)
    {
      dbg_printf("FC: armed=%u cycle=%uus vbat=%u.%uV amps(0.01A)=%d used=%umAh rcMismatch=%lu fails st=%lu rc=%lu an=%lu%s\r\n",
                 (unsigned)fcArmed, (unsigned)fcCycleTimeUs,
                 (unsigned)(fcVbatDeci / 10), (unsigned)(fcVbatDeci % 10), (int)fcAmpsCenti, (unsigned)fcMahDrawn,
                 (unsigned long)fcRcMismatchCount,
                 (unsigned long)fcStatusFailCount, (unsigned long)fcRcFailCount, (unsigned long)fcAnalogFailCount,
                 battLowLatched ? " BATTERY CUTOFF" : "");
    }
#endif

#if INPUT_SOURCE == INPUT_LORA
    if ((debugLoopCounter % STATUS_PRINT_EVERY_N_LOOPS) == 0)
    {
      dbg_printf("LORA: ready=%u pkts=%lu bad=%lu lost=%lu rssi=%d dBm\r\n",
                 (unsigned)loraReady, (unsigned long)loraPktCount, (unsigned long)loraCrcErrCount,
                 (unsigned long)loraLostCount, (int)loraRssiDbm);
    }
#endif

#if STATS_ENABLE
    if ((HAL_GetTick() - lastStatsPrint) >= STATS_PRINT_MS)
    {
      lastStatsPrint = HAL_GetTick();
      if (baselineCaptured) stats_print();
    }
#endif

#if SD_LOG_ENABLE
    if ((debugLoopCounter % SD_LOG_EVERY_N_LOOPS) == 0)
    {
      g_log.t_ms     = HAL_GetTick();
      g_log.thr_tx   = txThrottleDbg;
      g_log.arm_tx   = (txArmDbg > 1500);
      g_log.fc_armed = fcArmed;
      g_log.link_ok  = pilotLinkAlive;
      g_log.vbat_deci  = fcVbatDeci;
      g_log.amps_centi = fcAmpsCenti;
      g_log.mah_drawn  = fcMahDrawn;
      g_log.cmd_roll   = stickRawRollUs;
      g_log.cmd_pitch  = stickRawPitchUs;
      g_log.cmd_yaw    = stickRawYawUs;
      g_log.tx_roll    = txRollDbg;
      g_log.tx_pitch   = txPitchDbg;
      g_log.tx_yaw     = txYawDbg;
      g_log.att_roll   = attRoll;
      g_log.att_pitch  = attPitch;
      g_log.att_yaw    = attYaw;
      Logger_Push(&g_log);
    }
    Logger_Service();
#endif

    /* loop timing instrumentation: counts only once the baseline is locked */
    loopWorkMs = (uint16_t)(HAL_GetTick() - loopStart);
    if (baselineCaptured)
    {
      if (loopWorkMs > loopWorkMsMax) loopWorkMsMax = loopWorkMs;
      if (loopWorkMs > LOOP_PERIOD_MS) loopOverrunCount++;
      loopWorkSumMs += loopWorkMs;
      loopWorkN++;
    }

    /* ---- 8. Pace the loop so timing is steady --------------------------------- */
    while ((HAL_GetTick() - loopStart) < LOOP_PERIOD_MS)
    {
      /* In CRSF mode the pilot poll blocks briefly; otherwise just wait out the remainder of the period */
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

#if UART_LOOPBACK_TEST
    /* Half-duplex: the chip connects its TX to its own RX internally, so it hears what it sends. */
    HAL_UART_DeInit(&huart1);
    if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
    {
      Error_Handler();
    }
  #endif

  /* HAL_UART_Init calls HAL_UART_MspInit, which can leave the RX pin with no pull-up. An undriven RX line
   * then floats and the UART "receives" noise. Re-apply PA9/PA10 here so the RX line idles high on its own. */
    {
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  #if INPUT_SOURCE == INPUT_CRSF
    /* USART6 carries an ExpressLRS receiver: 420000 baud, receive only */
    huart6.Init.BaudRate = 420000;
    huart6.Init.Mode = UART_MODE_RX;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
      Error_Handler();
    }
  #endif

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CD_Pin */
  GPIO_InitStruct.Pin = SD_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_CD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_SET);   /* both LEDs on = fatal error */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
