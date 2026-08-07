#include <M5Unified.h>
#include <AMY-Arduino.h>
#include "patch_names.h"
#include <Adafruit_MCP23X17.h>
#include <Adafruit_NeoPixel.h>
// =====================================================
// PATCH BANK
// =====================================================

#define MAXVOICE 6
#define NOTEBASE  47  // SW1: B
#define MATRIX_COLS 8
#define MATRIX_ROWS 8
#define KEYBOARD_COLS 4
#define KEY_COUNT  26
#define KEY_LED_COUNT 26
#define KEY_LED_PIN 18
#define KEY_LED_BRIGHTNESS 77 // 30% of 255
#define STARTUP_LED_TEST_MS 1000
#define HT16K33_LED_COM_COUNT 5
#define HT16K33_LED_ROWS 8
#define CS_PIN 9// SPI CS pin
#define ANALOG_CHANNELS 8
#define PITCH_BEND_MUX_CH 0
#define MODULATION_MUX_CH 1
#define POD_COUNT 6
#define POD_DISPLAY_ENABLED false
#define POD_DISPLAY_CHANGE_THRESHOLD 2
#define POD_STARTUP_SETTLE_MS 1000

#define HT16K33_ADDRESS 0x70
#define HT16K33_I2C_FREQUENCY 400000

#define ENCODER_A_PIN 1
#define ENCODER_B_PIN 2
#define ENCODER_SWITCH_PIN 17

byte key_read_data[MATRIX_COLS] ;
byte key_read_data_old[MATRIX_COLS] ;
bool joystickSwitchState = false;

enum PatchBank
{
  BANK_JUNO,
  BANK_DX7,
  BANK_PIANO
};

volatile PatchBank currentBank = BANK_JUNO;

volatile uint16_t currentPatch = 0;
volatile uint16_t requestedPatch = 0xFFFF;
volatile uint8_t selectedPatchIndex = 0;
volatile int8_t octaveShift = 0;

uint16_t ht16k33Buffer[8];
uint16_t ht16k33LastWrittenBuffer[8];
bool ht16k33LastWriteValid = false;
int lastPodValues[POD_COUNT];
bool podValuesInitialized = false;
uint32_t podStartupSettleUntilMs = 0;

volatile int32_t encoderPosition = 0;
volatile bool encoderSwitchPressed = false;
uint8_t encoderPreviousAB = 0;
int8_t encoderTransitionAccumulator = 0;

#define NOTE_FIFO_SIZE 32

struct NoteEvent {
  uint8_t note;
  bool on;        // true=NoteOn false=NoteOff
};

volatile NoteEvent noteFIFO[NOTE_FIFO_SIZE];
volatile uint8_t fifoHead = 0;
volatile uint8_t fifoTail = 0;

#define CONTROL_FIFO_SIZE 16

enum ControlEventType {
  CONTROL_PITCH_BEND,
  CONTROL_MODULATION
};

struct ControlEvent {
  ControlEventType type;
  int16_t value;
};

volatile ControlEvent controlFIFO[CONTROL_FIFO_SIZE];
volatile uint8_t controlFifoHead = 0;
volatile uint8_t controlFifoTail = 0;

volatile bool billieflg = false;

volatile bool uiDirty = true;

float millis_per_tick = 250;

int start_millis = 3000;
int last_cycle = -1;

int xst1 = 10;
int xst2 = 130;
int yst0 = 120;
int yst1 = 170;
int yst2 = 220;
int yst3 = 270;
int xsize = 100 ;
int ysize = 45 ;
int rsize = 8 ;
int xoffset = 30;
int yoffset = 20 ;

uint8_t GPA[] = {0, 1, 2, 3, 4, 5, 6, 7}; //21-28  output
uint8_t GPB[] = {8, 9, 10, 11, 12, 13, 14, 15};  //1-8 input

uint8_t note_num[MAXVOICE];
uint8_t sounding_note[MAXVOICE];

uint8_t channel  = 1;
// 不感帯の設定（センター付近の微小なブレを無視する幅）
const int joystickDeadZone = 12;
const int pitchBendChangeThreshold = 64;
const int changeThresholdCC = 2;
const int modulationVibratoMaxBend = 2048;
const unsigned long modulationUpdateIntervalMs = 20;
const float modulationVibratoHz = 5.0f;

// キャリブレーション（初期値、必要に応じて調整）
int centerPB = 128;  // ピッチベンド軸の初期中央値
int centerMod = 128; // モジュレーション軸の初期中央値

// 状態記録用変数
int lastPitchBend = 0;
int lastModulation = -1;
int currentPitchBendBase = 0;
int currentModulationAmount = 0;
int lastSentCombinedPitchBend = 0;
unsigned long lastModulationUpdateMillis = 0;

Adafruit_MCP23X17 mcp;
Adafruit_NeoPixel keyLeds(
  KEY_LED_COUNT,
  KEY_LED_PIN,
  NEO_GRB + NEO_KHZ800
);

struct timed_note {
  float start_time;  // In ticks
  float duration;    // In ticks
  int note;
  float velocity;
};

// Cycle length (in ticks) for drums + bass
float cycle_len = 8.0;

// Drum notes have durations of 0 for no note-off
timed_note drum_notes[] = {
  { 0.0, 0.0, 42, 1.0},  //  0 HH + BD
  { 0.0, 0.0, 35, 1.0},
  { 1.0, 0.0, 42, 1.0},  //  1 HH
  { 2.0, 0.0, 42, 1.0},  //  2 HH + SN
  { 2.0, 0.0, 37, 1.0},
  { 3.0, 0.0, 42, 1.0},  //  3 HH
  { 4.0, 0.0, 42, 1.0},  //  4 HH + BD
  { 4.0, 0.0, 35, 1.0},
  { 5.0, 0.0, 42, 1.0},  //  5 HH
  { 6.0, 0.0, 42, 1.0},  //  6 HH + SN
  { 6.0, 0.0, 37, 1.0},
  { 7.0, 0.0, 42, 2.0},  //  7 OH
};

timed_note bass_notes[] = {
  { 0.0, 0.6, 43, 0.4},   // bass G2
  { 1.0, 0.6, 38, 0.2},   // bass D2
  { 2.0, 0.6, 41, 0.2},   // bass F2
  { 3.0, 0.6, 43, 0.4},   // bass G2
  { 4.0, 0.6, 41, 0.2},   // bass F2
  { 5.0, 0.6, 38, 0.2},   // bass D2
  { 6.0, 0.6, 36, 0.2},   // bass C2
  { 7.0, 0.6, 38, 0.2},   // bass D2
};

timed_note chord_notes[] = {
  { 0.0, 0.2, 70, 1.0},  // Fmin:1
  { 0.0, 0.2, 74, 1.0},
  { 0.0, 0.2, 79, 1.0},
  { 3.0, 0.2, 72, 1.0},  // Amin:1
  { 3.0, 0.2, 76, 1.0},
  { 3.0, 0.2, 81, 1.0},
  { 8.0, 0.2, 74, 1.0},  // Bb:1
  { 8.0, 0.2, 77, 1.0},
  { 8.0, 0.2, 82, 1.0},
  { 11.0, 0.2, 72, 1.0},  // Amin:1
  { 11.0, 0.2, 76, 1.0},
  { 11.0, 0.2, 81, 1.0},
};

int addata[ANALOG_CHANNELS];

const int analogPin = 10;

const float filterCoefficient = 0.1;
const bool debugAnalogRead = false;
float filteredValue[ANALOG_CHANNELS];
// 不感帯の幅。1未満のブレを無視するために設定
const int deadband = 1;
int lastStableValue[ANALOG_CHANNELS]; // 最終的に確定した安定値を保存
bool analogInitialized[ANALOG_CHANNELS];

void pushControlEvent(ControlEventType type, int16_t value);
bool popControlEvent(ControlEvent *ev);
int getAverageAnalog(int pin, int no);
void handleJoystickControls();
int mapCenteredJoystick(int rawValue, int centerValue, int deadZone, int minOut, int maxOut);
int mapPositiveJoystick(int rawValue, int centerValue, int deadZone);
void updatePitchBendModulation(bool force);
void sendPitchBendToAmy(int bendValue);
void scan_keyboard();
void noteget();
void handlePanelSwitchPress(uint8_t switchIndex);
void updateEncoderInput();
void handlePodInputs();
void selectAnalogChannel(uint8_t channel);
bool ht16k33Begin();
void ht16k33Write();
void showUnsignedValue(uint16_t value);
void showOctaveShift();
void updatePatchIndicators();
void requestSelectedPatch();
uint16_t selectedAmyPatch();
uint16_t selectedDisplayPatch();
void selectPatchButton(uint8_t buttonIndex);
void nextPatchPage();
void setKeyboardLed(uint8_t keyIndex, bool on);
void runStartupLedTest();

// =====================================================
// SETUP
// =====================================================

void setup()
{
  auto cfg = M5.config();

cfg.serial_baudrate = 115200;

  M5.begin(cfg);

Serial.println("setup start");

pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SWITCH_PIN, INPUT_PULLUP);
  encoderPreviousAB =
    (digitalRead(ENCODER_A_PIN) << 1) |
    digitalRead(ENCODER_B_PIN);
  analogReadResolution(8);

  keyLeds.begin();
  keyLeds.setBrightness(KEY_LED_BRIGHTNESS);
  keyLeds.clear();
  keyLeds.show();

  M5.Display.setRotation(0);

// MCP23S17 start
  if (!mcp.begin_SPI(CS_PIN)) {
    Serial.print("mcp error.");
    while (1);
  }

  // GPB0-7 receive the active-low KEY_ROW_0-7 signals.
  for (int i = 0; i < MATRIX_ROWS; i++) {
    mcp.pinMode(GPB[i], INPUT_PULLUP);
  }

  // GPA0-2 select KEY_COL_0-7 through the 74HC138.
  // GPA3-5 select the analog multiplexer channel.
  for (int i = 0; i <= 5; i++) {
    mcp.pinMode(GPA[i], OUTPUT);
    mcp.digitalWrite(GPA[i], LOW);
  }
  // GPA6 reads the joystick push switch. GPA7 drives the board LED.
  mcp.pinMode(GPA[6], INPUT_PULLUP);
  mcp.pinMode(GPA[7], OUTPUT);
  mcp.digitalWrite(GPA[7], LOW);

  if (!ht16k33Begin()) {
    Serial.println("HT16K33 error.");
  }
  runStartupLedTest();

  amy_config_t amy_config =
    amy_default_config();

  amy_config.features.startup_bleep = 1;
  amy_config.features.default_synths = 1;

  amy_config.audio = AMY_AUDIO_IS_I2S;

  amy_config.i2s_mclk = 8;
  amy_config.i2s_bclk = 6;
  amy_config.i2s_lrc  = 7;
  amy_config.i2s_dout = 5;

  amy_config.midi = AMY_MIDI_IS_UART;
  amy_config.midi_in = 44;

  amy_start(amy_config);

  // Reconfigure synth 1 as a 6-note polyphonic synth (for chords)
  amy_event e = amy_default_event();
  e.synth = 1;
  e.patch_number = 5;  // Juno A16 Brass & Strings
  e.num_voices = 6;
  amy_add_event(&e);

  // Reconfigure synth 2 as monophonic bass
  e = amy_default_event();
  e.synth = 2;
  e.patch_number = 30;  // Juno A47 Funky I
  e.num_voices = 1;
  amy_add_event(&e);

  drawStaticUI();
  requestSelectedPatch();

  xTaskCreatePinnedToCore(
    uiTask,
    "uiTask",
    8192,
    nullptr,
    1,
    nullptr,
    1
  );

  Serial.println("setup end");

}

// =====================================================
// HT16K33 / 3-DIGIT DISPLAY / PATCH LEDS
// =====================================================

static const uint8_t digitSegments[10] = {
  0x3F, // 0
  0x06, // 1
  0x5B, // 2
  0x4F, // 3
  0x66, // 4
  0x6D, // 5
  0x7D, // 6
  0x07, // 7
  0x7F, // 8
  0x6F  // 9
};

bool ht16k33Command(uint8_t command)
{
  if (!M5.In_I2C.start(
        HT16K33_ADDRESS,
        false,
        HT16K33_I2C_FREQUENCY
      )) {
    return false;
  }

  bool writeOk = M5.In_I2C.write(command);
  bool stopOk = M5.In_I2C.stop();
  return writeOk && stopOk;
}

bool ht16k33Begin()
{
  memset(ht16k33Buffer, 0, sizeof(ht16k33Buffer));
  ht16k33LastWriteValid = false;

  bool ok = true;
  ok &= ht16k33Command(0x21); // oscillator on
  ok &= ht16k33Command(0x81); // display on, blink off
  ok &= ht16k33Command(0xEF); // maximum brightness
  ht16k33Write();
  return ok;
}

void ht16k33Write()
{
  if (ht16k33LastWriteValid &&
      memcmp(
        ht16k33Buffer,
        ht16k33LastWrittenBuffer,
        sizeof(ht16k33Buffer)
      ) == 0) {
    return;
  }

  uint8_t displayData[16];
  for (int com = 0; com < 8; com++) {
    displayData[com * 2] =
      (uint8_t)(ht16k33Buffer[com] & 0xFF);
    displayData[com * 2 + 1] =
      (uint8_t)(ht16k33Buffer[com] >> 8);
  }

  bool writeOk = M5.In_I2C.writeRegister(
    HT16K33_ADDRESS,
    0x00,
    displayData,
    sizeof(displayData),
    HT16K33_I2C_FREQUENCY
  );
  if (writeOk) {
    memcpy(
      ht16k33LastWrittenBuffer,
      ht16k33Buffer,
      sizeof(ht16k33Buffer)
    );
    ht16k33LastWriteValid = true;
  } else {
    ht16k33LastWriteValid = false;
    Serial.println("HT16K33 write error");
  }
}

void setDisplayDigit(uint8_t position, uint8_t segments)
{
  // The schematic connects left, center, right digits to COM2, COM1, COM0.
  static const uint8_t digitCom[3] = {2, 1, 0};
  if (position >= 3) {
    return;
  }

  uint8_t com = digitCom[position];
  ht16k33Buffer[com] =
    (ht16k33Buffer[com] & 0xFF00) | segments;
}

void setUnsignedValue(uint16_t value)
{
  value = min((uint16_t)999, value);
  setDisplayDigit(0, digitSegments[(value / 100) % 10]);
  setDisplayDigit(1, digitSegments[(value / 10) % 10]);
  setDisplayDigit(2, digitSegments[value % 10]);
}

void showUnsignedValue(uint16_t value)
{
  setUnsignedValue(value);
  ht16k33Write();
}

void showOctaveShift()
{
  int8_t value = octaveShift;
  if (value < 0) {
    setDisplayDigit(0, 0x40); // minus sign (segment G)
    value = -value;
  } else {
    setDisplayDigit(0, digitSegments[0]);
  }
  setDisplayDigit(1, digitSegments[0]);
  setDisplayDigit(2, digitSegments[value]);
  ht16k33Write();
}

void setSelectedPatchLed(uint8_t buttonIndex)
{
  // COM3 drives SW1-8 LEDs and COM4 drives SW9-16 LEDs.
  ht16k33Buffer[3] &= 0xFF00;
  ht16k33Buffer[4] &= 0xFF00;

  if (buttonIndex < 8) {
    ht16k33Buffer[3] |= (uint16_t)1 << buttonIndex;
  } else if (buttonIndex < 16) {
    ht16k33Buffer[4] |= (uint16_t)1 << (buttonIndex - 8);
  }
}

uint16_t selectedDisplayPatch()
{
  // Patch numbers are shown as zero-padded values from 000 through 127.
  return currentBank == BANK_PIANO
           ? 0
           : selectedPatchIndex;
}

uint16_t selectedAmyPatch()
{
  switch (currentBank) {
    case BANK_JUNO:
      return selectedPatchIndex;
    case BANK_DX7:
      return 128 + selectedPatchIndex;
    case BANK_PIANO:
      return 256;
  }
  return 0;
}

void updatePatchIndicators()
{
  uint8_t ledIndex =
    currentBank == BANK_PIANO
      ? 0
      : selectedPatchIndex % 16;

  // Commit the selected SW1-SW16 LED and the 3-digit patch number together.
  setSelectedPatchLed(ledIndex);
  setUnsignedValue(selectedDisplayPatch());
  ht16k33Write();
}

void requestSelectedPatch()
{
  requestPatch(selectedAmyPatch());
  updatePatchIndicators();
  uiDirty = true;
}

void setKeyboardLed(uint8_t keyIndex, bool on)
{
  if (keyIndex >= KEY_LED_COUNT) {
    return;
  }

  // PCB mapping is direct: SW1-L1 through SW26-L26.
  uint32_t color = on
                     ? keyLeds.Color(128, 0, 255) // blue-violet
                     : 0;
  keyLeds.setPixelColor(keyIndex, color);
  keyLeds.show();
}

void runStartupLedTest()
{
  const uint16_t totalLedCount =
    KEY_LED_COUNT + HT16K33_LED_COM_COUNT * HT16K33_LED_ROWS;
  const uint32_t startTime = millis();
  uint16_t step = 0;

  auto waitForNextStep = [&]() {
    step++;
    const uint32_t targetTime =
      startTime + (uint32_t)step * STARTUP_LED_TEST_MS / totalLedCount;
    const int32_t remaining = (int32_t)(targetTime - millis());
    if (remaining > 0) {
      delay((uint32_t)remaining);
    }
  };

  // Test the 26 keyboard NeoPixels in PCB chain order L1 through L26.
  for (uint8_t ledIndex = 0; ledIndex < KEY_LED_COUNT; ledIndex++) {
    keyLeds.clear();
    keyLeds.setPixelColor(ledIndex, keyLeds.Color(128, 0, 255));
    keyLeds.show();
    waitForNextStep();
  }
  keyLeds.clear();
  keyLeds.show();

  // COM0-2 are the three 8-segment digits; COM3-4 are the 16 patch LEDs.
  for (uint8_t com = 0; com < HT16K33_LED_COM_COUNT; com++) {
    for (uint8_t row = 0; row < HT16K33_LED_ROWS; row++) {
      memset(ht16k33Buffer, 0, sizeof(ht16k33Buffer));
      ht16k33Buffer[com] = (uint16_t)1 << row;
      ht16k33Write();
      waitForNextStep();
    }
  }

  memset(ht16k33Buffer, 0, sizeof(ht16k33Buffer));
  ht16k33Write();
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  if (requestedPatch != 0xFFFF)
  {
    uint16_t patch = requestedPatch;

    requestedPatch = 0xFFFF;

    amy_event e = amy_default_event();

    e.synth = 1;
    e.patch_number = patch;

    amy_add_event(&e);

    currentPatch = patch;
    uiDirty = true;
  }

  NoteEvent ev;

  while (popNoteEvent(&ev))
  {
    if (ev.on)
    {
      amy_event e = amy_default_event();
      e.synth = channel;
      e.midi_note = ev.note ;
      e.velocity = 1.0f;
      amy_add_event(&e);
    }
    else
    {
      amy_event e = amy_default_event();
      e.synth = channel;
      e.midi_note = ev.note ;
      e.velocity = 0.0;
      amy_add_event(&e);
    }
  }

  ControlEvent cev;
  bool controlChanged = false;

  while (popControlEvent(&cev))
  {
    if (cev.type == CONTROL_PITCH_BEND)
    {
      currentPitchBendBase = cev.value;
      controlChanged = true;
    }
    else if (cev.type == CONTROL_MODULATION)
    {
      currentModulationAmount = cev.value;
      controlChanged = true;
    }
  }

  updatePitchBendModulation(controlChanged);

if (billieflg)
  {

    int now = millis();
    int current_cycle = floor((now - start_millis) / (millis_per_tick * cycle_len));
    if (current_cycle > last_cycle) {
      // A new cycle began, issue notes.
      // Drums
      schedule_notes(now, 10, drum_notes, sizeof(drum_notes) / sizeof(timed_note));
      // Bass comes in after two cycles of drums
      if (current_cycle >= 2)
        schedule_notes(now, 2, bass_notes, sizeof(bass_notes) / sizeof(timed_note));
      // Chord sequence is 2 cycles long, so only schedule every other cycle
      if ((current_cycle >= 4) && ((current_cycle % 2) == 0))
        schedule_notes(now, 1, chord_notes, sizeof(chord_notes) / sizeof(timed_note));
      last_cycle = current_cycle;
    }

  }
  amy_update();
}

void pushNoteEvent(uint8_t note, bool on)
{
  uint8_t next = (fifoHead + 1) % NOTE_FIFO_SIZE;

  // FIFO満杯
  if (next == fifoTail)
    return;

  noteFIFO[fifoHead].note = note;
  noteFIFO[fifoHead].on = on;

  fifoHead = next;
}

bool popNoteEvent(NoteEvent *ev)
{
    if (fifoHead == fifoTail)
        return false;

    ev->note = noteFIFO[fifoTail].note;
    ev->on   = noteFIFO[fifoTail].on;

    fifoTail = (fifoTail + 1) % NOTE_FIFO_SIZE;

    return true;
}

void pushControlEvent(ControlEventType type, int16_t value)
{
  uint8_t next = (controlFifoHead + 1) % CONTROL_FIFO_SIZE;

  if (next == controlFifoTail)
    return;

  controlFIFO[controlFifoHead].type = type;
  controlFIFO[controlFifoHead].value = value;

  controlFifoHead = next;
}

bool popControlEvent(ControlEvent *ev)
{
  if (controlFifoHead == controlFifoTail)
    return false;

  ev->type = controlFIFO[controlFifoTail].type;
  ev->value = controlFIFO[controlFifoTail].value;

  controlFifoTail = (controlFifoTail + 1) % CONTROL_FIFO_SIZE;

  return true;
}

void sendPitchBendToAmy(int bendValue)
{
  amy_event e = amy_default_event();
  e.synth = channel;
  e.pitch_bend = ((float)bendValue) / (6.0f * 8192.0f);
  amy_add_event(&e);
}

void updatePitchBendModulation(bool force)
{
  unsigned long now = millis();

  if (!force &&
      currentModulationAmount == 0 &&
      currentPitchBendBase == lastSentCombinedPitchBend)
    return;

  if (!force &&
      currentModulationAmount > 0 &&
      now - lastModulationUpdateMillis < modulationUpdateIntervalMs)
    return;

  int vibrato = 0;

  if (currentModulationAmount > 0)
  {
    float depth =
      ((float)modulationVibratoMaxBend * (float)currentModulationAmount) / 127.0f;
    float phase =
      2.0f * PI * modulationVibratoHz * ((float)now / 1000.0f);
    vibrato = (int)(sinf(phase) * depth);
  }

  int combinedPitchBend =
    constrain(
      currentPitchBendBase + vibrato,
      -8192,
      8191
    );

  if (force ||
      abs(combinedPitchBend - lastSentCombinedPitchBend) >= pitchBendChangeThreshold ||
      (currentModulationAmount == 0 && combinedPitchBend != lastSentCombinedPitchBend))
  {
    sendPitchBendToAmy(combinedPitchBend);
    lastSentCombinedPitchBend = combinedPitchBend;
  }

  lastModulationUpdateMillis = now;
}

//adc

void selectAnalogChannel(uint8_t channel)
{
  mcp.digitalWrite(GPA[3], (channel >> 0) & 1);
  mcp.digitalWrite(GPA[4], (channel >> 1) & 1);
  mcp.digitalWrite(GPA[5], (channel >> 2) & 1);
}

void adreaddata()
{
  for (auto i = 0 ; i < ANALOG_CHANNELS; i++)
  {
    selectAnalogChannel(i);
    delayMicroseconds(5);
    analogRead(analogPin); // discard the first sample after changing mux channel

    addata[i] = getAverageAnalog(analogPin, i);

    if (debugAnalogRead)
    {
      Serial.print(addata[i]);
      Serial.print(",");
    }

  }

  if (debugAnalogRead)
    Serial.println();

  handleJoystickControls();
  handlePodInputs();

}

void handlePodInputs()
{
  int currentValues[POD_COUNT];
  for (int i = 0; i < POD_COUNT; i++) {
    currentValues[i] =
      constrain(
        map(addata[i + 2], 0, 255, 0, 127),
        0,
        127
      );
  }

  if (!podValuesInitialized) {
    for (int i = 0; i < POD_COUNT; i++) {
      lastPodValues[i] = currentValues[i];
    }
    podStartupSettleUntilMs = millis() + POD_STARTUP_SETTLE_MS;
    podValuesInitialized = true;
    return;
  }

  // Let the ADC multiplexer and low-pass filters settle after UI startup.
  // Track the baseline during this period without replacing the patch display.
  if ((int32_t)(millis() - podStartupSettleUntilMs) < 0) {
    for (int i = 0; i < POD_COUNT; i++) {
      lastPodValues[i] = currentValues[i];
    }
    return;
  }

  if (!POD_DISPLAY_ENABLED) {
    for (int i = 0; i < POD_COUNT; i++) {
      lastPodValues[i] = currentValues[i];
    }
    return;
  }

  int lastChangedPod = -1;
  for (int i = 0; i < POD_COUNT; i++) {
    if (abs(currentValues[i] - lastPodValues[i]) >=
        POD_DISPLAY_CHANGE_THRESHOLD) {
      lastPodValues[i] = currentValues[i];
      lastChangedPod = i;
      Serial.printf("POD%d=%d\n", i, currentValues[i]);
    }
  }

  // Multiple POD changes in one scan are coalesced into one display transfer.
  if (lastChangedPod >= 0) {
    showUnsignedValue(currentValues[lastChangedPod]);
  }
}

void updateEncoderInput()
{
  static const int8_t transitionTable[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };

  uint8_t currentAB =
    (digitalRead(ENCODER_A_PIN) << 1) |
    digitalRead(ENCODER_B_PIN);
  uint8_t transition = (encoderPreviousAB << 2) | currentAB;
  encoderPreviousAB = currentAB;
  encoderTransitionAccumulator += transitionTable[transition];

  if (encoderTransitionAccumulator >= 4) {
    encoderPosition++;
    encoderTransitionAccumulator = 0;
    Serial.printf("ENCODER=%ld CW\n", (long)encoderPosition);
  } else if (encoderTransitionAccumulator <= -4) {
    encoderPosition--;
    encoderTransitionAccumulator = 0;
    Serial.printf("ENCODER=%ld CCW\n", (long)encoderPosition);
  }

  bool pressed = digitalRead(ENCODER_SWITCH_PIN) == LOW;
  if (pressed != encoderSwitchPressed) {
    encoderSwitchPressed = pressed;
    Serial.printf("ENCODER_SW=%s\n", pressed ? "DOWN" : "UP");
  }
}

//float filteredValue[no];
//int lastStableValue[no];

int getAverageAnalog(int pin, int no) {

  int rawValue = analogRead(analogPin);

  if (!analogInitialized[no])
  {
    filteredValue[no] = rawValue;
    lastStableValue[no] = rawValue;
    analogInitialized[no] = true;
    return lastStableValue[no];
  }

  // 1. ローパスフィルタで突発ノイズをならす
  filteredValue[no] = (filterCoefficient * rawValue) + ((1.0 - filterCoefficient) * filteredValue[no]);
  int currentIntVal = (int)filteredValue[no];

  // 2. 前回の安定値から一定以上動いた時だけ値を更新する
  if (abs(currentIntVal - lastStableValue[no]) >= deadband) {

    lastStableValue[no] = currentIntVal;
  }

  return lastStableValue[no];
}

int mapCenteredJoystick(int rawValue, int centerValue, int deadZone, int minOut, int maxOut)
{
  int delta = rawValue - centerValue;

  if (abs(delta) <= deadZone)
    return 0;

  if (delta < 0)
  {
    int lowLimit = max(0, centerValue - deadZone);
    return constrain(
             map(rawValue, 0, lowLimit, minOut, 0),
             minOut,
             0
           );
  }

  int highLimit = min(255, centerValue + deadZone);
  return constrain(
           map(rawValue, highLimit, 255, 0, maxOut),
           0,
           maxOut
         );
}

int mapPositiveJoystick(int rawValue, int centerValue, int deadZone)
{
  if (rawValue <= centerValue + deadZone)
    return 0;

  int highLimit = min(255, centerValue + deadZone);
  return constrain(
           map(rawValue, highLimit, 255, 0, 127),
           0,
           127
         );
}

void handleJoystickControls()
{
  int currentPitchBend =
    mapCenteredJoystick(
      addata[PITCH_BEND_MUX_CH],
      centerPB,
      joystickDeadZone,
      -8192,
      8191
    );

  if (abs(currentPitchBend - lastPitchBend) >= pitchBendChangeThreshold ||
      (currentPitchBend == 0 && lastPitchBend != 0))
  {
    pushControlEvent(CONTROL_PITCH_BEND, currentPitchBend);
    lastPitchBend = currentPitchBend;
  }

  int currentModulation =
    mapPositiveJoystick(
      addata[MODULATION_MUX_CH],
      centerMod,
      joystickDeadZone
    );

  if (abs(currentModulation - lastModulation) >= changeThresholdCC)
  {
    pushControlEvent(CONTROL_MODULATION, currentModulation);
    lastModulation = currentModulation;
  }
}

// =====================================================
// BilleJean
// =====================================================

void schedule_notes(int time, int channel, struct timed_note *notes, int num_notes) {
  amy_event e = amy_default_event();
  e.synth = channel;
  for (int i = 0; i < num_notes; ++i) {
    e.midi_note = notes[i].note;
    e.velocity = notes[i].velocity;
    e.time = time + millis_per_tick * notes[i].start_time;
    amy_add_event(&e);
    // Add note-off too if duration > 0
    if (notes[i].duration > 0) {
      e.time += millis_per_tick * notes[i].duration;
      e.velocity = 0;
      amy_add_event(&e);
    }
  }
}

// =====================================================
// SCAN KEYBOARD
// =====================================================

void scan_keyboard()
{
  for (int column = 0; column < MATRIX_COLS; column++) {
    // The schematic routes GPA2->A0, GPA1->A1, GPA0->A2.
    mcp.digitalWrite(GPA[2], (column >> 0) & 1);
    mcp.digitalWrite(GPA[1], (column >> 1) & 1);
    mcp.digitalWrite(GPA[0], (column >> 2) & 1);
    delayMicroseconds(2);

    key_read_data_old[column] = key_read_data[column];
    key_read_data[column] = (uint8_t)~mcp.readGPIOB(); // pressed key = 1
  }
}

void noteget()
{
  for (int column = 0; column < MATRIX_COLS; column++) {
    byte changed =
      key_read_data[column] ^ key_read_data_old[column];
    if (changed == 0) {
      continue;
    }

    for (int row = 0; row < MATRIX_ROWS; row++) {
      if (((changed >> row) & 0x01) == 0) {
        continue;
      }

      bool wasPressed =
        ((key_read_data_old[column] >> row) & 0x01) != 0;
      bool isPressed =
        ((key_read_data[column] >> row) & 0x01) != 0;

      if (column < KEYBOARD_COLS) {
        const int keyIndex = row + column * MATRIX_ROWS;
        if (keyIndex >= KEY_COUNT) {
          continue;
        }

        const int keyNumber = keyIndex + NOTEBASE;
        if (!wasPressed && isPressed) {
          setKeyboardLed(keyIndex, true);
          for (int voice = 0; voice < MAXVOICE; voice++) {
            if (note_num[voice] == 0) {
              note_on(voice, keyNumber);
              note_num[voice] = keyNumber;
              break;
            }
          }
        } else if (wasPressed && !isPressed) {
          setKeyboardLed(keyIndex, false);
          for (int voice = 0; voice < MAXVOICE; voice++) {
            if (note_num[voice] == keyNumber) {
              note_off(voice, keyNumber);
              note_num[voice] = 0;
            }
          }
        }
        continue;
      }

      uint8_t switchIndex =
        (column - KEYBOARD_COLS) * MATRIX_ROWS + row;
      Serial.printf(
        "SW%d=%s\n",
        switchIndex + 1,
        isPressed ? "DOWN" : "UP"
      );
      if (!wasPressed && isPressed) {
        handlePanelSwitchPress(switchIndex);
      }
    }
  }

  bool currentJoystickSwitch =
    mcp.digitalRead(GPA[6]) == LOW;
  if (currentJoystickSwitch != joystickSwitchState) {
    joystickSwitchState = currentJoystickSwitch;
    Serial.printf(
      "JOY_SW=%s\n",
      joystickSwitchState ? "DOWN" : "UP"
    );
  }
}

void note_on(int voice_no, int key_no)
{
  /*
    Serial.print("note on:" );
    Serial.print(voice_no);
    Serial.print(" key:" );
    Serial.println(key_no);
  */
  int midiNote =
    constrain(key_no + octaveShift * 12, 0, 127);
  sounding_note[voice_no] = midiNote;

  int drum_offset = 0;
  if (channel == 10) {
    drum_offset = -12;
  } else {
    drum_offset = 0;
  }
  //MIDI.sendNoteOn(midiNote + drum_offset, 127, channel);
  // pixelset(key_no - NOTEBASE, 50, 50, 128);

  // requestedNoteOn = midiNote + drum_offset;
  pushNoteEvent(midiNote + drum_offset, true);
}

void note_off(int voice_no, int key_no)
{
  /*
    Serial.print("note off:" );
    Serial.print(voice_no);
    Serial.print(" key:" );
    Serial.println(key_no);
  */
  int drum_offset = 0;

  if (channel == 10) {
    drum_offset = -12;
  } else {
    drum_offset = 0;
  }
  //MIDI.sendNoteOff(sounding_note[voice_no] + drum_offset, 0, channel);
  // pixelset(key_no - NOTEBASE, 0, 0, 0 );
  // Use the note captured at key-down time so octave changes cannot leave
  // a sounding note stuck.
  pushNoteEvent(sounding_note[voice_no] + drum_offset, false);
  sounding_note[voice_no] = 0;
}

// =====================================================
// BANK NAME
// =====================================================

const char* getBankName()
{
  switch (currentBank)
  {
    case BANK_JUNO:  return "JUNO";
    case BANK_DX7:   return "DX7";
    case BANK_PIANO: return "PIANO";
  }
  return "";
}

// =====================================================
// PATCH REQUEST
// =====================================================

void requestPatch(uint16_t patch)
{
  requestedPatch = patch;
}

// =====================================================
// BANK
// =====================================================

void nextBank()
{
  switch (currentBank)
  {
    case BANK_JUNO:
      currentBank = BANK_DX7;
      break;
    case BANK_DX7:
      currentBank = BANK_PIANO;
      break;
    case BANK_PIANO:
      currentBank = BANK_JUNO;
      break;
  }
  requestSelectedPatch();
}

void prevBank()
{
  switch (currentBank)
  {
    case BANK_JUNO:
      currentBank = BANK_PIANO;
      break;

    case BANK_DX7:
      currentBank = BANK_JUNO;
      break;

    case BANK_PIANO:
      currentBank = BANK_DX7;
      break;
  }
  requestSelectedPatch();
}

// =====================================================
// PATCH +/-1
// =====================================================

void patchMinus1()
{
  if (currentBank == BANK_PIANO) {
    return;
  }
  if (selectedPatchIndex > 0) {
    selectedPatchIndex--;
    requestSelectedPatch();
  }
}

void patchPlus1()
{
  if (currentBank == BANK_PIANO) {
    return;
  }
  if (selectedPatchIndex < 127) {
    selectedPatchIndex++;
    requestSelectedPatch();
  }
}

// =====================================================
// PATCH +/-10
// =====================================================

void patchMinus10()
{
  if (currentBank == BANK_PIANO) {
    return;
  }
  selectedPatchIndex =
    selectedPatchIndex >= 10
      ? selectedPatchIndex - 10
      : 0;
  requestSelectedPatch();
}

void patchPlus10()
{
  if (currentBank == BANK_PIANO) {
    return;
  }
  selectedPatchIndex =
    min((uint16_t)127, (uint16_t)(selectedPatchIndex + 10));
  requestSelectedPatch();
}

void selectPatchButton(uint8_t buttonIndex)
{
  if (buttonIndex >= 16) {
    return;
  }

  if (currentBank == BANK_PIANO) {
    if (buttonIndex == 0) {
      requestSelectedPatch();
    }
    return;
  }

  uint8_t page = selectedPatchIndex / 16;
  selectedPatchIndex = page * 16 + buttonIndex;
  requestSelectedPatch();
}

void nextPatchPage()
{
  if (currentBank == BANK_PIANO) {
    return;
  }

  uint8_t page = selectedPatchIndex / 16;
  uint8_t position = selectedPatchIndex % 16;
  page = (page + 1) % 8;
  selectedPatchIndex = page * 16 + position;
  requestSelectedPatch();
}

void handlePanelSwitchPress(uint8_t switchIndex)
{
  if (switchIndex < 16) {
    selectPatchButton(switchIndex);
    return;
  }

  switch (switchIndex + 1) {
    case 23:
      if (octaveShift > -2) {
        octaveShift--;
        showOctaveShift();
        Serial.printf("OCTAVE=%d\n", octaveShift);
      }
      break;

    case 24:
      if (octaveShift < 2) {
        octaveShift++;
        showOctaveShift();
        Serial.printf("OCTAVE=%d\n", octaveShift);
      }
      break;

    case 27:
      nextPatchPage();
      break;

    case 28:
      nextBank();
      break;

    // SW17-22 and SW25-26 are reserved for the screen UI.
    // SW29-32 are acquired but intentionally have no function yet.
    default:
      break;
  }
}

// =====================================================
// UI
// =====================================================

void drawStaticUI()
{
  auto& lcd = M5.Display;

  lcd.fillScreen(TFT_BLACK);

  lcd.setTextColor(TFT_WHITE);
  lcd.setTextSize(2);

  lcd.setCursor(10, 10);
  lcd.println("M5 AMY MIDI Synth");

  lcd.drawRoundRect(xst1, yst1, xsize, ysize, rsize, TFT_MAGENTA);
  lcd.drawRoundRect(xst2, yst1, xsize, ysize, rsize, TFT_MAGENTA);

  lcd.setCursor(xst1 + xoffset, yst1 + yoffset);
  lcd.print("B-");

  lcd.setCursor(xst2 + xoffset, yst1 + yoffset);
  lcd.print("B+");

  lcd.drawRoundRect(xst1, yst2, xsize, ysize, rsize, TFT_RED);
  lcd.drawRoundRect(xst2, yst2, xsize, ysize, rsize, TFT_GREEN);

  lcd.setCursor(xst1 + xoffset, yst2 + yoffset);
  lcd.print("-1");
  lcd.setCursor(xst2 + xoffset, yst2 + yoffset);
  lcd.print("+1");

  lcd.drawRoundRect(xst1, yst3, xsize, ysize, rsize, TFT_BLUE);
  lcd.drawRoundRect(xst2, yst3, xsize, ysize, rsize, TFT_BLUE);

  lcd.setCursor(xst1 + xoffset, yst3 + yoffset);
  lcd.print("-10");

  lcd.setCursor(xst2 + xoffset, yst3 + yoffset);

  lcd.print("+10");

  lcd.drawRoundRect(xst2, yst0, xsize, ysize, rsize, TFT_CYAN);
  lcd.setCursor(xst2 + xoffset - 10, yst0  + yoffset);
  lcd.setTextColor(TFT_CYAN, BLACK);
  lcd.print("START");

}

void billiejean()
{

  auto& lcd = M5.Display;
  if (billieflg)
  {
    billieflg = false ;
    // lcd.drawRoundRect(xst2, yst0, xsize, ysize, rsize, TFT_CYAN);
    lcd.setCursor(xst2 + xoffset - 10, yst0  + yoffset);
    lcd.setTextColor(TFT_CYAN, BLACK);
    lcd.print("START");
  } else {
    billieflg = true ;
    // lcd.fillRoundRect(xst2, yst0, xsize, ysize, rsize, TFT_CYAN);
    lcd.setCursor(xst2 + xoffset - 10, yst0  + yoffset);
    lcd.setTextColor(TFT_CYAN, BLACK);
    lcd.print("STOP ");
  }
}

void drawPatch()
{
  auto& lcd = M5.Display;

  lcd.fillRect(
    0,
    30,
    320,
    40,
    TFT_BLACK
  );

  lcd.setTextColor(TFT_CYAN, BLACK);
  lcd.setTextSize(2);

  lcd.setCursor(10, 35);
  lcd.printf("BANK:%s", getBankName());

  lcd.fillRect(
    60,
    55,
    200,
    50,
    TFT_BLACK
  );

  lcd.setTextColor(TFT_YELLOW, BLACK);
  lcd.setTextSize(4);

  lcd.setCursor(10, 60);

lcd.printf("%03u", selectedDisplayPatch());

  lcd.setTextSize(2);
  lcd.setCursor(10, 100);

lcd.print(getPatchName(currentPatch));
  lcd.print("                    ");

}

// =====================================================
// UI TASK
// =====================================================

void uiTask(void*)
{

bool touchLatch = false;
  int xmax, ymax ;
  while (true)
  {
    M5.update();

    auto touch = M5.Touch.getDetail();

    if (touch.isPressed())
    {
      if (!touchLatch)
      {
        touchLatch = true;

        int x = touch.x;
        int y = touch.y;

        xmax = xst1 + xsize;
        ymax = yst1 + ysize;

if (x >= xst1 && x <= xmax &&
            y >= yst1 && y <= ymax)
          prevBank();

        xmax = xst2 + xsize;
        ymax = yst1 + ysize;
        if (x >= xst2 && x <= xmax &&
            y >= yst1 && y <= ymax)
          nextBank();

        xmax = xst1 + xsize;
        ymax = yst2 + ysize;
        if (x >= xst1 && x <= xmax &&
            y >= yst2 && y <= ymax )
          patchMinus1();

        xmax = xst2 + xsize;
        ymax = yst2 + ysize;
        if (x >= xst2 && x <= xmax &&
            y >= yst2 && y <= ymax )
          patchPlus1();

        xmax = xst1 + xsize;
        ymax = yst3 + ysize;
        if (x >= xst1 && x <= xmax &&
            y >= yst3 && y <= ymax)
          patchMinus10();

        xmax = xst2 + xsize;
        ymax = yst3 + ysize;
        if (x >= xst2 && x <= xmax &&
            y >= yst3 && y <= ymax)
          patchPlus10();

        xmax = xst2 + xsize;
        ymax = yst0 + ysize;
        if (x >= xst2 && x <= xmax &&
            y >= yst0 && y <= ymax)
          billiejean();
      }
    }
    else
    {
      touchLatch = false;
    }

    if (uiDirty)
    {
      uiDirty = false;
      drawPatch();
    }

scan_keyboard();
    noteget();
    updateEncoderInput();
    adreaddata();
    vTaskDelay(pdMS_TO_TICKS(10));
  }

}
