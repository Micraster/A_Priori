#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Encoder.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include "splash.h"

// --- Display Configuration ---
// Using Software SPI pins for the OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_MOSI 10 
#define OLED_CLK 9
#define OLED_DC 12
#define OLED_CS 13
#define OLED_RESET 11
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// --- Hardware Interface ---
#define MODE_BTN 4    // Button to cycle through Scope/BPM/Euclidean/StepSeq
#define ENC_SW 5      // Encoder click button
#define CLOCK_IN 14   // External trigger input (A0 on some boards)
#define TRIGGER_NEXT_PIN 15 // Auxiliary trigger for pattern chaining
const byte gatePins[] = {6, 7, 8, 16, 17, 18}; // Physical output jacks for 6 channels
Encoder myEnc(2, 3); // Rotary encoder using interrupt-capable pins 2 and 3

// --- Global System State ---
byte systemMode = 1; // Start in BPM mode on boot
long oldEncPos = -999; // Last stored encoder position
bool lastModeBtn = HIGH; // Previous state of mode button for edge detection
bool lastEncSw = HIGH;  // Previous state of encoder switch for click detection
bool lastClockState = LOW; // Previous clock input state for rising-edge detection
uint32_t lastUiRefresh = 0;
const uint32_t uiRefreshInterval = 30; // Refresh screen every 30ms

// --- Oscilloscope & BPM Variables ---
byte cv[128];        // Buffer to store ADC samples for the scope
int scopeParam = 15; // Controls the horizontal time-base of the scope
uint32_t lastPulseMicros = 0;
uint32_t lastPulseDuration = 0;
uint32_t lastBpmValue = 0;

// --- Euclidean Mode Variables ---
// Patterns stored in Flash (PROGMEM) to save RAM
const static byte euc16[17][16] PROGMEM = {
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0}, {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0},
  {1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0}, {1,0,0,1,0,0,1,0,0,1,0,0,1,0,0,0},
  {1,0,0,1,0,1,0,0,1,0,0,1,0,1,0,0}, {1,0,0,1,0,1,0,1,0,0,1,0,1,0,1,0},
  {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0}, {1,0,1,1,0,1,0,1,0,1,1,0,1,0,1,0},
  {1,0,1,1,0,1,0,1,1,0,1,1,0,1,0,1}, {1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,1},
  {1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1}, {1,0,1,1,1,1,0,1,1,1,1,0,1,1,1,1},
  {1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1}, {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
  {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};
byte hits[6] = {4, 4, 5, 3, 2, 8};      // Hits per channel
byte offsets[6] = {0, 2, 0, 8, 3, 9};   // Pattern rotation
byte limits[6] = {16, 16, 16, 16, 16, 16}; // Steps per channel
bool mutes[6] = {0, 0, 0, 0, 0, 0};
byte playing_step[6] = {0, 0, 0, 0, 0, 0};
byte select_ch = 0;                    // Currently selected channel in UI
byte select_menu = 0;                  // Currently selected parameter (Hits/Offset/etc)
uint32_t gateTimer = 0;                // For Euclidean gate length (fixed 20ms)

// --- 6-Channel Step Sequencer Variables ---
uint16_t ch_steps[6] = {0, 0, 0, 0, 0, 0}; // Each channel pattern is a 16-bit bitmap for 16 steps
byte ch_prob[6] = {10, 10, 10, 10, 10, 10}; // Per-channel probability in 10% increments (0..10)
byte playhead = 0; // Current step index from 0..15
byte currentSlot = 0; // Active memory slot: 0=A,1=B,2=C,3=D,4=chain mode
byte pendingSlot = 0; // Slot selected by the encoder before confirming
bool isDirty = false; // True if the current pattern has unsaved edits
bool isEditingSlot = false; // True while choosing a slot rather than editing steps
byte topCursor = 0; // Top-level selector: 0=slot selector, 1..6 channels
bool inChannelEdit = false; // True when editing a specific channel
byte editChannel = 0; // Which channel is currently being edited, if inChannelEdit
byte channelCursor = 0; // 0=channel number, 1..16 steps, 17=probability bar
bool editingProbability = false; // true when the probability bar is being adjusted
bool showSavePrompt = false; // Show overwrite confirmation before switching slots
bool saveChoice = true; // YES/NO choice in the save prompt
byte targetSlot = 0; // Destination slot when switching/pending save
byte chain_steps[16] = {0, 1, 2, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4}; // Sequence of slots in chain mode; 4 means End
byte chain_playhead = 0; // Current position inside the chain_steps sequence
int chain_cursor = -1; // UI cursor inside chain mode; -1 means the start/default position
bool nextTriggerArmed = false; // Armed when an external next trigger is seen
bool chainAutoAdvance = false; // false = wait for trigger, true = advance automatically on loop
byte chainSource = 0; // 0=USER sequencers, 1=TECHNO, 2=HOUSE, 3=DNB
static const char progmem_chainSource_USER[] PROGMEM = "USER";
static const char progmem_chainSource_TECHNO[] PROGMEM = "TECHNO";
static const char progmem_chainSource_HOUSE[] PROGMEM = "HOUSE";
static const char progmem_chainSource_DNB[] PROGMEM = "DNB";
static const char* const chainSourceNames[] PROGMEM = {
  progmem_chainSource_USER,
  progmem_chainSource_TECHNO,
  progmem_chainSource_HOUSE,
  progmem_chainSource_DNB
};

static const char progmem_genreName_TECHNO[] PROGMEM = "TECHNO";
static const char progmem_genreName_HOUSE[] PROGMEM = "HOUSE";
static const char progmem_genreName_DNB[] PROGMEM = "DNB";
static const char* const genreNames[] PROGMEM = {
  progmem_genreName_TECHNO,
  progmem_genreName_HOUSE,
  progmem_genreName_DNB
};

// Genre Preset Data (Slot A, B, C, D for each genre)
// Each row is a Slot. Each column is a channel (BD, SD, HH, etc.)
const uint16_t genrePatterns[3][4][6] PROGMEM = {
  { // Techno
    {0x8080, 0x0000, 0xAAAA, 0x0000, 0x0000, 0x0000},   // Slot A
    {0x8080, 0x0808, 0xAAAA, 0x2222, 0x0000, 0x0000},   // Slot B
    {0x8888, 0x0808, 0xEEEE, 0x2222, 0xA0A0, 0x0505},   // Slot C
    {0x8888, 0x0B6F, 0xFFFF, 0x5555, 0xAAAA, 0x1111}    // Slot D (Fill)
  },
  { // House
    {0x8888, 0x0000, 0x2222, 0x0000, 0x0000, 0x0000},   // Slot A
    {0x8888, 0x0808, 0xAAAA, 0x2222, 0x0000, 0x0000},   // Slot B
    {0x9292, 0x0808, 0xAAAA, 0x2222, 0x4444, 0x2222},   // Slot C
    {0x8080, 0x080A, 0xAAA0, 0x2222, 0xB5B4, 0x242F}    // Slot D (Fill)
  },
  { // DnB
    {0x8020, 0x0808, 0xAAAA, 0x0000, 0x0000, 0x0000},   // Slot A
    {0x8420, 0x0808, 0xAAAA, 0x2222, 0x0020, 0x0002},   // Slot B
    {0x8424, 0x0808, 0xFFFF, 0x2222, 0x2020, 0x1010},   // Slot C
    {0xA08A, 0x097F, 0xAAAA, 0x8888, 0x2222, 0x5555}    // Slot D (Fill)
  }
};



// Euclidean UI Circle geometry
const byte x16[16] = {15,21,26,29,30,29,26,21,15,9,4,1,0,1,4,9};
const byte y16[16] = {0,1,4,9,15,21,26,29,30,29,26,21,15,9,4,1};
const byte gx[6] = {0, 40, 80, 15, 55, 95}; // X-offsets for 6 small circles
const byte gy[6] = {0, 0, 0, 32, 32, 32};    // Y-offsets for 6 small circles

// --- EEPROM Management ---
// Saves 6 channels of 16-bit patterns (12 bytes total) to a specific slot
void savePattern(byte slot) {
  if (slot > 3) return; // Only slots A-D can be saved, not the chain slot
  int addr = slot * 12; // Each slot stores 6 channels x 2 bytes per channel
  for (int i = 0; i < 6; i++) {
    EEPROM.put(addr + (i * 2), ch_steps[i]); // Save the 16-bit pattern for each channel
  }
  isDirty = false; // Pattern is now saved to EEPROM
}

void loadPatternFromSource(byte source, byte slot) {
  if (slot > 3) return;
  if (source == 0) {
    int addr = slot * 12;
    for (int i = 0; i < 6; i++) {
      EEPROM.get(addr + (i * 2), ch_steps[i]); // Load the 16-bit pattern for each channel
    }
  } else {
    for (int i = 0; i < 6; i++) {
      ch_steps[i] = pgm_read_word(&(genrePatterns[source - 1][slot][i]));
    }
  }
  isDirty = false;
}

void handleModeSwitch();
void runScope();
void runBPM();
void runEuclidean();
void run6ChSequencer();
void refreshDisplay();
void drawBpmUI();
void drawEuclideanUI();
void draw6ChUI();
void loadPatternFromSource(byte source, byte slot);
void savePattern(byte slot);
void digitalWriteGate(byte ch, bool state);



// Check for Mode Button press and force an immediate screen update
void handleModeSwitch() {
  bool mBtn = digitalRead(MODE_BTN);
  if (mBtn == LOW && lastModeBtn == HIGH) {
    systemMode = (systemMode + 1) % 4;
    display.clearDisplay();
    if (systemMode == 2) display.setTextSize(1);
    
    // Mode-specific pin setup
    if (systemMode == 0) pinMode(A0, INPUT); 
    else pinMode(CLOCK_IN, INPUT_PULLUP);
    
    refreshDisplay(); // IMMEDIATE UPDATE on mode change
    delay(200); // Debounce
  }
  lastModeBtn = mBtn;
}

// Helper to force a redraw of whatever module is active
void refreshDisplay() {
  switch(systemMode) {
    case 0: break; // Scope handles its own refresh because it's high-speed
    case 1: drawBpmUI(); break;
    case 2: draw6ChUI(); break;
    case 3: drawEuclideanUI(); break;
  }
}

// Helper to handle hardware gate outputs
void digitalWriteGate(byte ch, bool state) {
  digitalWrite(gatePins[ch], state);
}

// --- Mode 0: Oscilloscope ---
void runScope() {
  // --- 1. Proportional Encoder Handling ---
  long newPos = myEnc.read() / 4;
  if (newPos != oldEncPos) {
    bool increasing = (newPos > oldEncPos);
    
    // Proportional Step: Change by 10% of current value
    // We use float math here for precision at low values, then cast back to int.
    // If 10% is less than 1, we ensure it moves by at least 1.
    float step = scopeParam * 0.10;
    if (step < 1.0) step = 1.0; 

    if (increasing) {
      scopeParam = (int)(scopeParam + step);
    } else {
      scopeParam = (int)(scopeParam - step);
    }

    // Constrain to keep within safe timing limits
    // 6 = near-instant sampling, 2000 = very slow (several seconds)
    scopeParam = constrain(scopeParam, 6, 2000);
    
    oldEncPos = newPos;
  }

  // --- 2. High Speed Sampling ---
  for (int i = 0; i < 128; i++) {
    cv[i] = 63 - (analogRead(A0) / 16); 
    // Manual delay based on our parameter
    if (scopeParam > 5) delayMicroseconds((scopeParam - 5) * 10);
  }

  // --- 3. Dynamic Timebase Calculation ---
  display.clearDisplay();
  
  // totalMicros = 128 samples * (ADC overhead + user delay)
  // ADC is ~20us thanks to ADCSRA hack in setup
  uint32_t totalMicros = 128UL * (20UL + (uint32_t)(scopeParam - 5) * 10UL);
  
  display.setCursor(0, 0);
  if (totalMicros >= 1000000UL) {
    // Show in Seconds (s)
    display.print(totalMicros / 1000000.0, 2); 
    display.print(" s");
  } else {
    // Show in Milliseconds (ms)
    display.print(totalMicros / 1000.0, 1);
    display.print(" ms");
  }

  // --- 4. Render Waveform ---
  for (int i = 0; i < 127; i++) {
    display.drawLine(i, cv[i], i + 1, cv[i + 1], WHITE);
  }
  
  display.display();
}

// --- Mode 1: BPM counter ---
void runBPM() {
  bool clock = digitalRead(CLOCK_IN);
  if (clock == HIGH && lastClockState == LOW) {
    uint32_t m = micros();
    uint32_t dur = m - lastPulseMicros;
    lastPulseMicros = m;
    if (dur > 0) {
      lastPulseDuration = dur;
      lastBpmValue = 60000000UL / dur;
      drawBpmUI();
    }
  }
  lastClockState = clock;
}

void drawBpmUI() {
  display.clearDisplay();

  char clockLabel[20];
  snprintf(clockLabel, sizeof(clockLabel), "CLOCK=%lums", lastPulseDuration / 1000);
  display.setTextSize(1);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(clockLabel, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 18);
  display.print(clockLabel);

  char bpmLabel[16];
  snprintf(bpmLabel, sizeof(bpmLabel), "%lu BPM", lastBpmValue / 4);
  display.setTextSize(2);
  display.getTextBounds(bpmLabel, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 30);
  display.print(bpmLabel);

  display.display();
}

// --- Mode 2: Euclidean Rhythms ---
void runEuclidean() {
  bool needsRedraw = false;
  
  // Menu Navigation
  bool eSw = digitalRead(ENC_SW);
  if (eSw == LOW && lastEncSw == HIGH) {
    if (select_menu == 5) { 
       for (int r = 0; r < 6; r++) playing_step[r] = 0; // Reset all heads
       select_menu = 0; 
    } else {
       select_menu = (select_menu + 1) % 6;
    }
    needsRedraw = true;
   // delay(200);
  }
  lastEncSw = eSw;

  // Parameter Adjustment
  long newPos = myEnc.read() / 4;
  if (newPos != oldEncPos) {
    int dir = (newPos > oldEncPos) ? 1 : -1;
    switch(select_menu) {
      case 0: select_ch = constrain(select_ch + dir, 0, 5); break;
      case 1: hits[select_ch] = constrain(hits[select_ch] + dir, 0, 16); break;
      case 2: offsets[select_ch] = constrain(offsets[select_ch] + dir, 0, 15); break;
      case 3: limits[select_ch] = constrain(limits[select_ch] + dir, 1, 16); break;
      case 4: mutes[select_ch] = !mutes[select_ch]; break;
    }
    oldEncPos = newPos;
    needsRedraw = true;
  }

  // Trigger Logic
  bool clock = digitalRead(CLOCK_IN);
  if (clock == HIGH && lastClockState == LOW) {
    gateTimer = millis();
    for (int k = 0; k < 6; k++) {
      playing_step[k] = (playing_step[k] + 1) % limits[k];
      byte pIdx = (playing_step[k] + offsets[k]) % 16;
      // Read binary hit from PROGMEM pattern table
      if (pgm_read_byte(&(euc16[hits[k]][pIdx])) && !mutes[k]) {
        digitalWriteGate(k, HIGH);
      }
    }
    needsRedraw = true;
  }
  lastClockState = clock;

  // Gate turn-off (Fixed 20ms pulse)
  if (millis() - gateTimer > 20) {
    for(int i=0; i<6; i++) digitalWriteGate(i, LOW);
  }
  if (needsRedraw) drawEuclideanUI();
}

void drawEuclideanUI() {
  display.clearDisplay();
  display.setTextSize(1);
  // Sidebar info
  display.setCursor(110, 2);  display.print(select_menu == 0 ? ">" : " "); display.print(select_ch + 1);
  display.setCursor(110, 12); display.print(select_menu == 1 ? ">H" : " H");
  display.setCursor(110, 22); display.print(select_menu == 2 ? ">O" : " O");
  display.setCursor(2, 34); display.print(select_menu == 3 ? ">L" : " L");
  display.setCursor(2, 44); display.print(select_menu == 4 ? ">M" : " M");
  display.setCursor(2, 54); display.print(select_menu == 5 ? ">R" : " R");

  // Draw 6 circular visualizers
  for (int k = 0; k < 6; k++) {
    int firstX = -1, firstY = -1, prevX = -1, prevY = -1;
    for (int s = 0; s < limits[k]; s++) {
      int x = x16[s] + gx[k];
      int y = y16[s] + gy[k];
      display.drawPixel(x, y, WHITE);
      byte isHit = pgm_read_byte(&(euc16[hits[k]][(s + offsets[k]) % 16]));
      if (isHit) {
        if (firstX == -1) { firstX = x; firstY = y; }
        if (prevX != -1) display.drawLine(prevX, prevY, x, y, WHITE);
        prevX = x; prevY = y;
      }
      if (s == playing_step[k]) {
        if (mutes[k]) display.drawRect(x-1, y-1, 3, 3, WHITE);
        else if (isHit) display.fillCircle(x, y, 2, WHITE);
        else display.drawCircle(x, y, 2, WHITE);
      }
    }
    if (prevX != -1 && firstX != -1 && hits[k] > 1) display.drawLine(prevX, prevY, firstX, firstY, WHITE);
  }
  display.display();
}

// --- Mode 3: 6-Channel Step Sequencer ---
void run6ChSequencer() {
  bool needsRedraw = false; // Track whether the OLED needs to be refreshed
  display.setTextSize(1);

  // Read clock input first so timing is handled before slower UI/encoder work
  bool clock = digitalRead(CLOCK_IN);
  bool clockRising = (clock == HIGH && lastClockState == LOW);
  bool clockFalling = (clock == LOW && lastClockState == HIGH);

  // If chain mode is waiting for an external trigger, arm it when the trigger line goes HIGH
  if (!chainAutoAdvance && digitalRead(TRIGGER_NEXT_PIN) == HIGH) nextTriggerArmed = true;

  // Read encoder position, divide by 4 to reduce sensitivity/noise
  long newPos = myEnc.read() / 4;

  // --- Save prompt submenu ---
  if (showSavePrompt) {
    // Rotate encoder toggles YES/NO
    if (newPos != oldEncPos) {
      saveChoice = !saveChoice;
      oldEncPos = newPos;
      needsRedraw = true;
    }
    // Press encoder to confirm choice
    if (digitalRead(ENC_SW) == LOW && lastEncSw == HIGH) {
      if (saveChoice) savePattern(currentSlot); // Save current pattern before switching
      currentSlot = targetSlot;
      pendingSlot = targetSlot;
      loadPatternFromSource(chainSource, currentSlot); // Load the newly selected source-specific pattern
      showSavePrompt = false;
      needsRedraw = true;
    }
  }
  // --- Chain mode submenu ---
  else if (currentSlot == 4) {
    // Encoder moves the chain cursor through the source selector, start position, and 16 chain positions
    if (newPos != oldEncPos) {
      int dir = (newPos > oldEncPos) ? 1 : -1;
      chain_cursor = constrain(chain_cursor + dir, -2, 15);
      oldEncPos = newPos;
      needsRedraw = true;
    }
    // Encoder press changes the selected chain step or source selector
    if (digitalRead(ENC_SW) == LOW && lastEncSw == HIGH) {
      if (chain_cursor == -2) {
        chainSource = (chainSource + 1) % 4;
        byte slotToLoad = chain_steps[chain_playhead];
        if (slotToLoad < 4) {
          loadPatternFromSource(chainSource, slotToLoad);
        }
      } else if (chain_cursor == -1) {
        // Special case: select slot A and exit chain editor
        currentSlot = 0;
        pendingSlot = 0;
        loadPatternFromSource(chainSource, currentSlot);
      } else {
        // Cycle the targeted chain position through A, B, C, D, End
        chain_steps[chain_cursor] = (chain_steps[chain_cursor] + 1) % 5;
      }
      needsRedraw = true;
    }
  }
  // --- Standard step editing mode ---
  else {
    bool encSw = digitalRead(ENC_SW);
    bool encSwPressed = (encSw == LOW && lastEncSw == HIGH);

    if (isEditingSlot && topCursor != 0) {
      isEditingSlot = false;
    }

    if (newPos != oldEncPos) {
      int dir = (newPos > oldEncPos) ? 1 : -1;
      if (isEditingSlot && topCursor == 0) {
        pendingSlot = (pendingSlot + dir + 5) % 5; // Cycle through A, B, C, D, chain slot
      } else if (inChannelEdit) {
        if (channelCursor == 17 && editingProbability) {
          ch_prob[editChannel] = constrain(ch_prob[editChannel] + dir, 0, 10);
        } else {
          channelCursor = constrain(channelCursor + dir, 0, 17); // 0=channel number, 1..16 steps, 17=probability
        }
      } else {
        topCursor = constrain(topCursor + dir, 0, 6); // Move between slot selector and 6 channels
      }
      oldEncPos = newPos;
      needsRedraw = true;
    }

    if (encSwPressed) {
      if (inChannelEdit) {
        if (channelCursor == 0) {
          inChannelEdit = false;
          editingProbability = false;
          topCursor = editChannel + 1;
        } else if (channelCursor <= 16) {
          byte step = channelCursor - 1;
          ch_steps[editChannel] ^= (1 << (15 - step)); // Toggle the selected step
          isDirty = true;
          editingProbability = false;
        } else {
          editingProbability = !editingProbability;
        }
      } else if (topCursor == 0) {
        if (!isEditingSlot) {
          isEditingSlot = true;
          pendingSlot = currentSlot;
        } else {
          isEditingSlot = false;
          if (pendingSlot != currentSlot) {
            targetSlot = pendingSlot;
            if (isDirty && currentSlot < 4 && chainSource == 0) {
              // Unsaved user pattern changes exist, ask before switching slots
              showSavePrompt = true;
            } else {
              currentSlot = targetSlot;
              if (currentSlot < 4) loadPatternFromSource(chainSource, currentSlot);
            }
          }
        }
      } else {
        inChannelEdit = true;
        editingProbability = false;
        editChannel = topCursor - 1;
        channelCursor = 0;
      }
      needsRedraw = true;
    }
  }

  // Always capture encoder switch state for edge detection on the next loop
  lastEncSw = digitalRead(ENC_SW);

  // --- Playback logic triggered on the rising edge of the clock input ---
  if (clockRising) {
    playhead = (playhead + 1) % 16; // Advance to the next step and wrap every 16 steps

    // When the pattern loops in chain mode, advance the chain sequence automatically
    if (playhead == 0) {
      if (currentSlot == 4) {
        chain_playhead++;
        if (chain_playhead >= 16 || chain_steps[chain_playhead] == 4) {
          chain_playhead = 0; // Loop chain or stop at End marker
        }
        byte slotToLoad = chain_steps[chain_playhead];
        // Keep the menu in chain mode while loading the next chained slot for playback.
        if (slotToLoad < 4) {
          pendingSlot = slotToLoad;
          loadPatternFromSource(chainSource, slotToLoad);
        }
        nextTriggerArmed = false; // Consume the external trigger if it was set
      } else if ((currentSlot >= 0 && currentSlot <= 3) && nextTriggerArmed) {
        // Normal slots A-D advance to the next chain sequence only when a trigger was detected.
        chain_playhead++;
        if (chain_playhead >= 16 || chain_steps[chain_playhead] == 4) {
          chain_playhead = 0;
        }
        byte slotToLoad = chain_steps[chain_playhead];
        if (slotToLoad < 4) {
          currentSlot = slotToLoad;
          pendingSlot = slotToLoad;
          loadPatternFromSource(chainSource, slotToLoad);
        }
        nextTriggerArmed = false;
      }
    }

    // Fire gate outputs according to the current pattern and probability values
    if (!showSavePrompt) {
      for (int i = 0; i < 6; i++) {
        bool isHit = bitRead(ch_steps[i], 15 - playhead); // Step 0 uses MSB, step 15 uses LSB
        bool fire = false;
        if (isHit) {
          if (ch_prob[i] >= 10) fire = true;
          else if (ch_prob[i] > 0) fire = (random(10) < ch_prob[i]);
        }
        digitalWriteGate(i, fire ? HIGH : LOW);
      }
    }
    needsRedraw = true;
  }

  // When clock falls, turn all gates off once so pulses only occur on the clock edge
  if (clockFalling) {
    for (int i = 0; i < 6; i++) digitalWriteGate(i, LOW);
  }
  lastClockState = clock;

  if (needsRedraw && (millis() - lastUiRefresh >= uiRefreshInterval)) {
    lastUiRefresh = millis();
    draw6ChUI();
  }
}

void draw6ChUI() {
  display.clearDisplay();
  display.setTextSize(1);

  // Save prompt screen shows YES/NO before overwriting a pattern slot
  if (showSavePrompt) {
    display.setCursor(10, 20);
    display.print("Save change to "); display.print((char)('A' + currentSlot)); display.print("?");
    display.setCursor(35, 40);
    if (saveChoice) {
      display.setTextColor(BLACK, WHITE);
      display.print(" YES ");
    } else {
      display.setTextColor(WHITE);
      display.print(" YES ");
    }
    display.setTextColor(WHITE);
    display.print("  ");
    if (!saveChoice) {
      display.setTextColor(BLACK, WHITE);
      display.print(" NO ");
    } else {
      display.setTextColor(WHITE);
      display.print(" NO ");
    }
  }
  // Chain mode screen shows up to 16 chain steps and the selected slot for each
  else if (currentSlot == 4) {
    display.setCursor(0, 0);
    if (chain_cursor == -1) display.setTextColor(BLACK, WHITE);
    display.print("# CHAIN MODE");
    display.setTextColor(WHITE);
    if (chain_cursor == -2) {
      display.fillRect(74, 0, 54, 8, WHITE);
      display.setTextColor(BLACK);
    }
    display.setCursor(76, 0);
    {
      const char* ptr = (const char*)pgm_read_word(&(chainSourceNames[chainSource]));
      display.print((const __FlashStringHelper*)ptr);
    }
    display.setTextColor(WHITE);

    for (int i = 0; i < 16; i++) {
      int x = (i % 4) * 30 + 10;
      int y = (i / 4) * 12 + 15;
      if (chain_cursor == i) display.drawRect(x - 2, y - 2, 24, 12, WHITE);
      if (chain_playhead == i) {
        display.fillRect(x - 1, y - 1, 22, 10, WHITE);
        display.setTextColor(BLACK);
      } else {
        display.setTextColor(WHITE);
      }
      display.setCursor(x, y);
      display.print(i + 1);
      display.print(":");
      if (chain_steps[i] < 4) display.print((char)('A' + chain_steps[i]));
      else display.print("-"); // End marker or unused chain entry
      display.setTextColor(WHITE);
    }
  }
  // Main step editor grid for slots A-D
  else {
    const char slotChars[5] = {'A', 'B', 'C', 'D', '#'};
    int selectedSlot = isEditingSlot ? pendingSlot : currentSlot;
    int lx = 2;
    if (!isEditingSlot && topCursor == 0) {
      display.fillRect(0, 0, 14, 9, WHITE);
      display.setTextColor(BLACK);
    } else if (isEditingSlot) {
      display.fillRect(0, 0, 14, 9, WHITE);
      display.setTextColor(BLACK);
    } else {
      display.setTextColor(WHITE);
    }
    display.setCursor(lx, 0);
    display.print(slotChars[selectedSlot]);
    display.setTextColor(WHITE);

    // Draw a small timeline bar showing 16 step positions
    for (int s = 0; s < 16; s++) {
      int sx = 17 + (s * 6);
      display.drawPixel(sx + 2, 2, WHITE);
      if (s % 4 == 0) display.drawFastVLine(sx + 2, 0, 4, WHITE);
    }
    display.fillRect(17 + (playhead * 6), 1, 5, 2, WHITE); // Current playhead indicator

    // Draw the 6-channel step grid and top-level/channel edit indicators
    for (int ch = 0; ch < 6; ch++) {
      int y = 8 + (ch * 9);
      bool channelTopSelected = (!inChannelEdit && topCursor == ch + 1);
      bool channelNumberSelected = (inChannelEdit && editChannel == ch && channelCursor == 0);
      if (channelTopSelected || channelNumberSelected) {
        display.drawRect(8, y - 1, 10, 7, WHITE);
      }
      display.setCursor(10, y);
      display.print(ch + 1);

      for (int s = 0; s < 16; s++) {
        int x = 17 + (s * 6);
        bool stepActive = bitRead(ch_steps[ch], 15 - s);
        if (stepActive) {
          display.fillRect(x, y, 5, 5, WHITE); // Step is active
        } else {
          display.drawRect(x, y, 5, 5, WHITE); // Step is inactive
        }
        if (inChannelEdit && editChannel == ch && channelCursor == s + 1) {
          display.drawRect(x - 1, y - 1, 7, 7, WHITE);
        }
      }

      int probX = 114;
      int barWidth = 12;
      int fillWidth = map(ch_prob[ch], 0, 10, 0, barWidth);
      display.drawRect(probX, y, barWidth, 5, WHITE);
      if (fillWidth > 0) display.fillRect(probX, y, fillWidth, 5, WHITE);
      if (inChannelEdit && editChannel == ch && channelCursor == 17) {
        display.drawRect(probX - 1, y - 1, barWidth + 2, 7, WHITE);
      }
    }
  }
  display.display();
}

void showStartupScreen() {
  display.clearDisplay();
  display.drawBitmap(0, 0, splashBitmap, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
  display.display();
  delay(2500);
}

void setup() {
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
  for(;;); // Don't proceed, loop forever if screen fails
}
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  showStartupScreen();
  
  pinMode(MODE_BTN, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(CLOCK_IN, INPUT_PULLUP);
  pinMode(TRIGGER_NEXT_PIN, INPUT);
  randomSeed(analogRead(A0));
  
  for(int i=0; i<6; i++) pinMode(gatePins[i], OUTPUT);
  
  // Speed up Analog-to-Digital converter for the Scope mode
  ADCSRA = (ADCSRA & 0xf8) | 0x04; 
  loadPatternFromSource(0, currentSlot);
  
  // Initial draw so the screen isn't black on boot
  refreshDisplay();
}

void loop() {
  handleModeSwitch();
  
  // Dispatch to the active module logic
  switch(systemMode) {
    case 0: runScope(); break;
    case 1: runBPM();   break;
    case 2: run6ChSequencer(); break;
    case 3: runEuclidean(); break;
  }
}