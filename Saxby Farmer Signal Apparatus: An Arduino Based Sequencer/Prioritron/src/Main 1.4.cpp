

// Definitions
// Mozzi
#define MOZZI_CONTROL_RATE 128  // faster than usual to help smooth MOZZI_CONTROL_RATE adsr interpolation
#define MOZZI_AUDIO_RATE 16384  // 16kHz
#define MOZZI_ANALOG_READ_RESOLUTION 10

#include <ADSR.h>
#include "EadMOD.h"
#include <EEPROM.h>
#include <Main.h>
#include <Mozzi.h>
#include <EventDelay.h>
#include <Oscil.h>
#include <Sample.h>
#include <avr/pgmspace.h>
#include <mozzi_midi.h>
// #include <WaveFolder.h>
#include <ResonantFilter.h>
// #include <mozzi_rand.h>
#include <stdbool.h>
#include <tables/saw512_int8.h>
//#include <tables/sin512_int8.h>
#include <tables/square_analogue512_int8.h>
#include <tables/triangle512_int8.h>
 //#include "WhiteNoise512.h"
#include "Hat.h"
#include "Kick.h"
//#include "Shaker.h"
#include "Snare.h"
#include "data.h"
// #include "Clap.h"
// #include "GHOSTSNARE.h"
// #include "Ghostsnare.h"
// #include "Cow.h"

// hardware (LED and Encoder)
#include <Encoder.h>
#include <FastLED.h>


#define ENCODER_OPTIMIZE_INTERRUPTS  // countermeasure of encoder noise


// Specify Pins
// Rotary Encoder Pin
#define key 8
// button
#define button 5
// 2 potentiometers
#define POT_1 0
#define POT_2 1
// Data Pin for WS2812
#define DATA_PIN 10
// Clock Out
#define CLOCK_OUT 13
// Clock In
#define CLOCK_IN 12

// create Oscilator
Oscil<512, MOZZI_AUDIO_RATE> aOscil(SAW512_DATA);
Oscil<512, MOZZI_AUDIO_RATE> bOscil(SAW512_DATA);
Oscil<512, MOZZI_AUDIO_RATE> cOscil(SAW512_DATA);
Oscil<512, MOZZI_AUDIO_RATE> dOscil(SAW512_DATA);
Oscil <512, MOZZI_CONTROL_RATE> LFO(TRIANGLE512_DATA);

// Create noteDelay for triggering the envelope
EventDelay noteDelay;

// Create the Envelopes
ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> envelopea;
ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> envelopeb;
ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> envelopec;
ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> enveloped;

Ead eadA(MOZZI_CONTROL_RATE); 
 Ead eadB(MOZZI_CONTROL_RATE); 
 Ead eadC(MOZZI_CONTROL_RATE); 
 Ead eadD(MOZZI_CONTROL_RATE); 

ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> envelopefilter;

// Create the 3 samples
//  use: Sample <table_size, update_rate> SampleName (wavetable)

Sample<SNARE_NUM_CELLS, MOZZI_AUDIO_RATE> snare(SNARE_DATA);
Sample<KICK_NUM_CELLS, MOZZI_AUDIO_RATE> kick(KICK_DATA);
Sample<HAT_NUM_CELLS, MOZZI_AUDIO_RATE> hat(HAT_DATA);
// Sample <CLAP_NUM_CELLS, MOZZI_AUDIO_RATE>clap(CLAP_DATA);

// Sample <CLAP_NUM_CELLS, MOZZI_AUDIO_RATE>clap(CLAP_DATA);
//Sample<SHAKER_NUM_CELLS, MOZZI_AUDIO_RATE> shaker(SHAKER_DATA);
// Sample <COW_NUM_CELLS, MOZZI_AUDIO_RATE>cow(COW_DATA);
// Sample <GHOSTSNARE_NUM_CELLS, MOZZI_AUDIO_RATE>ghostsnare(GHOSTSNARE_DATA);

LowPassFilter lpf;

// createWaveFolder not used but left for experiments
// WaveFolder<> wf;

// rotary encoder
Encoder myEnc(2, 3);  // use 3pin 2pin
// Encoder myEnc(3, 2);//use 3pin 2pin for stripboardlayout
signed char oldPosition = 0;
signed char newPosition = 0;

// Create Array for LED matrix
CRGB leds[64];

// bitmaps array contains all bitmaps used for the display (eg wave forms and
// text) They are held in ProgMem
extern const byte bitmaps[33][8];

// variables for mozzi
// These variables used to control gain of oscillator using the envelope output
unsigned int gaina;
unsigned int gainb;
unsigned int gainc;
unsigned int gaind;
long filterenv;

// To allow two note polyphonic output this variable toggles between 0 and 1 on
// every note. aOscil & bOscil play on toggle=0 and cOscil & dOscil play on
// toggle=1

bool toggle = false;
int padCount =0;

// tvariable is used to determine if the notes are "pads"
bool oscillatorFour = 0;
byte padMode = 1;
int offsetLFO = 75;
// In pad mode these variables are used to determine the intervals that make up
// the chords
byte bInterval;
byte cInterval;
byte dInterval;

// variable for hardware

signed char keyPress;
int pot1;
int pot2;

// Assign Pallette for LED colours

long pallette[8] = {
    // 0= red
    0xFF0000,
    // 1= yellow
    0xFFFF00,
    // 2=green
    0x00FF00,
    // 3=blue
    0x0000FF,
    // 4=cyan
    0x00FFFF,
    // 5=magenta
    0xFF00FF,
    // 6=purple
    0x3f00ff,
    // 7=black
    0x000000,
};

// I have found two types of ws2812 matrix and the LEDs are in a different
// sequence. If display is scrambled try changing these. The ledMatrix array is
// used to  address the LEDS in a vertical columns. ledMatrix[x][y] with
// ledMatrix[0][0] being bottom left corner If the matrix is installed the wrong
// way around then orientation could be fixed by adjusting these values

/* This Array for Flexible 8*8 ws2812
byte ledMatrix[8][8]={
{0,15,16,31,32,47,48,63},
{1,14,17,30,33,46,49,62},
{2,13,18,29,34,45,50,61},
{3,12,19,28,35,44,51,60},
{4,11,20,27,36,43,52,59},
{5,10,21,26,37,42,53,58},
{6,9,22,25,38,41,54,57},
{7,8,23,24,39,40,55,56}};*/

// This is the map for hard pcb type 8*8 ws2812 matrix "Better Than Expected"
byte ledMatrix[8][8] = {
    {7, 15, 23, 31, 39, 47, 55, 63}, {6, 14, 22, 30, 38, 46, 54, 62},
    {5, 13, 21, 29, 37, 45, 53, 61}, {4, 12, 20, 28, 36, 44, 52, 60},
    {3, 11, 19, 27, 35, 43, 51, 59}, {2, 10, 18, 26, 34, 42, 50, 58},
    {1, 9, 17, 25, 33, 41, 49, 57},  {0, 8, 16, 24, 32, 40, 48, 56}};

// Variables for UI
// mode determines what menu is currently selected
byte mode = 4;
// menuMode variable used to scroll through menu items
byte menuMode = 1;
// this is a variable used for different things depending on the mode selected
signed char menuSubMode = 0;
//_________________________________________________Setup
//Variables__________________________________________________________ Variables
// for scales and notes

// scaleMode used to switch between Major and Minor Scales
byte scaleMode = 0;
// #define SCALEMODE_MAJOR 0
//  #define SCALEMODE_MINOR 1

// this array changes depending on the scale mode selected. It specifies the
// number of semitones (midi notes) above the tonic note for each note in the
// sequence
byte scaleIntervals[8];
// this array holds the midi note values for the scale
byte scale[8];

// This variable used to determine tonic note for scale (using Midi numbering)
// set to "C" initially
byte tonic = 12;

// seq[8] is an array for the currently playing sequence. Values are the y coord
// for the LED matrix. Value of 0 is off.
signed char seq[8] = {
    1, 0, 1, 0, 1, 0, 1, 0,
};
// octave[8] determines the octave that the value of Seq refers to
byte octave[8] = {1, 3, 2, 3, 3, 3, 4, 3};

// this array holds the sequences as a series of midi notes
byte seqMidi[8];

// number of sequence being played (there are 6 sequences that can be stored in
// the EEPROM), eg the green sequence is 1, yellow is 2
byte currentSequence = 1;
// this is to keep track of button press and release when copying sequences
bool dragBool = false;

// what step of the sequence is currently playing
byte step = 0;
byte oldStep = 0;

//
int startStep = 0;
int endStep = 7;

// comp[24] array is used for the composer/controller function. comp holds the
// values for the different sequences that are played. comp[24]={1,2,1.... would
// play green, yellow, green. The sequences themselves are held in EEPROM

signed char comp[24];
// this used to count where in comp[24] you are.
byte compCounter = 0;

// These are the frequencies of the oscilators
int aFreq;
int bFreq;
int cFreq;
int dFreq;

// drums[8] array is the equivalent of the Seq array but for drums. As more than
// one drum may play at the same time, the drum hits are indicated with binary
// values. eg 01010000 indicates a hit on Snare and HiHat
byte drums[8] = {0b00000001, 0b00010010, 0b00000100, 0b0001000,
                 0b00010000, 0b00100000, 0b10000000, 0b01000000};
// if drumBool = 1 then drums rather than synth plays
bool drumBool = false;

// ADSR Variables
// These are "heights" of the attack and decay
byte attack_level = 250;
byte decay_level = 100;

// These are the duration of the ADSR
int attack = 30;
int decay = 100;
int sustain = 20;
int release_ms = 20;
int scaleNoteCount = 7;

// variables use in ADSR mode to help draw the envelope. Used for scrolling and
// to assign colours to different parts of envelope
byte a;
byte d;
byte s;
byte r;

// Various Counter Variables
// Used in various places to keep track of the cursor (in the sequencer mode and
// drum mode)
int cursorPosition = 0;
int oldCursorPosition = 0;

// cursorMode is 0 for scrolling Left and Right through steps. cursorMode is 1
// for changing notes
bool cursorMode = 0;

// keyStatus is a variable used to test value of both button and the rotary
// encoder key
bool keyStatus = 1;
byte lastkeyStatus = 1;

// used to differentiate short and long clicks
unsigned long sw_timer = 0;

// Variables for waveform Function (do these need to be global variables?)
bool changeWavetable = 1;
byte oldwavetable = 3;
byte wavetable = 3;

// Timing Variables

// This variable determines is unit is conductor or player. If conductor=1 then
// the tempo is controlled by scrolling on the timing menu. If unit is player
// (conductor=0) then then the tempo is controlled by external clock
bool conductor = 1;
// This mode determines how many clock pulses translate into one step.
char timeMode = 4;

#define PULSE_LENGTH 50

// bool firstStep = 0;
bool pulseCheck = 0;

// This variable is used to record the length of the clock pulse. The 7th Pulse
// is longer to allow the sequences to be syncronised (as well as the steps)
unsigned long timeSinceLastClock = 0;

// This variable is used to keep track of the number of clock pulses
signed char readCycle = 0;
// tempo is step length is ms
int tempo = 200;
bool clock = 0;
signed char timeShift = 0;
signed char thisStepTime;
int lastStepTime = 1000;
byte pulseCount = 0;
byte numBar = 1;
byte countBar = 0;

// variables for filter NB these are not Hz but on an arditary 0-255 sized scale
byte resonance = 0;
// byte cutoff=100;
byte filterMode = 1;
bool filterBool = false;
// lpf.setCutoffFreqAndResonance(100,150);

// int lfoutput;

// Variables for Osc
// used to specify the behaviour of second oscillator
byte oscMode = 1;
// used to specify detune amount (sometimes freq, sometimes midi note interval
// depending on mode)
signed char detune = 0;

void setup() {
 // Serial.begin(230400);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, 64);
  // brightness set low to reduce current load on USB
  FastLED.setBrightness(20);
  pinMode(key, INPUT_PULLUP);
  pinMode(button, INPUT_PULLUP);
  pinMode(CLOCK_OUT, OUTPUT);
  pinMode(CLOCK_IN, INPUT);
  noteDelay.set(1000);  // 1 second countdown
  bool clockCheck = 0;
  bool clockCheckOld = 1;
  oscillatorFour = 0;

  // Check on CLOCK_IN for signal. If not found then this unit will become
  // conductor
  for (int i = 0; i < 256; i++) {
    clockCheck = digitalRead(CLOCK_IN);
    if (clockCheck == 1 && clockCheckOld == 0) {
      conductor = 0;
    };
    clockCheckOld = clockCheck;

    // start up animation
    if (i < 65) {
      leds[i] = pallette[random(0, 7)];
    } else {
      leds[random(0, 64)] = pallette[random(0, 7)];
    }
    FastLED.show();
    delay(4);
  }

  for (int i = 0; i < 64; i++) {
    if (conductor) {
      leds[i] = pallette[2];
    } else {
      leds[i] = pallette[5];
    }
    delay(4);
    FastLED.show();
  }

  startMozzi();
  setScaleMode();
  setTonic();
  transpose();
  updateADSR();
  initialiseMenuMode();
  moveCursor();
  // LFO.setFreq(0.3f);
  // wf.setLimits(-255,255);
}

void setScaleMode() {
  // writes the appropriate intervals into scaleIntervals[8]. Call this to
  // switch between major and minor key
  switch (scaleMode) {
    case 0:
      scaleIntervals[1] = 0;
      scaleIntervals[2] = 2;
      scaleIntervals[3] = 3;
      scaleIntervals[4] = 5;
      scaleIntervals[5] = 7;
      scaleIntervals[6] = 8;
      scaleIntervals[7] = 10;
      scaleNoteCount = 8;
      break;

    case 1:
      scaleIntervals[1] = 0;
      scaleIntervals[2] = 2;
      scaleIntervals[3] = 4;
      scaleIntervals[4] = 5;
      scaleIntervals[5] = 7;
      scaleIntervals[6] = 9;
      scaleIntervals[7] = 11;
      scaleNoteCount = 8;
      break;
  }
}

void resetSynth() {
  // this resets the synth variables when exiting the drum mode
oscillatorFour=false;
  drumBool = false;
  attack_level = 250;
  decay_level = 100;

   attack = 30;
    decay = 100;
 sustain = 20;
 release_ms = 20;

  updateADSR();
  aOscil.setTable(SAW512_DATA);
  bOscil.setTable(SAW512_DATA);
  cOscil.setTable(SAW512_DATA);
  dOscil.setTable(SAW512_DATA);
  currentSequence = 1;
}

// these 4 functions write and read the sequences to and from EEPROM


void readSeqFromEEPROM() {for (int j = 0; j < 8; j++) {
  seq[j] = EEPROM.read(currentSequence * 8 + j);
  octave[j] = EEPROM.read(48 + currentSequence * 8 + j);
}}

void writeSeqToEEPROM() {for (int j = 0; j < 8; j++) {
  EEPROM.update(currentSequence * 8 + j, seq[j]);
  EEPROM.update(48 + currentSequence * 8 + j, octave[j]);
}}

void readDrumFromEEPROM(){
  for (int j = 0; j < 8; j++) {
  drums[j] = (byte)EEPROM.read(88 + currentSequence * 8 + j);
}}

void writeDrumToEEPROM(){
  for (int j = 0; j < 8; j++) {
    EEPROM.update(88 + currentSequence * 8 + j, (byte)drums[j]);
  }
}

void setTonic() {
  // update the whole sequence to a new tonic note
  for (int i = 0; i < scaleNoteCount; i++) {
    scale[i] = tonic + scaleIntervals[i];
  }
}

void transpose() {
  // writes the sequence array into midi notes. Needs to be called whenever the
  // sequence changes or key changes

  for (int i = 0; i < 8; i++) {
    if (seq[i] == 0) {
      seqMidi[i] = 0;
    } else {
      seqMidi[i] = scale[seq[i]] + 12 * octave[i];
    }
  }
}

void playNote() {
  // Used to play synth sequences
  // toggle is used to allow some basic polyphony. Oscillator a&b and c&d are
  // used on alternating notes so that two can be played at the same time
  // Frequencies for the notes are assigned to a constant for modulation later

  if (seqMidi[step] != 0) {
    
    if (oscillatorFour) {
      padCount++;
      if (padCount==4){
        padCount=0;
      }
      envelopefilter.noteOn();


switch (padCount)
{
        case 0:
          
          aFreq = (int)mtof(seqMidi[step]);
          aOscil.setFreq(aFreq);
          eadA.start(attack,decay);
          break;
      
        case 1:
        
        bFreq = (int)mtof(seqMidi[step]);
          bOscil.setFreq(bFreq);
          eadB.start(attack,decay);
          break;
        case 2:
        
        cFreq = (int)mtof(seqMidi[step]);
          cOscil.setFreq(cFreq);
          eadC.start(attack,decay);
          break;
        case 3:
       
        dFreq = (int)mtof(seqMidi[step]);
          dOscil.setFreq(dFreq);
          eadD.start(attack,decay);
          break;
      }

      
      
    }
    

    else {
      
      envelopefilter.noteOn();

      if (toggle) {
        envelopea.noteOn();

        if (seqMidi[step] == 0) {
          bFreq = 0;
        }
        // oscMode == 4 allows for 2 oscillators to be detuned by intervals in
        // Hz
        else if (oscMode == 4) {
          bFreq = ((int)mtof(seqMidi[step]) - detune);
        }
        // otherwise detune is used to specify difference between 2 oscillators
        // in semitones (midi notes)
        else
          bFreq = ((int)mtof(seqMidi[step] - detune));
        aFreq = (int)mtof(seqMidi[step]);
        aOscil.setFreq(aFreq);
        bOscil.setFreq(bFreq);
      } else {
        envelopeb.noteOn();

        if (seqMidi[step] == 0) {
          dFreq = 0;
        } else if (oscMode == 4) {
          dFreq = ((int)mtof(seqMidi[step]) - detune * 0.1f);
        } else
          dFreq = ((int)mtof(seqMidi[step] - detune));
        cFreq = (int)mtof(seqMidi[step]);
        cOscil.setFreq(cFreq);
        dOscil.setFreq(dFreq);
      }
    }
  }

  toggle = !toggle;
  noteDelay.start(tempo);
  oldStep = step;
  step++;

  if (step == endStep + 1) {
    padCount = 0;
    step = startStep;
    if (mode == 11) {
      moveCompPlayHead();
    }
  }
  if (step > 7) {
    step = 0;
  };

}

void playDrums() {
  // Used to playDrum sequences
  // these binary operations use a mask to read whether drum should fire on this
  // particular step
  bool snareBool = ((drums[step] >> (6)) & 0x01);
  bool kickBool = ((drums[step] >> (5)) & 0x01);
  bool hatBool = ((drums[step] >> (4)) & 0x01);
  bool ltomBool = ((drums[step] >> (3)) & 0x01);
  bool htomBool = ((drums[step] >> (2)) & 0x01);
  bool cowBool = ((drums[step] >> (1)) & 0x01);
  bool boinkBool = ((drums[step] >> (0)) & 0x01);

  // First 4 drums use the oscillators to produce tuned percussion sounds

  if (boinkBool) {
    aFreq = 200;
    envelopefilter.noteOn();
  }
  if (cowBool) {
    aFreq = pot1 << 2;
    envelopefilter.noteOn();
  }
  if (htomBool) {
    bFreq = pot1 << 2;
    bOscil.setFreq(bFreq);
    envelopea.noteOn();
  }
  // if( ltomBool){bFreq=pot1<<1; cOscil.setFreq(cFreq);envelopeb.noteOn();}

  // Last 3 drums use samples - the limit here is free RAM and program space

  if (hatBool) {
    hat.start();
  }
  if (kickBool) {
    kick.start();
}
  if (snareBool) {
    snare.start();
  }
  if (ltomBool) {
   // shaker.start();
  }

  noteDelay.start(tempo);
  oldStep = step;
  step++;

  if (step == 8) {
    step = 0;
    if (mode == 11) {
      moveCompPlayHead();
    }
  }
}

void changeNote(int inc, int stepNum, bool disp) {
  // When notes in sequence are changed then this function makes sure octave and
  // seq arrays are updated.
  // There is an option not to allow the display to update so that this function
  // could be used for generative music where sequences evolve with small random
  // changes. It is envisaged that to implement this you would probably want to
  // be able to access menus other than the sequencer while these evolutions
  // occur.

  seq[stepNum] = seq[stepNum] + inc;
  if (seq[stepNum] < 0) {
    seq[stepNum] = 7;
    octave[stepNum]--;
    if (octave[stepNum] < 2) {
      octave[stepNum] = 1;
    };
  }
  if (seq[stepNum] > 7) {
    seq[stepNum] = 0;
    octave[stepNum]++;
    if (octave[stepNum] > 7) {
      octave[stepNum] = 7;
    };
  }
  transpose();
  if (disp) {
    displaySeq();
  }
}

void updateADSR() {
  // updates the shape of the envelope. Called at various times but mainly in
  // ADSR mode.
  
 
  envelopefilter.setADLevels(250, 0);
  envelopefilter.setTimes(0, attack + decay, 0, 0);

  if (oscillatorFour) {
    
    //envelopea.setADLevels(attack_level, 0);
    //envelopeb.setADLevels(attack_level, 0);
   // envelopec.setADLevels(attack_level, 0);
    //enveloped.setADLevels(attack_level, 0);
    
    
    //envelopea.setTimes(attack, decay, sustain, release_ms);
   // envelopeb.setTimes(attack, decay, sustain, release_ms);
   // envelopec.setTimes(attack, decay, sustain, release_ms);
   //enveloped.setTimes(attack, decay, sustain, release_ms);

    
  } else {
    envelopea.setTimes(attack, decay, sustain, release_ms);
    envelopeb.setTimes(attack, decay, sustain, release_ms);
    envelopefilter.setTimes(0, attack + decay, 0, 0);
    envelopea.setADLevels(attack_level, decay_level);
  envelopeb.setADLevels(attack_level, decay_level);
  }
}

void displaySeq() {
  // shows the current sequence on the LED matrix
  for (int col = 0; col < 8; col++) {
    colourPixel(col, seq[col]);
  }
  FastLED.show();
}

void colourPixel(int col, int val) {
  // This is used to determine correct colour of the pixel orignally this
  // function was used in several places but it could now be incorporated into
  // displaySeq function as that is the only place it is called
  for (int i = 0; i < 8; i++) {
    if (i == 0) {
      leds[ledMatrix[col][i]] = pallette[currentSequence];
    } else if (i == val) {
      leds[ledMatrix[col][i]] = pallette[octave[col] - 1];
    } else {
      leds[ledMatrix[col][i]] = pallette[7];
    }
  }
}

void displayComp() {
  // Used in controller/composer mode to show the (upto) 24 "bars"

  for (int i = 0; i < 24; i++) {
    byte colourComp = comp[i];
    if (comp[i] == 0) {
      colourComp = 7;
    }
    if (i < 8) {
      leds[ledMatrix[i][7]] = pallette[colourComp];
    } else if (i > 15) {
      leds[ledMatrix[i - 16][3]] = pallette[colourComp];
    } else if (i > 7) {
      leds[ledMatrix[i - 8][5]] = pallette[colourComp];
    }
  }
  moveCompCursor();
}

void displayDrums() {
  // Used to display drum sequences
  for (int col = 0; col < 8; col++) {
    for (int i = 0; i < 8; i++) {
      bool bit =
          (drums[col] >> (7 - i)) & 0x01;  // Shift and mask to extract each bit
      if (i == 0) {
        leds[ledMatrix[col][0]] = pallette[currentSequence];
      } else if (bit) {
        leds[ledMatrix[col][i]] = pallette[i - 1];
      } else {
        leds[ledMatrix[col][i]] = pallette[7];
      }
    }
  }
  FastLED.show();
}

void colourBar(int col, byte colour, int val) {
  //  this is used to draw a vertical colour bar of a particular colour and
  //  lenght at a particular x/col position. Used in ADSR mode.
  for (int i = 1; i < 8; i++) {
    if (i <= val) {
      leds[ledMatrix[col][i]] = pallette[colour];
    } else {
      leds[ledMatrix[col][i]] = pallette[7];
    }
  }
}

void movePlayHead() {
  // move a RED square to indicate step that is playing
  if (step == 0 && cursorPosition == 7) {
    leds[ledMatrix[7][0]] = CRGB::Red;
  } else if (step == 0) {
    leds[ledMatrix[7][0]] = pallette[currentSequence];
  } else if (step - 1 == cursorPosition) {
    leds[ledMatrix[step - 1][0]] = CRGB::Red;
  } else {
    leds[ledMatrix[step - 1][0]] = pallette[currentSequence];
  }
  leds[ledMatrix[step][0]] = CRGB::Black;
  FastLED.show();
}

void moveCursor() {
  // move the cursor

  leds[ledMatrix[oldCursorPosition][0]] = pallette[currentSequence];
  leds[ledMatrix[cursorPosition][0]] = CRGB::Red;
  oldCursorPosition = cursorPosition;
  FastLED.show();
  // step = cursorPosition;
}

void moveCompCursor() {
  if (oldCursorPosition < 8) {
    leds[ledMatrix[oldCursorPosition][6]] = pallette[7];
  } else if (oldCursorPosition > 15) {
    leds[ledMatrix[oldCursorPosition - 16][2]] = pallette[7];
  } else if (oldCursorPosition > 7) {
    leds[ledMatrix[oldCursorPosition - 8][4]] = pallette[7];
  }

  if (menuSubMode < 8) {
    leds[ledMatrix[menuSubMode][6]] = pallette[0];
  } else if (menuSubMode > 15) {
    leds[ledMatrix[menuSubMode - 16][2]] = pallette[0];
  } else if (menuSubMode > 7) {
    leds[ledMatrix[menuSubMode - 8][4]] = pallette[0];
  }

  FastLED.show();
  oldCursorPosition = menuSubMode;
}

void moveCompPlayHead() {
  // wipe the old playHead

  if (compCounter < 8) {
    leds[ledMatrix[compCounter][6]] = pallette[7];
  } else if (compCounter > 15) {
    leds[ledMatrix[compCounter - 16][2]] = pallette[7];
  } else if (compCounter > 7) {
    leds[ledMatrix[compCounter - 8][4]] = pallette[7];
  }

  compCounter++;
  if (compCounter == 25) {
    compCounter = 0;
  }
  currentSequence = comp[compCounter];
  if (currentSequence == 0) {
    currentSequence = comp[0];
    compCounter = 0;
  }

  if (drumBool) {
    readDrumFromEEPROM();
    transpose();
  } else {
    readSeqFromEEPROM();
    transpose();
  }

  if (compCounter < 8) {
    leds[ledMatrix[compCounter][6]] = pallette[2];
  } else if (compCounter > 15) {
    leds[ledMatrix[compCounter - 16][2]] = pallette[2];
  } else if (compCounter > 7) {
    leds[ledMatrix[compCounter - 8][4]] = pallette[2];
  }
  moveCompCursor();

  oldCursorPosition = menuSubMode;
}

void bitmapFromProgmem(byte n, byte colour) {
  if (colour >= 7) {
    colour = colour - 6;
  }
  byte buffer[8];
  // draw the bitmaps here https://xantorohara.github.io/led-matrix-editor/

  memcpy_P(buffer, bitmaps[n], 8);

  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      bool bit = buffer[y] >> (x) & 1;

      if (bit) {
        if (y > 2 && mode == 4) {
          leds[ledMatrix[7 - x][7 - y]] = pallette[0];
        } else {
          leds[ledMatrix[7 - x][7 - y]] = pallette[colour];
        }

      } else if (y > 6) {
        leds[ledMatrix[7 - x][7 - y]] = pallette[currentSequence];
      } else {
        leds[ledMatrix[7 - x][7 - y]] = CRGB::Black;
      }
    }
  }
  FastLED.show();
}

int encoderCheck() {
  // Returns -1 for left turn and +1 for right turn rotary encoder detents are
  // increments of 4 so divide by 4
  //-----------------Rotery encoder read----------------------
  oldPosition = newPosition;
  newPosition = myEnc.read() / 4;

  // If Rot Encoder has moved left
  if (newPosition < oldPosition) {
    oldPosition = newPosition;
    return (-1);
  }
  // If Rot Encoder has moved right
  else if (newPosition > oldPosition) {
    oldPosition = newPosition;
    return (+1);
  }
  // If Rot encoder has not moved
  else
    return (0);
}

byte checkKeyPress(byte pinToTest) {
  // check is key is pressed. Short Press returns 1. Long Press returns 2
  // this function used to check for rotary encoder click and button clicks
  keyStatus = digitalRead(pinToTest);

  if (keyStatus == 1 &&
      lastkeyStatus == pinToTest) {  // the button has just been released
    if (sw_timer + 750 <= millis()) {
      lastkeyStatus = 1;
      return (2);
    }

    else if (sw_timer + 75 <= millis()) {
      lastkeyStatus = 1;
      return (1);
    };
  }

  if (keyStatus == 0 && lastkeyStatus == 1) {  // the button is currently down
    sw_timer = millis();
    lastkeyStatus = pinToTest;
  }

  return (0);
}
// Menu_________________________________________________________________________________________________________________________
void scrollMenuMode(int inc) {
  menuMode = menuMode + inc;
  if (menuMode < 1) {
    menuMode = 9;
  }
  if (menuMode > 9) {
    menuMode = 1;
  }

  bitmapFromProgmem(menuMode + 16, menuMode);
}

void initialiseMenuMode() {
  lastkeyStatus = 1;
  sw_timer = 0;
  mode = 4;
  bitmapFromProgmem(menuMode + 16, menuMode);
  menu();
}

void menu() {
  //  Check if Rot Encoder has moved
  int cursorIncrement = encoderCheck();
  if (cursorIncrement != 0) {
    scrollMenuMode(cursorIncrement);
  }
  // Check for Rot Encoder Key (short and long)
  keyPress = checkKeyPress(key);
  if (keyPress > 0) {
    switch (menuMode) {
      case 1:
        mode = 1;
        if (drumBool) {
          resetSynth();
        }
        initialiseSeqMode();
        break;

      case 2:
        mode = 2;
        if (drumBool) {
          resetSynth();
        }
        initialiseWaveformMode();
        break;

      case 3:
        mode = 3;
        menuSubMode = 0;
        if (drumBool) {
          resetSynth();
        }
        initialiseEnvelopeMode();
        break;

      case 4:
        mode = 5;
        if (drumBool) {
          resetSynth();
        }
        initialiseScaleMode();
        break;

      case 5:
        mode = 7;
        if (drumBool) {
          resetSynth();
        }
        initialiseOscMode();
        break;

      case 6:
        mode = 12;
        initialisePadMode();
        break;

      case 7:
        mode = 8;
        initialiseTimeMode();
        break;

      case 8:
        mode = 9;
        initialiseDrums();
        break;

      case 9:
        mode = 11;
        initialiseCompMode();
        break;
    }
  }
}

// Pad_________________________________________________________________________________________________________________________
//void scrollPadMode(int inc) {
 // menuSubMode = menuSubMode + inc;
 // if (menuSubMode < 1) {
 //   menuSubMode = 7;
 // }
 // if (menuSubMode > 7) {
 //   menuSubMode = 1;
//  }
 // bitmapFromProgmem(menuSubMode + 36, 0);
//}

void initialisePadMode() {
  oscillatorFour = 1;
  updateADSR();
attack = 2000;
decay = 2000;
  lastkeyStatus = 1;
  sw_timer = 0;
  cursorMode = 0;
  menuSubMode = padMode;
  switchPadMode();

  padMenu();
}

void padMenu() {
  //  Check if Rot Encoder has moved
  int cursorIncrement = encoderCheck();
  if (cursorIncrement != 0) {
    padMode = padMode + cursorIncrement;
    if (padMode < 1) {
      padMode = 4;
    }
    if (padMode > 4) {
      padMode = 1;
    }
    switchPadMode();
  }

  keyPress = checkKeyPress(key);
  switch (keyPress) {
    case 1:

   // oscillatorFour = !oscillatorFour;
      break;
    case 2:
      initialiseMenuMode();
      break;
  }
}

void switchPadMode() {
  switch (padMode) {
    
    case 1://RED
  aOscil.setTable(TRIANGLE512_DATA);
    bOscil.setTable(TRIANGLE512_DATA);
    cOscil.setTable(TRIANGLE512_DATA);
    dOscil.setTable(TRIANGLE512_DATA);
    
      break;

    case 2://YELLOW
    aOscil.setTable(TRIANGLE512_DATA);
    bOscil.setTable(TRIANGLE512_DATA);
    cOscil.setTable(TRIANGLE512_DATA);
    dOscil.setTable(TRIANGLE512_DATA);
   
 
 
      break;
    break;

    case 3://GREEN
    aOscil.setTable(SAW512_DATA);
    bOscil.setTable(SAW512_DATA);
    cOscil.setTable(SAW512_DATA);
    dOscil.setTable(SAW512_DATA);
    
      break;

    case 4://BLUE
     aOscil.setTable(TRIANGLE512_DATA);
    bOscil.setTable(SAW512_DATA);
    cOscil.setTable(SQUARE_ANALOGUE512_DATA);
    dOscil.setTable(SQUARE_ANALOGUE512_DATA);
    
      break;

   // case 5://CYAN
   // aOscil.setTable(SQUARE_ANALOGUE512_DATA);
  //  bOscil.setTable(SQUARE_ANALOGUE512_DATA);
   // cOscil.setTable(SQUARE_ANALOGUE512_DATA);
   // dOscil.setTable(SQUARE_ANALOGUE512_DATA);
    
   //   break;

    //case 6://PINK
    //aOscil.setTable(TRIANGLE512_DATA);
   // bOscil.setTable(TRIANGLE512_DATA);
    //cOscil.setTable(TRIANGLE512_DATA);
   // dOscil.setTable(TRIANGLE512_DATA);
    
   //   break;

    
   // case 7://PURPLE
    //aOscil.setTable(SAW512_DATA);
   // bOscil.setTable(SAW512_DATA);
    //cOscil.setTable(SAW512_DATA);
    //dOscil.setTable(SAW512_DATA);
    
    //  break;

  }
  bitmapFromProgmem(32, padMode-1);
  updateADSR();
}

// Composer________________________________________________________________________________________________________________________________
void initialiseCompMode() {
  compCounter = 0;
  lastkeyStatus = 1;
  sw_timer = 0;
  bitmapFromProgmem(31, 7);
  displayComp();
  oldCursorPosition = 0;
  menuSubMode = 0;
  compMode();
}

void compMode() {
  //  byte row;

  // Check if Rot Encoder has moved
  int cursorIncrement = encoderCheck();
  if (cursorIncrement != 0) {
    if (cursorMode) {
      comp[menuSubMode] = comp[menuSubMode] + cursorIncrement;
      if (comp[menuSubMode] > 7) {
        comp[menuSubMode] = 0;
      };
      if (comp[menuSubMode] < 0) {
        comp[menuSubMode] = 7;
      };
      displayComp();
    } else {
      menuSubMode = menuSubMode + cursorIncrement;
      // if cursor goes off one side and add one to row
      if (menuSubMode > 23) {
        menuSubMode = 23;
      }
      if (menuSubMode < 0) {
        menuSubMode = 0;
      }
      moveCompCursor();
    }
  }

  // Check for Rot Encoder Key (short and long)

  keyPress = checkKeyPress(key);
  switch (keyPress) {
    case 1:
      cursorMode = !cursorMode;
      break;
    case 2:
      initialiseMenuMode();
      break;
  }
}

// Sequencer________________________________________________________________________________________________________________________________
void initialiseSeqMode() {
  lastkeyStatus = 1;
  sw_timer = 0;
  displaySeq();
  seqMode();
}
void seqMode() {
  // Check if Rot Encoder has moved
  int cursorIncrement = encoderCheck();
  if (cursorIncrement != 0) {
    if (digitalRead(button) == 0) {
            //if copydrag mode is on then write the current sequence to the EEPROM.
            if (dragBool==false){writeSeqToEEPROM();dragBool = true;}
      currentSequence = currentSequence + cursorIncrement;
      if (currentSequence > 6) {
        currentSequence = 1;
      }
      if (currentSequence == 0) {
        currentSequence = 6;
      }
      displaySeq();
    }

    else if (cursorMode) {
      changeNote(cursorIncrement, cursorPosition, 1);
    }  // Change note needs to restrict note from going above 7 or below 0}

    else {
      cursorPosition = cursorPosition + cursorIncrement;
      // if cursor goes off one side or other then wrap around
      if (cursorPosition > 7) {
        cursorPosition = 0;
      }
      if (cursorPosition < 0) {
        cursorPosition = 7;
      }
      moveCursor();
    }
  }

  // Check for Rot Encoder Key (short and long)
  keyPress = checkKeyPress(key);
  switch (keyPress) {
    case 1:
      cursorMode = !cursorMode;
      break;
    case 2:
      initialiseMenuMode();
      break;
  }
}
// WaveForm__________________________________________________________________________________________________________________________________
void initialiseWaveformMode() {
  changeWavetable = 1;
  oldwavetable = wavetable;
  lastkeyStatus = 1;
  sw_timer = 0;
  bitmapFromProgmem(wavetable - 1, 2);
  waveformMode();
}

void waveformMode() {
  // Check if Rot Encoder has moved
  int waveMenuIncrement = encoderCheck();
  if (waveMenuIncrement != 0) {
    wavetable = wavetable + waveMenuIncrement;
    // wrap around wavetypes when scrolling
    if (wavetable > 4) {
      wavetable = 2;
    }
    if (wavetable < 1) {
      wavetable = 4;
    }
  }

  if (wavetable != oldwavetable) {
    switch (wavetable) {
      case 1:
        // statements
        //if (changeWavetable) {
        //  aOscil.setTable(SIN512_DATA);
        //  bOscil.setTable(SIN512_DATA);
         // cOscil.setTable(SIN512_DATA);
        //  dOscil.setTable(SIN512_DATA);
          // playTestNote();
         // ;
        //  bitmapFromProgmem(0, 2);
       // }
        break;
      case 2:
        // statements
        if (changeWavetable) {
          aOscil.setTable(TRIANGLE512_DATA);
          bOscil.setTable(TRIANGLE512_DATA);
          cOscil.setTable(TRIANGLE512_DATA);
          dOscil.setTable(TRIANGLE512_DATA);
          // playTestNote();

          bitmapFromProgmem(1, 2);
        }
        break;
      case 3:
        // statements
        if (changeWavetable) {
          {
            aOscil.setTable(SAW512_DATA);
            bOscil.setTable(SAW512_DATA);
            cOscil.setTable(SAW512_DATA);
            dOscil.setTable(SAW512_DATA);

            bitmapFromProgmem(2, 2);
          }
          break;
          case 4:
            // statements
            if (changeWavetable) {
              {
                aOscil.setTable(SQUARE_ANALOGUE512_DATA);
                bOscil.setTable(SQUARE_ANALOGUE512_DATA);
                cOscil.setTable(SQUARE_ANALOGUE512_DATA);
                dOscil.setTable(SQUARE_ANALOGUE512_DATA);

                bitmapFromProgmem(3, 2);
              }
              break;
            }
        }
    }
  }
  oldwavetable = wavetable;

  // Check for Rot Encoder Key (short and long)
  keyPress = checkKeyPress(key);
  switch (keyPress) {
    case 1:
      break;
    case 2:
      initialiseMenuMode();
      break;
  }
}
// Envelope___________________________________________________________________________________________________________________________________
void initialiseEnvelopeMode() {
  byte envMap[23];
  byte envColours[23];

  if (attack < 10) {
    a = 1;
    envMap[0] = 7;
  } else if (attack < 30) {
    a = 2;
    envMap[0] = 4;
    envMap[1] = 7;
  } else if (attack < 60) {
    a = 3;
    envMap[0] = 3;
    envMap[1] = 5;
    envMap[2] = 7;
  } else if (attack < 120) {
    a = 4;
    envMap[0] = 2;
    envMap[1] = 4;
    envMap[2] = 6;
    envMap[3] = 7;
  } else if (attack < 320) {
    a = 5;
    envMap[0] = 1;
    envMap[1] = 3;
    envMap[2] = 5;
    envMap[3] = 6;
    envMap[4] = 7;
  } else if (attack < 640) {
    a = 6;
    envMap[0] = 1;
    envMap[1] = 2;
    envMap[2] = 3;
    envMap[3] = 5;
    envMap[4] = 6;
    envMap[5] = 7;
  } else {
    a = 7;
    envMap[0] = 1;
    envMap[1] = 2;
    envMap[2] = 3;
    envMap[3] = 4;
    envMap[4] = 5;
    envMap[5] = 6;
    envMap[6] = 7;
  }

  if (decay < 30) {
    d = 1;
    envMap[a] = 4;
  } else if (decay < 60) {
    d = 2;
    envMap[a] = 6;
    envMap[a + 1] = 4;
  } else if (decay < 120) {
    d = 3;
    envMap[a] = 6;
    envMap[a + 1] = 5;
    envMap[a + 2] = 4;
  } else if (decay < 300) {
    d = 4;
    envMap[a] = 7;
    envMap[a + 1] = 6;
    envMap[a + 2] = 5;
    envMap[a + 3] = 4;
  } else {
    d = 5;
    envMap[a] = 7;
    envMap[a + 1] = 6;
    envMap[a + 2] = 6;
    envMap[a + 3] = 5;
    envMap[a + 4] = 4;
  }

  if (sustain < 100) {
    s = 1;
    envMap[a + d] = 4;
  } else if (sustain < 200) {
    s = 2;
    envMap[a + d] = 4;
    envMap[a + d + 1] = 4;
  } else if (sustain < 400) {
    s = 3;
    envMap[a + d] = 4;
    envMap[a + d + 1] = 4;
    envMap[a + d + 2] = 4;
  } else if (sustain < 800) {
    s = 4;
    envMap[a + d] = 4;
    envMap[a + d + 1] = 4;
    envMap[a + d + 2] = 4;
    envMap[a + d + 3] = 4;
  } else {
    s = 5;
    envMap[a + d] = 4;
    envMap[a + d + 1] = 4;
    envMap[a + d + 2] = 4;
    envMap[a + d + 3] = 4;
    envMap[a + d + 4] = 4;
  }

  if (release_ms < 30) {
    r = 1;
    envMap[a + d + s] = 2;
  } else if (release_ms < 60) {
    r = 2;
    envMap[a + d + s] = 3;
    envMap[a + d + s + 1] = 2;
  } else if (release_ms < 120) {
    r = 3;
    envMap[a + d + s] = 3;
    envMap[a + d + s + 1] = 2;
    envMap[a + d + s + 2] = 1;
  } else if (release_ms < 320) {
    r = 4;
    envMap[a + d + s] = 3;
    envMap[a + d + s + 1] = 2;
    envMap[a + d + s + 2] = 2;
    envMap[a + d + s + 3] = 1;
  } else {
    r = 5;
    envMap[a + d + s] = 3;
    envMap[a + d + s + 1] = 3;
    envMap[a + d + s + 2] = 2;
    envMap[a + d + s + 3] = 2;
    envMap[a + d + s + 4] = 1;
  }

  for (int l = 0; l < 23; l++) {
    if (l < a) {
      envColours[l] = 0;
    } else if (l < a + d) {
      envColours[l] = 1;
    } else if (l < a + d + s) {
      envColours[l] = 2;
    } else if (l < a + d + s + r) {
      envColours[l] = 3;
    } else
      envColours[l] = 7;
  }

  for (int col = 0; col < 8; col++) {
    colourBar(col, envColours[col + menuSubMode], envMap[col + menuSubMode]);
  }
  FastLED.show();
}

void changeADSR(int inc) {
  if (oscillatorFour) {
    inc = inc * 50;
  } else {
    inc = inc * 5;
  }
  if ((cursorPosition + menuSubMode) < a) {
    if (5 > (attack + inc)) {
      attack = 0;
    } else {
      attack = attack + inc;
    }
  } else if ((cursorPosition + menuSubMode) < a + d) {
    if (decay + inc < 5) {
      decay = 0;
    } else {
      decay = decay + inc;
    }
  } else if ((cursorPosition + menuSubMode) < a + d + s) {
    if (sustain + inc < 5) {
      sustain = 0;
    } else {
      sustain = sustain + inc;
    }
  } else {
    if (release_ms + inc < 5) {
      release_ms = 0;
    } else {
      release_ms = release_ms + inc;
    }
  }
  updateADSR();
  initialiseEnvelopeMode();
}

void envelopeMode() {
  // Check if Rot Encoder has moved
  int cursorIncrement = encoderCheck();
  if (cursorIncrement != 0) {
    if (cursorMode) {
      changeADSR(cursorIncrement);
    } else {
      cursorPosition = cursorPosition + cursorIncrement;
      // if cursor goes off one side scroll right
      if (cursorPosition > 7) {
        cursorPosition = 7;
        menuSubMode++;
        if (menuSubMode >= a + d + s + r) {
          menuSubMode = a + d + s + r;
        }
        initialiseEnvelopeMode();
      }
      if (cursorPosition < 0) {
        cursorPosition = 0;
        menuSubMode--;
        if (menuSubMode < 0) {
          menuSubMode = 0;
        }
        initialiseEnvelopeMode();
      }

      moveCursor();
    }
  }

  // Check for Rot Encoder Key (short and long)
  keyPress = checkKeyPress(key);
  switch (keyPress) {
    case 1:
      cursorMode = !cursorMode;
      break;
    case 2:
      initialiseMenuMode();
      break;
  }
}
//(Minor/Major)_________________________________________________________________________________________________________________________
void scrollScaleMode(int inc) {
  menuSubMode = menuSubMode + inc;
  if (menuSubMode < 0) {
    menuSubMode = 1;
  }
  if (menuSubMode > 1) {
    menuSubMode = 0;
  }
  bitmapFromProgmem(menuSubMode + 4, 0);
}

void initialiseScaleMode() {
  lastkeyStatus = 1;
  sw_timer = 0;

  menuSubMode = scaleMode;
  bitmapFromProgmem(scaleMode + 4, 2);
  scaleMenu();
}

void scaleMenu() {
  //  Check if Rot Encoder has moved
  int cursorIncrement = encoderCheck();
  if (cursorIncrement != 0) {
    scrollScaleMode(cursorIncrement);
  }  // Change note needs to restrict note from going above 7 or below 0}

  // Check for Rot Encoder Key (short and long)
  keyPress = checkKeyPress(key);
  switch (keyPress) {
    case 1:
      scaleMode = menuSubMode;
      setScaleMode();
      setTonic();
      transpose();

      bitmapFromProgmem(menuSubMode + 4, 2);

      break;
    case 2:
      mode = 6;
      initialiseTonicMode();
      break;
  }
}
// Tonic_________________________________________________________________________________________________________________________
void scrollTonicMode(int inc) {
  menuSubMode = menuSubMode + inc;
  if (menuSubMode < 1) {
    menuSubMode = 7;
  }
  if (menuSubMode > 7) {
    menuSubMode = 1;
  }
  bitmapFromProgmem(menuSubMode + 5, 0);
}

void initialiseTonicMode() {
  menuSubMode = 3;
  lastkeyStatus = 1;
  sw_timer = 0;

  bitmapFromProgmem(menuSubMode + 5, 2);
  tonicMenu();
}

void tonicMenu() {
  //  Check if Rot Encoder has moved
  int cursorIncrement = encoderCheck();
  if (cursorIncrement != 0) {
    scrollTonicMode(cursorIncrement);
  }  // Change note needs to restrict note from going above 7 or below 0}

  // Check for Rot Encoder Key (short and long)
  keyPress = checkKeyPress(key);
  switch (keyPress) {
    case 1:

      switch (menuSubMode) {
        case 1:
          // A
          tonic = 9;
          break;

        case 2:
          // B
          tonic = 11;
          break;

        case 3:
          // C
          tonic = 12;
          break;

        case 4:
          // D
          tonic = 14;

          break;

        case 5:
          // E
          tonic = 16;
          break;

        case 6:
          // F
          tonic = 17;
          break;

        case 7:
          // G
          tonic = 19;

          break;
      }

      setTonic();
      transpose();

      bitmapFromProgmem(menuSubMode + 5, 2);
      break;
    case 2:
      initialiseMenuMode();
      break;
  }
}

// Osc_________________________________________________________________________________________________________________________
void scrollOscMode(int inc) {
  menuSubMode = menuSubMode + inc;
  if (menuSubMode < 1) {
    menuSubMode = 4;
  }
  if (menuSubMode > 4) {
    menuSubMode = 1;
  }
  bitmapFromProgmem(menuSubMode + 12, 0);
  if (menuSubMode == 4) {
    colourBar(6, 5, detune);
    colourBar(7, 5, detune - 7);
    FastLED.show();
  }
}

void initialiseOscMode() {
 if(oscillatorFour){oscillatorFour = 0; resetSynth();}
  lastkeyStatus = 1;
  sw_timer = 0;
  cursorMode = 0;
  menuSubMode = oscMode;
  bitmapFromProgmem(menuSubMode + 12, 2);
  oscMenu();
}

void oscMenu() {
  //  Check if Rot Encoder has moved
  int cursorIncrement = encoderCheck();
  if (cursorIncrement != 0) {
    if (cursorMode == 1 && menuSubMode == 4) {
      detune = detune + (cursorIncrement);
      colourBar(6, 5, detune);
      colourBar(7, 5, detune - 7);
      FastLED.show();
    } else {
      scrollOscMode(cursorIncrement);
    }
  }  // Change note needs to restrict note from going above 7 or below 0}

  // Check for Rot Encoder Key (short and long)
  keyPress = checkKeyPress(key);

  switch (keyPress) {
    case 1:
      oscMode = menuSubMode;
      bitmapFromProgmem(menuSubMode + 12, 2);
      if (menuSubMode == 4) {
        colourBar(6, 5, detune);
        colourBar(7, 5, detune - 7);
      }
      if (oscMode == 1) {
        detune = 0;
      };
      if (oscMode == 2) {
        detune = 12;
      };
      if (oscMode == 3) {
        detune = 5;
      };
      if (oscMode == 4) {
        if (cursorMode == 1) {
          cursorMode = 0;
        };
        cursorMode = 1;
      }
      break;

    case 2:
      initialiseMenuMode();
      break;
  }
}

// Time_________________________________________________________________________________________________________________________
void scrollTimeMode(int inc) {
  menuSubMode = menuSubMode - inc;
  if (menuSubMode < 1) {
    menuSubMode = 1;
  }
  if (menuSubMode > 4) {
    menuSubMode = 4;
  }
  bitmapFromProgmem(menuSubMode + 25, 0);
}

void initialiseTimeMode() {
  lastkeyStatus = 1;
  sw_timer = 0;
  cursorMode = 0;
  if (conductor) {
    timeMode = 9;
  }
  menuSubMode = timeMode;
  bitmapFromProgmem(menuSubMode + 25, 2);
  timeMenu();
}

void timeMenu() {
  //  Check if Rot Encoder has moved
  int cursorIncrement = encoderCheck();

  if (cursorIncrement != 0) {
    if (conductor) {
      tempo = tempo - cursorIncrement;
    } else {
      scrollTimeMode(cursorIncrement);
    }
  }  // Change note needs to restrict note from going above 7 or below 0}

  // Check for Rot Encoder Key (short and long)
  keyPress = checkKeyPress(key);
  switch (keyPress) {
    case 1:
      if (!conductor) {
        timeMode = menuSubMode;
        bitmapFromProgmem(menuSubMode + 25, 2);
        switch (timeMode) {
          case 1:
            timeShift = 4;
            numBar = 4;
            break;

          case 2:
            timeShift = 2;
            numBar = 2;
            break;

          case 3:
            timeShift = 0;
            numBar = 1;
            break;

          case 4:
            timeShift = -1;
            numBar = 0.5;
            break;
         
        }
      }
      break;

    case 2:
      initialisePhaserMode();
      mode = 13;
      break;
  }
}

void phaserCursor(int x, int start) {
  if (start == 0) {
    if (x > 0) {
      leds[ledMatrix[x - 1][3]] = CRGB::Red;
    }
    leds[ledMatrix[x][3]] = CRGB::Red;
    leds[ledMatrix[x][2]] = CRGB::Red;
    leds[ledMatrix[x][1]] = CRGB::Red;
  }

  if (start == 1) {
    if (x < 7) {
      leds[ledMatrix[x + 1][3]] = CRGB::Green;
    }
    leds[ledMatrix[x][3]] = CRGB::Green;
    leds[ledMatrix[x][2]] = CRGB::Green;
    leds[ledMatrix[x][1]] = CRGB::Green;
  }

  oldCursorPosition = cursorPosition;
  FastLED.show();
  // step = cursorPosition;
}

void initialisePhaserMode() {
  lastkeyStatus = 1;
  sw_timer = 0;
  cursorMode = 1;
  if (conductor) {
    timeMode = 9;
  }
  menuSubMode = 0;
  bitmapFromProgmem(31, 2);
  phaserCursor(startStep, 1);
  phaserCursor(endStep, 0);
}

void phaserMenu() {
  //  Check if Rot Encoder has moved
  int cursorIncrement = encoderCheck();

  if (cursorIncrement != 0) {
    if (menuSubMode == 1) {
      startStep = startStep + cursorIncrement;
      endStep = endStep + cursorIncrement;
      if (startStep < 0) {
        startStep = 7;
      }
      if (startStep > 7) {
        startStep = 0;
      }
      if (endStep < 0) {
        endStep = 7;
      }
      if (endStep > 7) {
        endStep = 0;
      }
    }

    if (menuSubMode == 0) {
      endStep = endStep + cursorIncrement;
      if (endStep < 0) {
        endStep = 7;
      }
      if (endStep > 7) {
        endStep = 0;
      }
    }
    bitmapFromProgmem(31, 2);
    phaserCursor(startStep, 1);
    phaserCursor(endStep, 0);
  }

  // Check for Rot Encoder Key (short and long)
  keyPress = checkKeyPress(key);
  switch (keyPress) {
    case 1:
      if (menuSubMode == 0) {
        menuSubMode = 1;
      } else if (menuSubMode == 1) {
        menuSubMode = 0;
      }
      break;

    case 2:
      initialiseMenuMode();
      break;
  }
}

// Drums_________________________________________________________________________________________________________________________

void initialiseDrums() {
  oscillatorFour = 0;
  drumBool = true;
  currentSequence = 1;
  menuSubMode = 1;
  snare.setFreq((float)SNARE_SAMPLERATE / (float)SNARE_NUM_CELLS);
  // ghostsnare.setFreq((float) GHOSTSNARE_SAMPLERATE / (float)
  // GHOSTSNARE_NUM_CELLS);
  kick.setFreq((float)KICK_SAMPLERATE / (float)KICK_NUM_CELLS);
  hat.setFreq((float)HAT_SAMPLERATE / (float)HAT_NUM_CELLS);
  // clap.setFreq((float) CLAP_SAMPLERATE / (float) CLAP_NUM_CELLS);
  //shaker.setFreq((float)SHAKER_SAMPLERATE / (float)SHAKER_NUM_CELLS);
  //    cow.setFreq((float) COW_SAMPLERATE / (float) COW_NUM_CELLS);
  aFreq = 500;
  bFreq = 200;
  cFreq = 100;
  dFreq = 200;

  // kick
  //  aOscil.setFreq(dFreq);
  // LTOM
  //  bOscil.setFreq(100);
  // HTOM
  //  cOscil.setFreq(500);
  // COW
  //  dOscil.setFreq(500);

  aOscil.setTable(SAW512_DATA);
  bOscil.setTable(TRIANGLE512_DATA);
  cOscil.setTable(SQUARE_ANALOGUE512_DATA);
  dOscil.setTable(SQUARE_ANALOGUE512_DATA);

  envelopea.setADLevels(250, 0);
  envelopeb.setADLevels(250, 0);
  envelopefilter.setADLevels(250, 0);
  envelopea.setTimes(20, 100, 0, 0);
  envelopeb.setTimes(20, 200, 0, 0);
  envelopefilter.setTimes(20, 250, 0, 0);

  lastkeyStatus = 1;
  sw_timer = 0;
  displayDrums();
  drumMode();
}

void changeDrum(int inc, int stepNum, bool disp) {
  displayDrums();
  menuSubMode = menuSubMode + inc;
  if (menuSubMode < 0) {
    menuSubMode = 7;
  }
  if (menuSubMode > 7) {
    menuSubMode = 0;
  }
  leds[ledMatrix[stepNum][menuSubMode]] = pallette[0];
  FastLED.show();
}

void drumMode() {
  // Check if Rot Encoder has moved
  int cursorIncrement = encoderCheck();

  if (cursorIncrement != 0) {
   
    //if button is pressed down then intiate copy/drag mode. 
    if (digitalRead(button) == 0) {
      
      //if copydrag mode is on then write the current sequence to the EEPROM.
      if (dragBool==false){writeDrumToEEPROM();dragBool = true;}
      currentSequence = currentSequence + cursorIncrement;
      if (currentSequence > 6) {
        currentSequence = 1;
      }
      if (currentSequence == 0) {
        currentSequence = 6;
      }
      displayDrums();
      
    }
    else if (cursorMode) {
      changeDrum(cursorIncrement, cursorPosition, 1);
    }  // Change drum needs to restrict note from going above 7 or below 0}
    else {
      cursorPosition = cursorPosition + cursorIncrement;
      // if cursor goes off one side or other then wrap around
      if (cursorPosition > 7) {
        cursorPosition = 0;
      }
      if (cursorPosition < 0) {
        cursorPosition = 7;
      }
      moveCursor();
    }
  }

  // Check for Rot Encoder Key (short and long)
  keyPress = checkKeyPress(key);
  switch (keyPress) {
    case 1:
      if ((cursorMode == 1 && menuSubMode == 0) || (cursorMode == 0)) {
        cursorMode = !cursorMode;
      } else {
        drums[cursorPosition] ^= (1 << (7 - menuSubMode));
        displayDrums();
      }

      break;
    case 2:
      if (cursorMode == 1) {
        cursorMode = !cursorMode;
      } else if (cursorMode == 0) {
        // reset original settings

        initialiseMenuMode();
      }

      break;
  }
}

void updateControl() {
  // first run through whatever mode the menu is on (check encoder and check for
  // key clicks etc)

  switch (mode) {
    case 1:
      seqMode();
      break;
    case 2:
      waveformMode();
      break;
    case 3:
      envelopeMode();
      break;
    case 4:
      menu();
      break;
    case 5:
      scaleMenu();
      break;
    case 6:
      tonicMenu();
      break;
    case 7:
      oscMenu();
      break;
    case 8:
      timeMenu();
      break;
    case 9:
      drumMode();
      break;
    case 11:
      compMode();
      break;
    case 12:
      padMenu();
      break;
    case 13:
      phaserMenu();
      break;
  }
  // This code mutates the sequence - not currently used
  // if (oldStep != step) {oldStep=step; //only evolve once per step

  // combine two random numbers to create a distribution weighted towards the
  // middle
  //       byte randEvolveCheck = rand (1,160);
  //       if (stability<<2>randEvolveCheck){
  //       byte randStep = (rand	(0,4)	+ rand	(0,4));
  //     int randInc = 6-(rand (1,6)+r and(1,6));
  //     changeNote(randInc,randStep,(mode==1));
  //  }
  //}

  // pulseCheck (if the clock is high then pulseCheck is 1, low then pulseCheck
  // is 0) timeSinceLastClock is the time since the last clock pulse went high
  // (start of pulse) clock tempo timeShift timeCheck readCycle firstStep - not
  // actually used??

  // conductor if in conductor mode then send out pulses
  // noteDelay is the timer that is used to time the steps. The duration of the
  // steps is determined by "tempo". When in conductor mode the tempo is set at
  // 200ms on reset and then modified in the timemenu function with the rotary
  // encoder

  if (conductor) {
    if (noteDelay.ready()) {
      movePlayHead();
      if (drumBool) {
        playDrums();
      } else {
        playNote();
      }
      // firstStep=1;
      pulseCheck = 1;
      digitalWrite(CLOCK_OUT, HIGH);
      timeSinceLastClock = millis();

    } else if (step == endStep && pulseCheck == 1 &&
               millis() > (timeSinceLastClock + (PULSE_LENGTH << 1))) {
      digitalWrite(CLOCK_OUT, LOW);
      pulseCheck = 0;
    } else if (step != endStep && pulseCheck == 1 &&
               millis() > (timeSinceLastClock + PULSE_LENGTH)) {
      digitalWrite(CLOCK_OUT, LOW);
      pulseCheck = 0;
    }
  }

  // player

  // noteDelay is used when
  // timeShift determines how the clock in is used to determine the step length.
  // This is set in the timeMenu numBar timeCheck is the

  else {
    clock = digitalRead(CLOCK_IN);  // check for clockin signal
    if (timeShift < 0 && noteDelay.ready()) {
      movePlayHead();
      playNote();
      pulseCount = 0;
    }  // if steps are running faster than the conductor then use noteDelay to
       // trigger notes and pulses to calculate tempo

    if (clock == 1 && pulseCheck == 0) {  // When clock goes high (detect rising edge) this code runs
                  // once everytime a clock pulse is detected
      digitalWrite(CLOCK_OUT, HIGH);  // to allow chaining of synths

      long timeCheck = millis();
      tempo = timeCheck - timeSinceLastClock;  // record gap between pulses
      timeSinceLastClock = timeCheck;          // start timing

      // this code relates to timing modes where the player steps are faster
      // than the clock pulse
      if (timeShift == -1) {
        tempo = tempo >> 1;
      }
      // if (timeShift==-4){tempo = tempo>>2;}

      // this code reltes to timing modes where the player steps are equal or
      // slower than the clock pulse
      pulseCheck = 1;  // to allow rising edge to be detected

      if (pulseCount ==
          timeShift +
              1) {  // this counts timeshift number of steps before playing note
        if (drumBool) {
          movePlayHead();
          playDrums();
          pulseCount = 0;
        } else {
          movePlayHead();
          playNote();
          pulseCount = 0;
        }
      }
      pulseCount++;  // keep counting pulses until you reach the timeShift value

    }

    else if (clock == 0 &&
             pulseCheck ==
                 1) {  // When clock goes low (detect falling edge) this
                       // triggers once at the end of the clock pulse
      digitalWrite(CLOCK_OUT, LOW);  // turn off clock out to allow chaining
      long detectedPulseLength =
          millis() - timeSinceLastClock;  // measure length of pulse
      pulseCheck = 0;  // to allow rising edge/falling edge to be detected
      // if (timeShift==-1){tempo = tempo>>1;}
      // if (timeShift==-4){tempo = tempo>>2;}

      if (detectedPulseLength < (PULSE_LENGTH >> 1)) {
        countBar++;
        if (countBar == numBar) {
          step = 7;
          countBar = 0;
        }  // if the detected pulse is less that double the pulse length then
           // just count
      };
    };
  }

  // this code cycles through checking if the button is pressed and reading the
  // pots to reduce processor load
  readCycle = readCycle + 1;
  if (readCycle == 4) {
    readCycle = 0;
  }

  byte buttonPress;
  switch (readCycle) {
    case 0:

      buttonPress = (checkKeyPress(button));

      if (buttonPress == 1 && dragBool) {

        if (drumBool) {
        writeDrumToEEPROM();
        }

        else {
          writeSeqToEEPROM();
        }
       
        dragBool = false;
      }

      else if (buttonPress == 1) {
        if (mode == 11) {
          compCounter = 24;
        }

        else if (drumBool) {
          
          writeDrumToEEPROM();
          currentSequence++;
          if (currentSequence > 6) {
            currentSequence = 1;
          }
          readDrumFromEEPROM();
            displayDrums();
          

        }

        else {
          writeSeqToEEPROM();
          currentSequence++;
          if (currentSequence > 6) {
            currentSequence = 1;
          }
          readSeqFromEEPROM();
          transpose();
          mode = 1;
          initialiseSeqMode();
        }
      }

      if (buttonPress == 2) {
        if (drumBool) {
          writeDrumToEEPROM();
        }

        else {
          writeSeqToEEPROM();
        }
      }
      break;

    case 1:
     
    if (oscillatorFour){
     
      pot1 = mozziAnalogRead(POT_1);
     // pot1 = map(pot1, 0, 1024, 0, 50);
    
     float LFOFreq = ((pow(pot1,2))*0.00002);
      LFO.setFreq(LFOFreq); // LFO frequency is set by pot1

   }
  

   
  
    else {pot1 = mozziAnalogRead(POT_1);
      pot1 = map(pot1, 0, 1024, 30, 255);
      // filter is turned off in top 10th of the pot1 dial
      if (pot1 > 220) {
        filterBool = false;
      } else {
        filterBool = true;
      }}

      break;

    case 2:
       if (oscillatorFour){pot2 = mozziAnalogRead(POT_2);
        offsetLFO = map(pot2, 0, 1024, 75, 180);

    //if (oscillatorFour){pot2 = mozziAnalogRead(POT_2);
    //  pot2 = map(pot2, 0, 1024, 50, 10000);
    //attack = pot2;
    //  decay = pot2;
  
    

    
    }
    else {
  
    pot2 = mozziAnalogRead(POT_2);
      pot2 = map(pot2, 0, 1024, 0, 255);

    }
      break;

    case 3:
    //do nothing
   break;
  }
  int cutoff2;

  if (oscillatorFour){
        
    
    //envelopea.update();
   // envelopeb.update();
   // envelopec.update();
   // enveloped.update();
    
    
    
    //gaina = envelopea.next(); 
    //gainb = envelopeb.next();
    //gainc = envelopec.next();
    //gaind = enveloped.next();

gaina = eadA.next();
gainb = eadB.next();
gainc = eadC.next();
gaind = eadD.next();


     int LFOutput = LFO.next();
     lpf.setCutoffFreqAndResonance((offsetLFO+(LFOutput>>1)),(250));
     
     envelopefilter.update();
     filterenv = envelopefilter.next(); }


   else

  {envelopea.update();
  envelopeb.update();
  envelopefilter.update();

  gaina = (envelopea.next()>>4);           // envelopes are 8 bit
  gainb = (envelopeb.next()>>4);           // envelopes are 8 bit
  filterenv = envelopefilter.next();
  //____________________________________________SIMPLE FM FOR DRUMS________________________________________________________

  if (drumBool) {
    

    envelopea.update();
    envelopeb.update();
    envelopefilter.update();

    gaina = envelopea.next();           // envelopes are 8 bit
    gainb = envelopeb.next();           // envelopes are 8 bit
    filterenv = envelopefilter.next();  // envelopes are 8 bit

    // HTOM and LTOM
    //  bFreq = bFreq*(1-(gainb*0.0001*pot2/128));
    //"COWBELL"
    aFreq = aFreq * (1 - (filterenv * 0.0001 * (200 - pot2) / 32));
    aOscil.setFreq(aFreq);
    bOscil.setFreq(bFreq);
    cOscil.setFreq(cFreq);
    dOscil.setFreq(dFreq);
  }

  else {
    cutoff2 = ((pot1 + (((254 - pot1) * filterenv * pot2) >> 16))>>1);
   // lpf.setCutoffFreq(cutoff2 >> 1);
    lpf.setCutoffFreqAndResonance(cutoff2,200);
  //  lpf.setResonance(cutoff2 >> 1);
   //   lpf.setResonance(200);
  }
}}

AudioOutput updateAudio() {
  char asig;

 if (oscillatorFour) {
   // asig = (MonoOutput::fromNBit(16, (((gaina * aOscil.next() )>>4) + ((gainb * bOscil.next())>>4) + ((gainc * cOscil.next())>>4) + ((gaind * dOscil.next()>>4))))) ;
    
    
  asig=  MonoOutput::from16Bit(((((gaina*aOscil.next()))>>2)+((gainb*bOscil.next())>>2)+((gainc*cOscil.next())>>2)+((gaind*dOscil.next())>>2)));
    
      //  if (filterBool) {
      //return lpf.next(asig);
    //} else {
      return asig;
   // }
} 



 // if (oscillatorFour) {
 //   asig = (MonoOutput::fromNBit(
 //       16, (gaina * ((aOscil.next() + bOscil.next()) >> 2) +
 //            gainb * ((cOscil.next() + dOscil.next()) >> 2))));
 //   if (filterBool) {
 //     return lpf.next(asig);
 //   } else {
 //     return asig;
 //   }
  //} 
  else if (drumBool) {
    asig = (MonoOutput::from16Bit(
                (int)((gaina * bOscil.next() + gainb * cOscil.next() +
                       filterenv * aOscil.next()))) +
           // shaker.next() 
            + snare.next() + hat.next()+ kick.next());
    return asig;
  } else {
    asig = MonoOutput::fromNBit(
        14, ((gaina * ((aOscil.next() + bOscil.next()))) +
             (gainb * ((cOscil.next() + dOscil.next())))));
    if (filterBool) {
      return lpf.next(asig);
    } else {
      return asig;
    }
  }

  // return asig;
}

void loop() {
  audioHook();  // required here
}
