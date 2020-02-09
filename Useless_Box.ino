#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include "IRremote.h"
#include "Servo.h"

// PIN CONFIG:

/*
   PWM: 3, 5, 6, 9, 10, and 11. Provide 8-bit PWM output with the analogWrite() function.
*/

// pin configuration:
const int BTN_PIN = 2;    // trigger button pin
const int SV1_PIN = 5;    // servo 1 (door) pin
const int SV2_PIN = 6;    // servo 2 (hand) pin
const int MRX_PIN = 8;    // mp3 RX pin
const int MTX_PIN = 7;    // mp3 TX pin
const int LE1_PIN = 9;    // led 1 (green) pin
const int LE2_PIN = 10;   // led 2 (red) pin
const int RCV_PIN = 12;   // IR receiver pin

// Remote Controller:

const long REMOTE_KEY_ANY = 0xFFFFFFFF;
const long REMOTE_KEY_UP = 0xFF629D;
const long REMOTE_KEY_DOWN = 0xFFA857;
const long REMOTE_KEY_LEFT = 0xFF22DD;
const long REMOTE_KEY_RIGHT = 0xFFC23D;
const long REMOTE_KEY_NUM_0 = 0xFF4AB5;
const long REMOTE_KEY_NUM_1 = 0xFF6897;
const long REMOTE_KEY_NUM_2 = 0xFF9867;
const long REMOTE_KEY_NUM_3 = 0xFFB04F;
const long REMOTE_KEY_NUM_4 = 0xFF30CF;
const long REMOTE_KEY_NUM_5 = 0xFF18E7;
const long REMOTE_KEY_NUM_6 = 0xFF7A85;
const long REMOTE_KEY_NUM_7 = 0xFF10EF;
const long REMOTE_KEY_NUM_8 = 0xFF38C7;
const long REMOTE_KEY_NUM_9 = 0xFF5AA5;
const long REMOTE_KEY_STAR = 0xFF42BD;
const long REMOTE_KEY_HASH = 0xFF52AD;
const long REMOTE_KEY_OK = 0xFF02FD;

IRrecv irrecv(RCV_PIN);
decode_results results;
long lastPressed;

enum BoxState {BOX_STATE_NONE, BOX_STATE_IDLE, BOX_STATE_ATTACK, BOX_STATE_DECAY, BOX_STATE_SUSTAIN, BOX_STATE_RELEASE };  // states enum
//const String NAMES[] = {"None", "Idle", "Attack", "Decay", "Sustain", "Release"};               // state names (used for debugging)

const int SV1_ANGLE1 = 64;                // chest closed angle
const int SV1_ANGLE2 = 95;                // chest opened angle
const int SV2_ANGLE1 = 220;               // hand closed angle
const int SV2_ANGLE2 = 20;                // hand opened angle

const int MUSIC_VOLUME = 15;              // music volume
const int MUSIC_DELAY_MIN = 300;          // min music delay
const int MUSIC_DELAY_MAX = 4000;         // max music delay

const bool debug = true;                  // debugger

BoxState currentState = BOX_STATE_IDLE;   // current state
BoxState lastState = BOX_STATE_NONE;      // last state
BoxState nextState = BOX_STATE_NONE;      // next state after reaching nextStamMS delay
bool changedState = false;                // true if state changed in this iteration
long startMS = millis();                  // start time in milliseconds
long stampMS = millis();                  // time stamp in milliseconds
long nextStampMS = 0;                     // delay in milliseconds until the next time stamp
int iterHigh = 0;                         // HIGH trigger button signal iterations counter
int iterHighTreshold = 500;               // HIGH trigger button threshold, after exceeding we know it is really HIGH
int folderNum = 1;                        // mp3 folder number
const int folderNumMax = 3;               // mp3 folder number max
int trackNum = 1;                         // mp3 track number from folderNum
const int trackNumMax = 9;                // mp3 track number max

Servo servo1;
Servo servo2;
SoftwareSerial mySoftwareSerial(MRX_PIN, MTX_PIN); // RX, TX
DFRobotDFPlayerMini player;

void initPins() {
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LE1_PIN, OUTPUT);
  pinMode(LE2_PIN, OUTPUT);
  pinMode(SV1_PIN, OUTPUT);
  pinMode(SV2_PIN, OUTPUT);
  digitalWrite(LE1_PIN, LOW);
  digitalWrite(LE2_PIN, LOW);
}

void initServos() {
  servo1.attach(SV1_PIN);
  servo1.write(SV1_ANGLE1);
  servo2.attach(SV2_PIN);
  servo2.write(SV2_ANGLE1);
}

void initAudio() {
  mySoftwareSerial.begin(9600);
  player.begin(mySoftwareSerial);
  delay(3000);
  player.setTimeOut(600); //Set serial communictaion time out 500ms
  player.volume(MUSIC_VOLUME);  //Set volume value (0~30).
  player.EQ(DFPLAYER_EQ_NORMAL);
  player.outputDevice(DFPLAYER_DEVICE_SD);
}

void initRemote() {
  irrecv.enableIRIn(); // Start the receiver
  irrecv.blink13(true);
}

void logger(String msg) {
  if (debug) {
    Serial.println(msg);
  }
}
/*
String getState(State state) {
  return NAMES[state];
}
*/
void setup() {
  Serial.begin(115200);
//  logger("Setup Started.");
//  logger("Initializing Pins...");
  initPins();
//  logger("Initializing Audio...");
  initAudio();
//  logger("Initializing Servos...");
  initServos();
//  logger("Initializing Remote...");
  initRemote();
//  logger("Setup Finished.");
  //tests();
  changeState(500, BOX_STATE_IDLE);
}

void loop() {
  updateInputs();
  updateState();
  updateOutputs();
}

void playMusic(int folderNum, int trackNum) {
//  logger("Playing folder " + String(folderNum) + " track " + String(trackNum));
  //play specific mp3 in SD:/15/004.mp3; Folder Name(1~99); File Name(1~255)
  player.playFolder(folderNum, trackNum);
}

void stopMusic() {
  player.stop();
}

void volumeUp() {
  player.volumeUp();
}

void volumeDown() {
  player.volumeDown();
}

void increaseFolder() {
  folderNum = folderNum < folderNumMax ? folderNum + 1 : 1;
  delay(200);
  playMusic(folderNum, 2);
}

void decreaseFolder() {
  folderNum = folderNum > 1 ? folderNum - 1 : folderNumMax;
  delay(200);
  playMusic(folderNum, 2);
}

bool isBtnOFF() {
  return digitalRead(BTN_PIN) == LOW;
}

bool isBtnON() {
  return digitalRead(BTN_PIN) == HIGH;
}

void changeState(long delayMS, BoxState NS) {
  // create a timestamp
  stampMS = millis();
  nextStampMS = delayMS;
  nextState = NS;
}

long getStampMS() {
  // milliseconds from the last time stamp
  return millis() - stampMS;
}

long getCurrentMS() {
  // milliseconds from the start of the program
  return millis() - startMS;
}

bool getLedBeatState() {
  // 1 measure = 930 ms, 1 beat = 930 ms / 4 = 232 ms, 2 beats = 465 ms
  int measure = 930;
  int halfMeasure = measure / 2;
  int oneBeat = halfMeasure / 2;
  return getStampMS() % (halfMeasure / 2) > (oneBeat / 2);
}

bool getLedStroboState() {
  // 1 measure = 930 ms, 1 beat = 930 ms / 4 = 232 ms, 2 beats = 465 ms
  int measure = 930;
  int halfMeasure = measure / 2;
  int oneBeat = halfMeasure / 2;
  return getStampMS() % (halfMeasure / 6) > (oneBeat / 6);
}

int getLedFadeValue() {
  // output value range: 128-255
  int minAmp = 16;
  int maxAmp = 255;
  int amp = maxAmp - minAmp;
  long val = 0;
  // fullCycle = 1000 ms
  long fc = 3000;
  // halfCycle = 500 ms
  long hc = fc / 2;
  // extract current cycle value
  long ms = getStampMS() % fc;
  if (ms < hc) {
    // fadeIn
    val = minAmp + ms * amp / hc;
  } else {
    // fadeOut
    val = maxAmp - (ms - hc) * amp / hc;
  }
  //logger(String(ms) + " -> " + String(val));
  return val;
}

void updateState() {
  if (getStampMS() >= nextStampMS && nextState != BOX_STATE_NONE) {
    // change state after reaching next stamp ms delay
    currentState = nextState;
  }
  changedState = lastState != currentState;
  if (changedState) {
    // notice state change
//    logger("State " + getState(currentState));
    lastState = currentState;
  }
}

void updateInputs() {
  // remote controller:
  
  if (irrecv.decode(&results)) {
    if (results.value != REMOTE_KEY_ANY) {
      lastPressed = results.value;
    }
    Serial.println(lastPressed, HEX);
    
    switch (lastPressed) {
      case REMOTE_KEY_UP: // increase volume
        volumeUp();
        break;
      case REMOTE_KEY_DOWN: // decrease volume
        volumeDown();
        break;
      case REMOTE_KEY_LEFT: // decrease folder
        decreaseFolder();
        break;
      case REMOTE_KEY_RIGHT: // increase folder
        increaseFolder();
        break;
      case REMOTE_KEY_OK: // stop sound
        stopMusic();
        break;
      case REMOTE_KEY_HASH:
        break;
      case REMOTE_KEY_STAR:
        break;
    }
    
    irrecv.resume(); // Receive the next value
  }
  // trigger button:
  if (digitalRead(BTN_PIN) == HIGH) {
    iterHigh++;
    if (iterHigh > iterHighTreshold && (currentState == BOX_STATE_IDLE || currentState == BOX_STATE_RELEASE)) {
      currentState = BOX_STATE_ATTACK;
      nextState = BOX_STATE_ATTACK;
    }
  } else {
    iterHigh = 0;
  }
}

void updateOutputs() {
  switch (currentState) {
    case BOX_STATE_ATTACK: // opening the box, switched on, starting the music, red led flashing to the music:
      if (changedState) {
        changeState(random(MUSIC_DELAY_MIN, MUSIC_DELAY_MAX), BOX_STATE_DECAY);
        digitalWrite(LE1_PIN, LOW);
        digitalWrite(LE2_PIN, HIGH);
        trackNum = random(3, trackNumMax);
        playMusic(folderNum, trackNum);
        servo1.attach(SV1_PIN);
        servo1.write(SV1_ANGLE2);
      }
      digitalWrite(LE2_PIN, getLedBeatState() ? HIGH : LOW);
      break;
    case BOX_STATE_DECAY: // opening the hand, switched on, red led strobo
      if (changedState) {
        if (isBtnON()) { // if switch is still ON, than turn it off
          changeState(400, BOX_STATE_SUSTAIN);
          servo2.attach(SV2_PIN);
          servo2.write(SV2_ANGLE2);
        } else { // otherwise go to close box state
          changeState(100, BOX_STATE_RELEASE);
        }
      }
      digitalWrite(LE2_PIN, getLedBeatState() ? HIGH : LOW);
      break;
    case BOX_STATE_SUSTAIN: // closing the hand, switched off, red led strobo
      if (changedState) {
        changeState(300, BOX_STATE_RELEASE);
        servo2.write(SV2_ANGLE1);
      }
      digitalWrite(LE2_PIN, getLedStroboState() ? HIGH : LOW);
      break;
    case BOX_STATE_RELEASE: // closing the box, switched off, red led strobo
      if (changedState) {
        changeState(200, BOX_STATE_IDLE);
        servo1.attach(SV1_PIN);
        servo1.write(SV1_ANGLE1);
      }
      digitalWrite(LE2_PIN, getLedStroboState() ? HIGH : LOW);
      break;
    case BOX_STATE_IDLE: // box closed, hand closed, snorring sound, green led constant
    default:
      if (changedState) {
        digitalWrite(LE1_PIN, HIGH);
        digitalWrite(LE2_PIN, LOW);
        playMusic(folderNum, 1);
        servo1.detach();
        servo2.detach();
      }
      // fade in and out the green led
      analogWrite(LE1_PIN, getLedFadeValue());
  }
}
