#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <Servo.h>

// PIN CONFIG:

/*
   PWM: 3, 5, 6, 9, 10, and 11. Provide 8-bit PWM output with the analogWrite() function.
*/

// pin configuration:
const int SV1_PIN = 5;  // servo 1 (door) pin
const int SV2_PIN = 6;  // servo 2 (hand) pin
const int BTN_PIN = 8;  // trigger button pin
const int LE2_PIN = 10; // led 2 (red) pin
const int LE1_PIN = 11; // led 1 (green) pin
const int MRX_PIN = 12; // mp3 RX pin
const int MTX_PIN = 13; // mp3 TX pin


enum State {STATE_NONE, STATE_IDLE, STATE_ATTACK, STATE_DECAY, STATE_SUSTAIN, STATE_RELEASE };  // states enum
const String NAMES[] = {"None", "Idle", "Attack", "Decay", "Sustain", "Release"};               // state names (used for debugging)

const int SV1_ANGLE1 = 64;        // chest closed angle
const int SV1_ANGLE2 = 95;        // chest opened angle
const int SV2_ANGLE1 = 220;       // hand closed angle
const int SV2_ANGLE2 = 20;        // hand opened angle

const int MUSIC_VOLUME = 15;      // music volume
const int MUSIC_DELAY_MIN = 300;  // min music delay
const int MUSIC_DELAY_MAX = 4000; // max music delay

const bool debug = true;          // debugger

State currentState = STATE_IDLE;  // current state
State lastState = STATE_NONE;     // last state
State nextState = STATE_NONE;     // next state after reaching nextStamMS delay
bool changedState = false;        // true if state changed in this iteration
long startMS = millis();          // start time in milliseconds
long stampMS = millis();          // time stamp in milliseconds
long nextStampMS = 0;             // delay in milliseconds until the next time stamp
int iterHigh = 0;                 // HIGH trigger button signal iterations counter
int iterHighTreshold = 500;       // HIGH trigger button threshold, after exceeding we know it is really HIGH
int folderNum = 3;                // mp3 folder number
const int folderNumMax = 3;       // mp3 folder number max
int trackNum = 2;                 // mp3 track number from folderNum
const int trackNumMax = 8;        // mp3 track number max

Servo servo1;
Servo servo2;
SoftwareSerial mySoftwareSerial(MRX_PIN, MTX_PIN); // RX, TX
DFRobotDFPlayerMini player;

void tests() {
  if (debug) {
    int interval = 500;
    logger("Tests Started.");
    delay(interval);
    logger("Door OPEN.");
    servo1.write(SV1_ANGLE2);
    delay(interval);
    logger("Green LED blink.");
    digitalWrite(LE1_PIN, HIGH);
    delay(interval);
    digitalWrite(LE1_PIN, LOW);
    delay(interval);
    logger("Hand OPEN.");
    servo2.write(SV2_ANGLE2);
    delay(interval);
    logger("Red LED blink.");
    digitalWrite(LE2_PIN, HIGH);
    delay(interval);
    digitalWrite(LE2_PIN, LOW);
    delay(interval);
    logger("Hand CLOSE.");
    servo2.write(SV2_ANGLE1);
    delay(interval);
    logger("Door CLOSE.");
    servo1.write(SV1_ANGLE1);
    delay(interval);
    logger("Tests Finished.");
  }
}

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
  //delay(500);
  player.volume(MUSIC_VOLUME);  //Set volume value (0~30).
  //delay(500);
  player.EQ(DFPLAYER_EQ_NORMAL);
  //delay(500);
  player.outputDevice(DFPLAYER_DEVICE_SD);
  //delay(500);
}

void logger(String msg) {
  if (debug) {
    Serial.println(msg);
  }
}

String getState(State state) {
  return NAMES[state];
}

void setup() {
  Serial.begin(115200);
  logger("Setup Started.");
  logger("Initializing Pins...");
  initPins();
  logger("Initializing Audio...");
  initAudio();
  logger("Initializing Servos...");
  initServos();
  logger("Initializing Tests...");
  logger("Setup Finished.");
  //tests();
  changeState(500, STATE_IDLE);
}

void loop() {
  updateInputs();
  updateState();
  updateOutputs();
}

void play(int folderNum, int trackNum) {
  logger("Playing folder " + String(folderNum) + " track " + String(trackNum));
  //play specific mp3 in SD:/15/004.mp3; Folder Name(1~99); File Name(1~255)
  player.playFolder(folderNum, trackNum);
}

bool isBtnOFF() {
  return digitalRead(BTN_PIN) == LOW;
}

bool isBtnON() {
  return digitalRead(BTN_PIN) == HIGH;
}

void changeState(long delayMS, State NS) {
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
  if (getStampMS() >= nextStampMS && nextState != STATE_NONE) {
    // change state after reaching next stamp ms delay
    currentState = nextState;
  }
  changedState = lastState != currentState;
  if (changedState) {
    // notice state change
    logger("State " + getState(currentState));
    lastState = currentState;
  }
}

void updateInputs() {
  if (digitalRead(BTN_PIN) == HIGH) {
    iterHigh++;
    if (iterHigh > iterHighTreshold && (currentState == STATE_IDLE || currentState == STATE_RELEASE)) {
      currentState = STATE_ATTACK;
      nextState = STATE_ATTACK;
    }
  } else {
    iterHigh = 0;
  }
}

void updateOutputs() {
  switch (currentState) {
    case STATE_ATTACK: // opening the box, switched on, starting the music, red led flashing to the music:
      if (changedState) {
        changeState(random(MUSIC_DELAY_MIN, MUSIC_DELAY_MAX), STATE_DECAY);
        digitalWrite(LE1_PIN, LOW);
        digitalWrite(LE2_PIN, HIGH);
        trackNum = random(2, trackNumMax);
        play(folderNum, trackNum);
        servo1.attach(SV1_PIN);
        servo1.write(SV1_ANGLE2);
      }
      digitalWrite(LE2_PIN, getLedBeatState() ? HIGH : LOW);
      break;
    case STATE_DECAY: // opening the hand, switched on, red led strobo
      if (changedState) {
        if (isBtnON()) { // if switch is still ON, than turn it off
          changeState(400, STATE_SUSTAIN);
          servo2.attach(SV2_PIN);
          servo2.write(SV2_ANGLE2);
        } else { // otherwise go to close box state
          changeState(100, STATE_RELEASE);
        }
      }
      digitalWrite(LE2_PIN, getLedBeatState() ? HIGH : LOW);
      break;
    case STATE_SUSTAIN: // closing the hand, switched off, red led strobo
      if (changedState) {
        changeState(300, STATE_RELEASE);
        servo2.write(SV2_ANGLE1);
      }
      digitalWrite(LE2_PIN, getLedStroboState() ? HIGH : LOW);
      break;
    case STATE_RELEASE: // closing the box, switched off, red led strobo
      if (changedState) {
        changeState(200, STATE_IDLE);
        servo1.attach(SV1_PIN);
        servo1.write(SV1_ANGLE1);
      }
      digitalWrite(LE2_PIN, getLedStroboState() ? HIGH : LOW);
      break;
    case STATE_IDLE: // box closed, hand closed, snorring sound, green led constant
    default:
      if (changedState) {
        digitalWrite(LE1_PIN, HIGH);
        digitalWrite(LE2_PIN, LOW);
        play(folderNum, 1);
        servo1.detach();
        servo2.detach();
      }
      // fade in and out the green led
      analogWrite(LE1_PIN, getLedFadeValue());
  }
}
