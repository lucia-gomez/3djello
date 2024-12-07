#include <SparkFun_TB6612.h>  //include sparkfun TB6612 library
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "Adafruit_MPR121.h"
#include <MIDIUSB.h>

#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const int MOTOR_COUNT = 8;

Adafruit_MPR121 cap = Adafruit_MPR121();
uint16_t lasttouched = 0;
uint16_t currtouched = 0;
const int TOUCH_PIN_START = 0;
const int TOUCH_PIN_END = 8;

// int touchBaselines[TOUCH_PIN_END];
// int touchThresholds[TOUCH_PIN_END] = {10, 3};
// bool isTouched[TOUCH_PIN_END];
// int currTouch[TOUCH_PIN_END];

struct MotorState {
  int wiggleStep;               // Current step in the sequence
  unsigned long nextActionTime; // When the next action should occur
  bool active;                  // Is the motor active
};
const long wiggleDurations[] = {2000, 2000, 1000, 1000};
const int numWiggleSteps = sizeof(durations) / sizeof(durations[0]);
MotorState motors[MOTOR_COUNT];

// bool motorActive[MOTOR_COUNT];              // Tracks if a motor is currently active
// unsigned long motorStartTime[MOTOR_COUNT];  // Stores the start time for each motor
// long motorDurations[MOTOR_COUNT];           // Duration for each motor to run

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(1600);
  Wire.setClock(400000);

  if (!cap.begin(0x5A, &Wire, 12, 6)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 found!");

  // delay(1000);
  // for(int i = 0; i < TOUCH_PIN_END; i++) {
  //   touchBaselines[i] = cap.baselineData(i);
  //   isTouched[i] = false;
  // }

  for (int i = 0; i < MOTOR_COUNT; i++) {
    motors[i] = {0, 0, false};
  }

  for (int i = 0; i < MOTOR_COUNT; i++) {
    pwm.setPWM(i * 2, 4096, 0);
    pwm.setPWM(i * 2 + 1, 0, 4096);
    motorActive[i] = false;
  }
  delay(4000);
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pwm.setPWM(i * 2 + 1, 4096, 0);
    pwm.setPWM(i * 2, 0, 4096);
    motorActive[i] = false;
  }
  delay(4000);
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pwm.setPWM(i * 2, 0, 4096);
    pwm.setPWM(i * 2 + 1, 0, 4096);
  }
}

void loop() {
  unsigned long currentTime = millis();
  currtouched = cap.touched();

  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (motors[i].active && currentTime >= motors[i].nextActionTime) {
      processMotor(i);
    }
  }

  for(int i = TOUCH_PIN_START; i < TOUCH_PIN_END; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.println(" touched");
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.println(" released");
      noteOn(5, 60 + (-6 + i*2), 10);
      startMotor(i);
    }
  }
  lasttouched = currtouched;

  delay(20);
}

void processMotor(int motor) {
  MotorState &m = motors[motor];
  switch (m.step) {
    case 0:
    case 2:
      extendMotor(motor);
      break;
    case 1:
    case 3:
      retractMotor(motor);
      break;
    default:
      stopMotor(motor);
      return;
  }
  m.nextActionTime = millis() + durations[m.step];
  m.step++;
}

void startMotor(int motor) {
  motors[motor] = {0, millis(), true}; // Reset sequence
}

void stopMotor(int motor) {
  motors[motor].active = false;
  stopMotor(motor);
}

void extendMotor(int motor, long duration) {
  pwm.setPWM(motor * 2 + 1, 4096, 0);
  pwm.setPWM(motor * 2, 0, 4096);
  
  // motorActive[motor] = true;           // Mark the motor as active
  // motorStartTime[motor] = millis();   // Record the start time
  // motorDurations[motor] = duration;  // Set the duration for the motor
}

void retractMotor(int motor, long duration) {
  pwm.setPWM(motor * 2, 4096, 0);
  pwm.setPWM(motor * 2 + 1, 0, 4096);
  
  // motorActive[motor] = true;           // Mark the motor as active
  // motorStartTime[motor] = millis();   // Record the start time
  // motorDurations[motor] = duration;  // Set the duration for the motor
}

void stopMotor(int motor) {
  pwm.setPWM(motor * 2, 0, 4096);
  pwm.setPWM(motor * 2 + 1, 0, 4096);
}

/**
  MIDI COMMANDS
**/

void sendMIDIStopPlayback() {
  byte controlChangeNumber = 0x41; 
  byte value = 127; 

  midiEventPacket_t midiEvent = {0x0B, 0xB0 | 6, controlChangeNumber, value};
  MidiUSB.sendMIDI(midiEvent);
}

void sendMIDIStartPlayback() {
  byte controlChangeNumber = 0x40; 
  byte value = 127;  

  midiEventPacket_t midiEvent = {0x0B, 0xB0 | 6, controlChangeNumber, value};
  MidiUSB.sendMIDI(midiEvent);
}

void sendCCMessage(byte channel, byte value) {
  byte status = 0xB0;
  midiEventPacket_t midiMsg = {status >> 4, status, channel, value};
  MidiUSB.sendMIDI(midiMsg);
}

// channel is not 0 based
void noteOn(byte channel, byte note, byte velocity) {
  byte cmd = 0x90 + (channel - 1);
  midiEventPacket_t midiMsg = {cmd >> 4, cmd, note, velocity};
  MidiUSB.sendMIDI(midiMsg);
}

// channel is not 0 based
void noteOff(byte channel, byte note, byte velocity) {
  byte cmd = 0x80 + (channel - 1);
  midiEventPacket_t midiMsg = {cmd >> 4, cmd, note, velocity};
  MidiUSB.sendMIDI(midiMsg);
}