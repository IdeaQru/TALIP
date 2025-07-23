#include <Arduino.h>
#include "System/Process.h"
#include "System/ArmSmoother.h"

// Instance smoother
ARM_SMOOTHER armSmoother;

// Inisialisasi di setup atau constructor


#define RXD1 21
#define TXD1 19
// #define RXD1 19
// #define TXD1 22
#define RXD2 16
#define TXD2 17

#define standup 1
#define walk 2
#define atplace 3
#define stop 4
#define  pinBuzzer 13

INSTRUCTION instruction;

void loopingBuzzer() {
    for (int i = 0; i < 10; i++) {
        digitalWrite(pinBuzzer, HIGH);
        delay(100);
        digitalWrite(pinBuzzer, LOW);
        delay(100);
    }
}
// GYRO gyro;
// SYNC sync;
// HWT hwt;
// MOTIONROBOT motionRobot;
// KINEMATIC kinematic;
// KALMAN kalman;
