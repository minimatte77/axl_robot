#include <Arduino.h>
#include <ESP32Servo.h>
#include "axlrobot.h"

// Costruttore
AxlRobot::AxlRobot() : speed(100), oldCommand(""), nowCommand(""), goalCommand("") {
    // Inizializza i dizionari e i valori predefiniti
    initializeDicts();
}

// Distruttore
AxlRobot::~AxlRobot() {
    // Logica di pulizia (se necessaria)
}

// Inizializza i servocomandi
void AxlRobot::init() {
    attachServos();
}

// Imposta la velocità
void AxlRobot::setSpeed(int value) {
    speed = value;
}

// Salva la posa attuale
void AxlRobot::savePose() {
    for (int i = 0; i < PORT_COUNT; i++) {
        oldDict[i] = nowDict[i];
        nowDict[i] = goalDict[i];
    }
}

// Muove tutti i servocomandi
void AxlRobot::moveAllServos() {
    for (int i = 0; i < PORT_COUNT; i++) {
        setLegPosition(i);
    }
    savePose();
}

// Muove un singolo servo
void AxlRobot::moveSingleServo(int servoIndex) {
    setLegPosition(servoIndex);
    savePose();
}

// Imposta la posizione di una gamba
void AxlRobot::setLegPosition(int legIndex) {
    if (speed == 100) {
        servosLeg[legIndex].write(goalDict[legIndex]);
    } else {
        int step = (goalDict[legIndex] >= nowDict[legIndex])
                       ? ((goalDict[legIndex] - nowDict[legIndex]) / 100) * speed
                       : -((nowDict[legIndex] - goalDict[legIndex]) / 100) * speed;

        if (step == 0) step = (goalDict[legIndex] >= nowDict[legIndex]) ? 1 : -1;

        for (int pos = nowDict[legIndex]; pos != goalDict[legIndex]; pos += step) {
            servosLeg[legIndex].write(pos);
            delay(10); // Ritardo per simulare il movimento
        }

        servosLeg[legIndex].write(goalDict[legIndex]);
    }
}

// Salva il comando attuale
void AxlRobot::saveCommand() {
    oldCommand = nowCommand;
    nowCommand = goalCommand;
}

// Collega i servocomandi ai pin
void AxlRobot::attachServos() {
    for (int i = 0; i < PORT_COUNT; i++) {
        servosLeg[i].attach(pinDict[i]);
    }
}

void AxlRobot::setCalibratePose() {
    for (int leg = 0; leg < PORT_COUNT; leg++) {
        goalDict[leg] = 90;
    }
    moveAllServos();
    oldCommand = nowCommand = goalCommand = "CALIBRATE";
}

// Inizializza i dizionari
void AxlRobot::initializeDicts() {
    int initValues[PORT_COUNT] = {FLF_init, FLT_init, FRF_init, FRT_init, RLF_init, RLT_init, RRF_init, RRT_init, P_init, T_init};
    int minValues[PORT_COUNT] = {FLF_min, FLT_min, FRF_min, FRT_min, RLF_min, RLT_min, RRF_min, RRT_min, P_min, T_min};
    int maxValues[PORT_COUNT] = {FLF_max, FLT_max, FRF_max, FRT_max, RLF_max, RLT_max, RRF_max, RRT_max, P_max, T_max};
    int directionValues[PORT_COUNT] = {FLF_direction, FLT_direction, FRF_direction, FRT_direction, RLF_direction, RLT_direction, RRF_direction, RRT_direction, P_direction, T_direction};
    int pinValues[PORT_COUNT] = {FLF_pin, FLT_pin, FRF_pin, FRT_pin, RLF_pin, RLT_pin, RRF_pin, RRT_pin, P_pin, T_pin};

    for (int i = 0; i < PORT_COUNT; i++) {
        oldDict[i] = initValues[i];
        nowDict[i] = initValues[i];
        goalDict[i] = initValues[i];
        minDict[i] = minValues[i];
        maxDict[i] = maxValues[i];
        directionDict[i] = directionValues[i];
        pinDict[i] = pinValues[i];
    }
}

void AxlRobot::repose() {
    goalCommand = "REPOSE";
    goalDict[FLF_port] = 170;
    goalDict[FRF_port] = 170;
    moveAllServos();
    goalDict[RLF_port] = 170;
    goalDict[RRF_port] = 170;
    moveAllServos();
    saveCommand();
}

