#ifndef AXLROBOT_H
#define AXLROBOT_H

#include <ESP32Servo.h>
#include <Arduino.h>

class AxlRobot {
public:
    // Costanti
    static const int PORT_COUNT = 10;

    // Porte Dict servi
    //Pin Servi
    static const int FLF_pin = 14;
    static const int FLT_pin = 13;
    static const int FRF_pin = 11;
    static const int FRT_pin = 12;
    static const int RLF_pin = 10;
    static const int RLT_pin = 9;
    static const int RRF_pin = 16;
    static const int RRT_pin = 46;
    static const int P_pin = 3;
    static const int T_pin = 8;

    // Pwm Leg
    // LEG I init
    static const int FLF_init = 0;
    static const int FLF_min = 0;
    static const int FLF_max = 180;
    static const int FLT_init = 0;
    static const int FLT_min = 0;
    static const int FLT_max = 180;
    // LEG II init
    static const int FRF_init = 0;
    static const int FRF_min = 0;
    static const int FRF_max = 180;
    static const int FRT_init = 0;
    static const int FRT_min = 0;
    static const int FRT_max = 180;
    // LEG III init
    static const int RLF_init = 0;
    static const int RLF_min = 0;
    static const int RLF_max = 180;
    static const int RLT_init = 0;
    static const int RLT_min = 0;
    static const int RLT_max = 180;
    // LEG IV init
    static const int RRF_init = 0;
    static const int RRF_min = 0;
    static const int RRF_max = 180;
    static const int RRT_init = 0;
    static const int RRT_min = 0;
    static const int RRT_max = 180;
    // PAN/TILT
    static const int P_init = 0;
    static const int P_min = 0;
    static const int P_max = 180;
    static const int T_init = 0;
    static const int T_min = 0;
    static const int T_max = 180;
    // Direzione servi
    static const int FLF_direction = 1;
    static const int FLT_direction = -1;
    static const int FRF_direction = -1;
    static const int FRT_direction = 1;
    static const int RLF_direction = 1;
    static const int RLT_direction = -1;
    static const int RRF_direction = -1;
    static const int RRT_direction = 1;
    static const int P_direction = 1;
    static const int T_direction = 1;

    // Enumerazioni per le porte
    enum Ports {
        FLF_port = 0,
        FLT_port,
        FRF_port,
        FRT_port,
        RLF_port,
        RLT_port,
        RRF_port,
        RRT_port,
        P_port,
        T_port
    };

    // Costruttore e distruttore
    AxlRobot();
    ~AxlRobot();

    // Metodi pubblici
    void init();
    void setSpeed(int value);
    void savePose();
    void moveAllServos();
    void moveSingleServo(int servoIndex);
    void setLegPosition(int legIndex);
    void saveCommand();
    void setCalibratePose();
    void repose();

private:
    // Variabili private
    int speed;
    int directionDict[PORT_COUNT];
    int pinDict[PORT_COUNT];
    int oldDict[PORT_COUNT];
    int nowDict[PORT_COUNT];
    int goalDict[PORT_COUNT];
    int minDict[PORT_COUNT];
    int maxDict[PORT_COUNT];
    String oldCommand, nowCommand, goalCommand;
    Servo servosLeg[PORT_COUNT];

    // Metodi privati
    void attachServos();
    void initializeDicts();
};

#endif // AXLROBOT_H
