#include "main.h"
using namespace pros;
#ifndef _GLOBALS_
#define _GLOBALS_

namespace glb {

    #define P_RF 14
    #define P_RB 13
    #define P_LF 20
    #define P_LB 15
    #define P_INTAKE 12
    #define P_OPTICAL 21
    #define P_FLYWHEEL1 4
    #define P_FLYWHEEL2 18
    #define P_IDX 8
    #define P_IMU 17
    #define P_EXPANSION 'A'
    #define P_SELECTOR 'H'
    #define P_DIST 10

    Motor RF (P_RF, E_MOTOR_GEARSET_06, 1);
    Motor RB (P_RB, E_MOTOR_GEARSET_06, 1);
    Motor LF (P_LF, E_MOTOR_GEARSET_06);
    Motor LB (P_LB, E_MOTOR_GEARSET_06);
    Motor F1 (P_FLYWHEEL1, E_MOTOR_GEARSET_06, 1);
    Motor F2 (P_FLYWHEEL2, E_MOTOR_GEARSET_06, 1);
    Motor INTAKE (P_INTAKE, E_MOTOR_GEARSET_06, true);
    Motor IDX (P_IDX, E_MOTOR_GEARSET_18); 
    Controller con (E_CONTROLLER_MASTER);
    Optical optical (P_OPTICAL);
    Imu imu(P_IMU);
    ADIDigitalOut expansion(P_EXPANSION);
    ADIDigitalIn button (P_SELECTOR);
    Distance dist (P_DIST);
}
#endif