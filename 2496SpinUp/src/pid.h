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
    #define P_F1 10
    #define P_F2 17


    Motor RF (P_RF, E_MOTOR_GEARSET_06, 1);
    Motor RB (P_RB, E_MOTOR_GEARSET_06, 1);
    Motor LF (P_LF, E_MOTOR_GEARSET_06);
    Motor LB (P_LB, E_MOTOR_GEARSET_06);
    Motor F1(P_F1, E_MOTOR_GEARSET_06);
    Motor F2(P_F2, E_MOTOR_GEARSET_06);
    Motor INTAKE (P_INTAKE, E_MOTOR_GEARSET_18, true); 
    Controller con (E_CONTROLLER_MASTER);

}
#endif