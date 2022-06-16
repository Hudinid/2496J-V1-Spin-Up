#include "main.h"
#include "global.h"
#include <cmath>

#ifndef _PID_
#define _PID_
using namespace std;
using namespace glb;
using namespace pros;

void spinToBlue() {
    double hue = optical.get_hue(); 

    while(hue < 100 || hue > 200) {
        hue = optical.get_hue();
        INTAKE.move(127);
        delay(5);
    }
    INTAKE.stop();

}

void spinToRed() {
    double hue = optical.get_hue(); 

    while(hue > 80) {
        hue = optical.get_hue();
        INTAKE.move(127);
        delay(5);
    }

    INTAKE.stop();

}

#endif