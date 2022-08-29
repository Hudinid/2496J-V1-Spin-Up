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
    optical.set_led_pwm(25);
    while(hue < 80 || hue > 200) {
        hue = optical.get_hue();
        INTAKE.move(67);
        delay(5);
    }
    INTAKE.move(0);

}

void spinToRed() {
    double hue = optical.get_hue(); 
    optical.set_led_pwm(25);

    while(hue > 70) {
        hue = optical.get_hue();
        INTAKE.move(67);
        delay(5);
    }

    INTAKE.move(0);
}

void spinIndexer(int target, int speed) {
    IDX.move_relative(target, speed);
}

void PIDturn(int target) {

}
#endif