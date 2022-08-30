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

void PIDturn(int target, int currHeading) { // 45, 90 
    double kP = 0.2;
    double kI = 0.1;
    double kD = 0;
    int integral = 0;
    
    target = target - currHeading;
    target += 180;
    double curr = imu.get_rotation();
    double error = target - curr;
    int power;
    int prevError = 0;
    int count = 0; 
    int count2 = 0;
    con.clear();
    while( abs(error) > 1.5 ) {
        curr = imu.get_rotation();
        error = target - curr;
        power = kP * error + kI * integral + kD * prevError;
        prevError = error;
        if(count % 50 == 0) {
            con.print(0, 0, "Heading: %f", error);
        }
        integral += error;
        count++;
        LF.move(power); LB.move(power); RF.move(-power); RB.move(-power);
        delay(5);
    }
}

void relTurn(int target) {
    con.clear();
    imu.set_heading(180);
    double curr = imu.get_heading();
    target = target + 180;
    double error = target - curr;
    double kP = 0.8;
    double kI = 0.0022;
    double kD = 0.0;
    int power;
    int prevError = 0;
    int integral = 0;
    int count = 0;
    while (abs(error) > 1) {
        curr = imu.get_heading();
        error = target - curr;
        power = kP * error + kI * integral + kD * prevError;
        if(count % 50 == 0) {
            con.print(0, 0, "Heading: %f", curr);
        }
        integral += error;
        prevError = error;
        delay(5);
        count += 1;
        LF.move(power); LB.move(power); RF.move(-power); RB.move(-power);
    }
}
#endif