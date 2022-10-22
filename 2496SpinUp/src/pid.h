#include "main.h"
#include "global.h"
#include <chrono>
#include <cmath>
#include <iostream>

#ifndef _PID_
#define _PID_
using namespace std;
using namespace glb;
using namespace pros;
float kp;
float ki;
float kd;
float power = 0;
float derivative = 0;
float integral = 0;
float error = 0;
float prev_error = 0;
float prev_derivative;
int stored_error; //global
int stored_target; //global
int stored_enc_avg; //global
int stored_runtime; //global
int stored_min_max[20]; //global
int stored_time[20]; //global
int stored_imu;
int moveCount = 0;


#define STRAIGHT_KP 6
#define STRAIGHT_KI 1
#define STRAIGHT_KD 1
#define INTEGRAL_KI 5
#define MAX_INTEGRAL 20
#define COUNT_CONST 50
#define TURN_KP 12 // 9.9 // 2
#define TURN_KI 1 // 0.1 // 0.1
#define TURN_KD 24  // 35 // 20
#define MAXTIME 300
#define MINSPEED 0



void spinToBlue() {
    double hue = optical.get_hue(); 
    optical.set_led_pwm(25);
    while(hue < 80 || hue > 220) {
        hue = optical.get_hue();
        INTAKE.move(55);
        delay(5);
    }
    INTAKE.move(0);

}

void spinToRed() {
    double hue = optical.get_hue(); 
    optical.set_led_pwm(25);

    while(hue > 70) {
        hue = optical.get_hue();
        INTAKE.move(55);
        delay(5);
    }

    INTAKE.move(0);
}

void spinIndexer(int times, int speed) {
    IDX.move_relative(times*480, speed);
}

void spinFlywheel(int speed) {
    F1.move(speed);
    F2.move(speed);
}

void stopFlyWheel() {
    F1.set_brake_mode(E_MOTOR_BRAKE_COAST);
    F2.set_brake_mode(E_MOTOR_BRAKE_COAST);
    F1.move_velocity(0);
    F2.move_velocity(0);
}

void moveIntake(int speed) { 
    INTAKE.move(speed);
}

void stopIntake() {
    INTAKE.move(0);
}

int signOf(int num) {
    if (num > 0) return 1;
    if (num < 0) return -1;
    return 0;
}

float calc(float target, float input, float maxI, int integralKI) {
    prev_error = error;   
    error = target - input;

    prev_derivative = derivative; derivative = error - prev_error; 

    if(abs(error) < integralKI) {
        integral += error;
    }
    else {
        integral = 0; 
    }

    if (integral >= 0) {
        integral = min(integral, maxI);
    }
    else {
        integral = max(integral,-maxI);
    }
    
    if (abs(kp*error) <= MINSPEED) {
        power = signOf(error)*MINSPEED + ki*integral + kd*derivative;
    } 
    else {
        power = kp*error + ki*integral + kd*derivative;
    }

    

    //power = kp*error + ki*integral + kd*derivative;

    return power;
}
bool sameSign(int num1, int num2)
{
    return num1 >= 0 && num2 >= 0 || num1 < 0 && num2 < 0;
}



void chas_move(int left_power, int right_power){
    RF.move(right_power);
    RB.move(right_power);
    LF.move(left_power);
    LB.move(left_power);
}
void reset_encoders(){
    RF.tare_position();
    RB.tare_position();
    LF.tare_position();
    LB.tare_position(); 
}
void setValues(float p_kp, float p_ki, float p_kd){
    kp = p_kp;
    ki = p_ki; 
    kd = p_kd; 
}

void pidmove (int target){
    setValues(STRAIGHT_KP,STRAIGHT_KI,STRAIGHT_KD);
    reset_encoders();
    int count = 0;
    float encoder_average;
    float voltage;
    float error = 0;
    int runtime_count = 0;


    while(true){
        encoder_average = (LF.get_position() + RF.get_position())/2; //Average encoder value for left front and right front chassis motors
        voltage = calc(target, encoder_average, MAX_INTEGRAL, INTEGRAL_KI); //Setting voltage to the PID power
        chas_move(voltage, voltage); //Moving chassis based on voltage value
        error = target-encoder_average; //Creating error
        if (abs(error) <= 50) count++; //Incrementing count if error is less than 50
        if (!sameSign(prev_derivative, derivative)) {
            // linear[moveCount].cStore(error, stored_runtime);
        }
        if (count >= COUNT_CONST) {
            stored_error = error;
            stored_enc_avg = ((LF.get_position() + RF.get_position())/2);
            break;
        }
        if (runtime_count % 5 == 0 && !(runtime_count % 10 == 0)) {
            con.clear();
        } else if (runtime_count % 10 == 0) {
            con.print(1,0,"Error: %f", error);
        }
        stored_runtime += 10;
        runtime_count++;
        pros::delay(10);
    }
    // linear[moveCount].store(target, stored_runtime);
    chas_move(0,0);
}


void pidturn (float target){
    setValues(TURN_KP,TURN_KI,TURN_KD);
    // switch(turnType) {
    //     case 0:
    //         setValues(TURN_KP,TURN_KI,TURN_KD);
    //         break;
    //     case 1:
    //         setValues(TURN_KP1,TURN_KI1,TURN_KD1);
    //         break;
    //     case 2:
    //         setValues(TURN_KP2,TURN_KI2,TURN_KD2);
    //         break;
    // } 
    float position; 
    float start; 
    float voltage;
    int count = 0;
    int arr_count = 0;
    int turn_time = 0;
    int runtime_count = 0;
    int timeout = 0;


    start = imu.get_rotation();

    while(true) { 
        position = imu.get_rotation();

        voltage = calc(target, position, MAX_INTEGRAL, INTEGRAL_KI);

        chas_move(voltage, -voltage);

        if (abs(target-position) <= 0.75) count++;
        if (count >= COUNT_CONST) break; //|| runtime_count >= MAX_RUNTIME

        if (abs(target-position) <= 2) timeout++;
        if(timeout >= MAXTIME) break;

        if (runtime_count % 5 == 0 && !(runtime_count % 10 == 0)) {
            con.clear();
        } else if (runtime_count % 10 == 0) {
            con.print(0,0,"Header: %f", imu.get_rotation());
            con.print(1,0,"Kp:%f, Ki:%f", TURN_KP, TURN_KI);
            con.print(2,0,"Kd:%f", TURN_KD);
        }
        runtime_count++;
        pros::delay(10);

    }
    chas_move(0,0);
}

//Needed autons - fullAwp left, fullAwp right, elims left, elims right, halfAwp left, halfAwp right
// to qual for an awp - need 2 discs in high goal, and both rollers controlled
// 
// elims awp - fire 2 -> spin roller -> intake and shoot more, could skip the roller as 2 discs = 1 roller
// and could result in more points
// half awp - same as elim awp ? fire 2, spin roller, intake and shoot more
 

void fullAwp() {

}

void redHalfAwpRight() {
    spinFlywheel(100);
    moveIntake(127);
    chas_move(75, 75);
    delay(500);
    chas_move(0, 0);
    delay(250);
    pidturn(217);
    delay(1500);
    spinIndexer(-1, 60);
    spinFlywheel(108);

    delay(750);
    spinIndexer(-1, 60);
    spinFlywheel(116);
    delay(800);
    spinIndexer(-1, 60);
    delay(500);
    
    spinFlywheel(0);
    pidturn(180);
    chas_move(70, 70);
    delay(500);
    chas_move(0, 0);
    delay(500);
    pidturn(94);
    
    pidmove(1625);
    pidturn(178);
    chas_move(35, 35);
    moveIntake(-70);

    delay(1000);
    chas_move(5, 5);
    // chas_move(0, 0);
    spinToRed();

}

void blueHalfAwpRight() {
    spinFlywheel(100);
    moveIntake(127);
    chas_move(75, 75);
    delay(500);
    chas_move(0, 0);
    delay(250);
    pidturn(217);
    delay(1500);
    spinIndexer(-1, 60);
    spinFlywheel(108);

    delay(750);
    spinIndexer(-1, 60);
    spinFlywheel(116);
    delay(800);
    spinIndexer(-1, 60);
    delay(500);
    
    spinFlywheel(0);
    pidturn(180);
    chas_move(70, 70);
    delay(500);
    chas_move(0, 0);
    delay(500);
    pidturn(94);
    
    pidmove(1625);
    pidturn(178);
    chas_move(35, 35);
    moveIntake(-70);

    delay(1000);
    chas_move(5, 5);
    // chas_move(0, 0);
    spinToBlue();
}

void redHalfAwpLeft() {
    // spin flywheel
    
    spinFlywheel(116);
    delay(2100);
    // fire twice
    
    spinIndexer(-1, 60);
    spinFlywheel(120);
    delay(1200);
    spinIndexer(-1, 60);
    delay(500);
    // drive back (possibly turn) and toggle roller
    // pidturn(-6);

    chas_move(30, 30);

    moveIntake(-50);
    delay(1000);
    moveIntake(0);
    spinToRed();
    delay(150);

    chas_move(0, 0);

    pidmove(5);
    
    // turn and face towards discs at mid
    
    pidturn(-135);
    delay(5);

    // drive and intake

    moveIntake(127);
    pidmove(1500);


    // move chassis forward to intake disc stack
    chas_move(45, 45);
    delay(1750);
    spinFlywheel(93);
    chas_move(0, 0);

    pidturn(-33);
    // face goal
    // pray (fire discs at high goal)s
    
    delay(500);

    spinIndexer(-1, 60);
    spinFlywheel(98);
    delay(800);
    
    spinIndexer(-1, 60);
    spinFlywheel(103);
    delay(900);
    spinIndexer(-1, 60);
}

void blueHalfAwpLeft() {
    // spin flywheel
    
    spinFlywheel(116);
    delay(2100);
    // fire twice
    
    spinIndexer(-1, 60);
    spinFlywheel(120);
    delay(1200);
    spinIndexer(-1, 60);
    delay(500);
    // drive back (possibly turn) and toggle roller
    // pidturn(-6);

    chas_move(30, 30);

    moveIntake(-50);
    delay(1000);
    moveIntake(0);
    spinToBlue();
    delay(150);

    chas_move(0, 0);

    pidmove(5);
    
    // turn and face towards discs at mid
    
    pidturn(-135);
    delay(5);

    // drive and intake

    moveIntake(127);
    pidmove(1500);


    // move chassis forward to intake disc stack
    chas_move(45, 45);
    delay(1750);
    spinFlywheel(93);
    chas_move(0, 0);

    pidturn(-33);
    // face goal
    // pray (fire discs at high goal)s
    
    delay(500);

    spinIndexer(-1, 60);
    spinFlywheel(98);
    delay(800);
    
    spinIndexer(-1, 60);
    spinFlywheel(103);
    delay(900);
    spinIndexer(-1, 60);
}

void redSoloAwp() {
    //setup straight (roller perpendicular to robot, as far forward without touching disc)
    //charge flywheel
    spinFlywheel(125);
    chas_move(30, 30);
    moveIntake(-50);
    delay(400);
    moveIntake(0);
    delay(100);

    //spin roller
    spinToRed();
    chas_move(-60, -60);
    pidturn(2);
    delay(125);
    chas_move(0, 0);
    delay(500);

    
    
    //fire discs
    spinIndexer(-1, 60);
    delay(450);
    
    spinIndexer(-2, 60);
    delay(600);

    //drive and pick up 3 disc stack

    //turn
    // pidmove(-5);
    pidturn(-128);
    spinFlywheel(105);
    moveIntake(127);
    pidmove(1290);

    //collect discs
    chas_move(55, 55); // speed up later
    delay(1535);
    
    chas_move(0, 0);
    

    //turn towards goal
    pidturn(-33);

    //fire three discs
    spinIndexer(-1, 60);
    spinFlywheel(108); 
    delay(500);
    //prevent rpm drop on flywheel by increasing rpm
    
    spinIndexer(-1, 60);
    spinFlywheel(112);
    delay(600);
    
    spinIndexer(-2.2, 60);
    delay(600);

    //turn towards discs -- theoretical
    pidturn(-139);
    moveIntake(127);
    pidmove(4023);

    chas_move(100,0);
    
    delay(1000);
    moveIntake(0);
    delay(10);
    chas_move(30,30);
    delay(50);
    chas_move(0,0);
    moveIntake(-50);
    delay(400);
    moveIntake(0);
    delay(100);
    //spin roller
    spinToRed();

    /*pidturn(-85);

    //roller
    moveIntake(-50);
    delay(1000);
    moveIntake(0);

    //spin roller
    spinToRed();

    // align with goal
    chas_move(30, 30);
    delay(1000);*/

}
void spitDisc(){
    spinFlywheel(50);
    spinIndexer(-1,60);
}
//old code (brian's from start of practice)
void BHredSoloAwp() {
    //setup straight (roller perpendicular to robot, as far forward without touching disc)
    //charge flywheel
    spinFlywheel(107);
    chas_move(30, 30);
    moveIntake(-50);
    delay(400);
    moveIntake(0);
    delay(100);

    //spin roller
    spinToRed();
    chas_move(-70, -70);
    delay(100);
    pidturn(3);
    delay(300);
    // chas_move(0, 0);
    // delay(500);

    
    
    //fire discs
    spinIndexer(-1, 60);
    spinFlywheel(112);

    delay(800);
    
    spinIndexer(-2, 60);
    delay(600);

    //drive and pick up 3 disc stack

    //turn
    spinFlywheel(97);

    // pidmove(-5);
    pidturn(-128);
    moveIntake(127);
    pidmove(1375);

    //collect discs
    moveIntake(90);
    chas_move(80, 80); // speed up later
    delay(1000);
    
    chas_move(0, 0);
    

    //turn towards goal
    pidturn(-31);

    //fire three discs
    spinIndexer(-1, 60);
    spinFlywheel(103); 
    delay(500);
    //prevent rpm drop on flywheel by increasing rpm
    
    spinIndexer(-1, 60);
    spinFlywheel(108);
    delay(600);
    
    spinIndexer(-2, 60);
    delay(400);

    //turn towards discs -- theoretical
    spinFlywheel(60);

    pidturn(-143);
    spinIndexer(-1, 60);
    moveIntake(127);
    pidmove(4125);
    moveIntake(-100);
    chas_move(100,0);
    moveIntake(0);
    delay(400);
    chas_move(30,30);
    moveIntake(-50);
    delay(500);
    chas_move(0,0);
    // delay(00);
     //spin roller
    spinToRed();

    /*pidturn(-85);

    //roller
    moveIntake(-50);
    delay(1000);
    moveIntake(0);

    //spin roller
    spinToRed();

    // align with goal
    chas_move(30, 30);
    delay(1000);*/

}

void skills() {
    // spin flywheel
    
    spinFlywheel(116);
    delay(2100);
    // fire twice
    
    spinIndexer(-1, 60);
    spinFlywheel(120);
    delay(1200);
    spinIndexer(-1, 60);
    delay(500);
    // drive back (possibly turn) and toggle roller
    // pidturn(-6);

    chas_move(30, 30);

    moveIntake(-50);
    delay(1000);
    moveIntake(0);
    spinToRed();
    delay(150);

    chas_move(0, 0);

    pidmove(5);
    
    // turn and face towards discs at mid
    
    pidturn(-135);
    delay(5);

    // drive and intake

    moveIntake(127);
    pidmove(1500);


    // move chassis forward to intake disc stack
    chas_move(45, 45);
    delay(1750);
    spinFlywheel(93);
    chas_move(0, 0);

    pidturn(-33);
    // face goal
    // pray (fire discs at high goal)s
    
    delay(500);

    spinIndexer(-1, 60);
    spinFlywheel(98);
    delay(800);
    
    spinIndexer(-1, 60);
    spinFlywheel(103);
    delay(900);
    spinIndexer(-1, 60);
    // skills portion
    pidturn(45);
    pidmove(3000);
    pidturn(30);
    expansion.set_value(true);
}

#endif