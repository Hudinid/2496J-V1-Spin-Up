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

void velocitySpinFlywheel(int rpm) {
    F1.move_velocity(rpm);
    F2.move_velocity(rpm);
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



void blueHalfAwpRight() {
 
    velocitySpinFlywheel(482);
    moveIntake(127);
    chas_move(75, 75);
    delay(500);
    chas_move(0, 0);
    pidturn(220);
    delay(1500);
    spinIndexer(-1, 60);
    // velocitySpinFlywheel(510);

    delay(1750);
    spinIndexer(-1, 60);
    velocitySpinFlywheel(480);
    delay(1000);
    spinIndexer(-2, 100);
    delay(750);
    
    pidturn(180);
    chas_move(70, 70);
    // velocitySpinFlywheel(0);
    delay(500);
    chas_move(0, 0);
    moveIntake(-120);
    
    pidturn(94);
    
    pidmove(1602);
    pidturn(180);
    
    // flipping a coin
    chas_move(55, 55);
    delay(1000);
    chas_move(0, 0);
    //spin roller
    spinToBlue();
             
}


void redHalfAwpRight() {
    
    velocitySpinFlywheel(482);
    moveIntake(127);
    chas_move(75, 75);
    delay(500);
    chas_move(0, 0);
    pidturn(220);
    delay(1500);
    spinIndexer(-1, 60);
    // velocitySpinFlywheel(510);

    delay(1750);
    spinIndexer(-1, 60);
    velocitySpinFlywheel(480);
    delay(1000);
    spinIndexer(-2, 100);
    delay(750);
    
    pidturn(180);
    chas_move(70, 70);
    // velocitySpinFlywheel(0);
    delay(500);
    chas_move(0, 0);
    moveIntake(-120);
    
    pidturn(94);
    
    pidmove(1602);
    pidturn(180);
    
    // flipping a coin
    chas_move(55, 55);
    delay(1000);
    chas_move(0, 0);
    //spin roller
    spinToRed();
    
}

void redHalfAwpLeft() {
//setup straight (roller perpendicular to robot, as far forward without touching disc)
//charge flywheel
    velocitySpinFlywheel(520);
    chas_move(30, 30);
    moveIntake(-70);
    delay(250);
    // moveIntake(0);
    // delay(100); 

    //spin roller
    spinToRed();
    delay(50);
    chas_move(-70, -70);
    delay(100);
    pidturn(3);
    delay(1000);
        
    //fire discs
    spinIndexer(-1, 60);
    // spinFlywheel(113);

    delay(800);
    
    spinIndexer(-2, 60);
    moveIntake(-77);
    delay(1000);

    //drive and pick up 3 disc stack
    velocitySpinFlywheel(463);

    pidturn(-131);
    pidmove(1350);
    moveIntake(127);
    
    //collect discs
    chas_move(80, 80); // speed up later
    delay(1050);
    chas_move(0, 0);

    //turn towards goal
    pidturn(-30);
    delay(1000);
    //fire three discs
    spinIndexer(-1, 60);
    delay(500);
    
    spinIndexer(-1, 60);
    delay(350);
    
    spinIndexer(-2, 60);
    
    delay(650);
   

    spinIndexer(-2, 80);
    delay(1000);

    // stopIntake();
}   


void blueHalfAwpLeft() {
    //setup straight (roller perpendicular to robot, as far forward without touching disc)
//charge flywheel
    velocitySpinFlywheel(520);
    chas_move(30, 30);
    moveIntake(-70);
    delay(250);
    // moveIntake(0);
    // delay(100); 

    //spin roller
    spinToBlue();
    delay(50);
    chas_move(-70, -70);
    delay(100);
    pidturn(3);
    delay(1000);
        
    //fire discs
    spinIndexer(-1, 60);
    // spinFlywheel(113);

    delay(800);
    
    spinIndexer(-2, 60);
    moveIntake(-77);
    delay(1000);

    //drive and pick up 3 disc stack
    velocitySpinFlywheel(463);

    pidturn(-131);
    pidmove(1350);
    moveIntake(127);
    
    //collect discs
    chas_move(80, 80); // speed up later
    delay(1050);
    chas_move(0, 0);

    //turn towards goal
    pidturn(-30);
    delay(1000);
    //fire three discs
    spinIndexer(-1, 60);
    delay(500);
    
    spinIndexer(-1, 60);
    delay(350);
    
    spinIndexer(-2, 60);
    
    delay(650);
   

    spinIndexer(-2, 80);
    delay(1000);

    // stopIntake();
}


void redSoloAwp() {
    //setup straight (roller perpendicular to robot, as far forward without touching disc)
    //charge flywheel
    velocitySpinFlywheel(495);
    chas_move(30, 30);
    moveIntake(-70);
    delay(300);
    moveIntake(0);
    delay(100); 

    //spin roller
    spinToRed();
    delay(50);
    chas_move(-70, -70);
    delay(100);
    pidturn(4);
    delay(300);
        
    //fire discs
    spinIndexer(-1, 60);
    // spinFlywheel(113);

    delay(620);
    
    spinIndexer(-2, 80);
    moveIntake(-50);
    delay(600);

    //drive and pick up 3 disc stack
    velocitySpinFlywheel(462);

    pidturn(-131);
    pidmove(1339);
    moveIntake(127);
    
    //collect discs
    chas_move(80, 80); // speed up later
    delay(1000);
    chas_move(0, 0);

    //turn towards goal
    pidturn(-29);

    //fire three discs
    spinIndexer(-1, 60);
    delay(400);
    
    spinIndexer(-1, 60);
    delay(350);
    
    spinIndexer(-2, 80);
    delay(650);

    //turn towards discs -- theoretical
    // spinFlywheel(60);

    pidturn(-143);
    spinIndexer(-1, 60);
    moveIntake(127);
    pidmove(4113);
    moveIntake(-100);
    
    chas_move(127,0);
    moveIntake(-90);
    delay(500);
    chas_move(10,10);
    delay(200);
    spinToRed();
    // delay(10);
}

void blueSoloAwp() {
    //setup straight (roller perpendicular to robot, as far forward without touching disc)
    //charge flywheel
    velocitySpinFlywheel(495);
    chas_move(30, 30);
    moveIntake(-70);
    delay(300);
    moveIntake(0);
    delay(100); 

    //spin roller
    spinToBlue();
    delay(50);
    chas_move(-70, -70);
    delay(100);
    pidturn(4);
    delay(300);
        
    //fire discs
    spinIndexer(-1, 60);
    // spinFlywheel(113);

    delay(620);
    
    spinIndexer(-2, 80);
    moveIntake(-50);
    delay(600);

    //drive and pick up 3 disc stack
    velocitySpinFlywheel(462);

    pidturn(-131);
    pidmove(1339);
    moveIntake(127);
    
    //collect discs
    chas_move(80, 80); // speed up later
    delay(1000);
    chas_move(0, 0);

    //turn towards goal
    pidturn(-29);

    //fire three discs
    spinIndexer(-1, 60);
    delay(400);
    
    spinIndexer(-1, 60);
    delay(350);
    
    spinIndexer(-2, 80);
    delay(650);

    //turn towards discs -- theoretical
    // spinFlywheel(60);

    pidturn(-143);
    spinIndexer(-1, 60);
    moveIntake(127);
    pidmove(4113);
    moveIntake(-100);
    
    chas_move(127,0);
    moveIntake(-90);
    delay(500);
    chas_move(10,10);
    delay(200);
    spinToBlue();
    // delay(10);
}

void skills_discs() {
    velocitySpinFlywheel(516);
    chas_move(30, 30);
    moveIntake(-50);
    delay(150);
    chas_move(0,0);
    delay(250);
    // moveIntake(0);
    // delay(100); 

    //spin roller
    spinToRed();
    delay(50);
    chas_move(-70, -70);
    delay(100);
    pidturn(3);
    delay(1050);
        
    //fire discs
    spinIndexer(-1, 60);
    // spinFlywheel(113);

    delay(800);
    
    spinIndexer(-2, 60);
    moveIntake(-77);
    delay(1000);

    //drive and pick up 3 disc stack
    velocitySpinFlywheel(463);

    pidturn(-131);
    pidmove(1350);
    moveIntake(127);
    
    //collect discs
    chas_move(60, 60); // speed up later
    delay(1250);
    chas_move(0, 0);

    //turn towards goal
    pidturn(-30);
    delay(1000);
    //fire three discs
    spinIndexer(-1, 60);
    delay(500);
    
    spinIndexer(-1, 60);
    delay(350);
    
    spinIndexer(-2, 60);
    
    delay(650);
   

    spinIndexer(-2, 80);
    delay(1000);

    // skills portion
    pidturn(45);
    pidmove(1750);
    pidturn(90);
    moveIntake(-127);
    chas_move(90, 90); // speed up later
    delay(600);
    chas_move(0, 0);
    moveIntake(127);
    velocitySpinFlywheel(500);
    chas_move(60, 60); // speed up later
    delay(800);
    chas_move(0, 0);
    
    pidturn(112);
    delay(1000);
    moveIntake(-127);
    spinIndexer(-1, 60);
    delay(500);
    
    spinIndexer(-1, 60);
    delay(350);
    pidturn(110);
    
    spinIndexer(-2, 60);
    
    delay(650);
   

    spinIndexer(-2, 80);
    delay(1000);

    pidmove(-100);
    pidturn(88);
    
    moveIntake(127);
    /*chas_move(30, 30);
    moveIntake(-127);
    delay(150);
    chas_move(0,0);
    delay(250);*/
    
    pidmove(300);
    pidmove(-300);
    pidturn(90);
    
    chas_move(50, 50);
    
    delay(150);
    chas_move(0,0);
    delay(250);
    // moveIntake(0);
    // delay(100); 

    //spin roller
    spinToRed();

    pidmove(-1300);
    
    pidturn(40);

    pidmove(100);

    expansion.set_value(true);
    
    
      //expansion.set_value(true);
}

void skills_half(){
    velocitySpinFlywheel(513);
    chas_move(30, 30);
    moveIntake(-70);
    delay(250);
    // moveIntake(0);
    // delay(100); 

    //spin roller
    spinToRed();
    delay(50);
    chas_move(-70, -70);
    delay(100);
    pidturn(3);
    delay(1000);
        
    //fire discs
    spinIndexer(-1, 60);
    // spinFlywheel(113);

    delay(800);
    
    spinIndexer(-2, 60);
    moveIntake(-77);
    delay(1000);

    //drive and pick up 3 disc stack
    velocitySpinFlywheel(463);

    pidturn(-131);
    pidmove(1350);
    moveIntake(127);
    
    //collect discs
    chas_move(80, 80); // speed up later
    delay(1050);
    chas_move(0, 0);

    //turn towards goal
    pidturn(-31);
    delay(1000);
    //fire three discs
    spinIndexer(-1, 60);
    delay(500);
    
    spinIndexer(-1, 60);
    delay(350);
    
    spinIndexer(-2, 60);
    
    delay(650);
   

    spinIndexer(-2, 80);
    delay(1000);

    // stopIntake();

    pidturn(45);
    pidmove(1750);

    pidturn(40);

    pidmove(100);

    expansion.set_value(true);



}

#endif