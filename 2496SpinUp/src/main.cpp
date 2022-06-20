#include "main.h"
#include "global.h"
#include "pid.h"
using namespace pros;
using namespace std;
using namespace glb;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	con.clear();
	delay(50);
	con.print(0, 0, "jeff don't int");
	bool autoRoll = false;
	bool hitToggle = false;
	optical.set_led_pwm(25);
	
	double hue;

	// delay(50);
	// con.print(1, 0, "stay under the speed limit-vip");
	// delay(50);
	// con.print(2, 0, "go over the speed limit-elkins");
	// delay(50);
	while(true) {
		int power = con.get_analog(ANALOG_LEFT_Y);
		int turn = con.get_analog(ANALOG_RIGHT_X);

		int left = power + turn;
		int right = power - turn;

		RF.move(right);
		RB.move(right); // hi
		LF.move(left);
		LB.move(left);

		if(!autoRoll) {
			if(con.get_digital(E_CONTROLLER_DIGITAL_R1)) {
				INTAKE.move(127);
			}
			else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)){
				INTAKE.move(-127);
			}
			else INTAKE.move(0);
		}
		else {
			hue = optical.get_hue(); 
			if(hue < 80 || hue > 200) {
				hue = optical.get_hue();
				INTAKE.move(67);
				delay(5);
			}
			else {
				autoRoll = false;
				INTAKE.move(0);
			}
		}

		if(con.get_digital(E_CONTROLLER_DIGITAL_A)) {
			if(!hitToggle) {
				hitToggle = true;
				autoRoll = !autoRoll;
			}
		}
		else hitToggle = false;


		if(con.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			F1.move(127);
			F2.move(127);
		}
		else {
			F1.move(0);
			F2.move(0);
		}
		delay(5);
	}	
}
