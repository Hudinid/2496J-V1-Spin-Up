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

	expansion.set_value(false);
}
	
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	imu.reset();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
int currAuton = 1;
void competition_initialize() {
	imu.reset();
	while(imu.is_calibrating()) delay (5);

	bool selected = true;
	int localTime = 0;
	int totalAutons = 5;
	con.clear();

	while(true) {

		// if(button.get_value() == 0 || button2.get_value() == 0) {
		// 	if(selected) {
		// 		currAuton ++;
		// 		if(currAuton == totalAutons+1) {
		// 			currAuton = 1;
		// 		}
		// 		selected = false;
		// 	}
		// 	selected = false;
		// }
		// else selected = true;

		if(localTime%50 == 0) {
			// con.clear();
			switch(currAuton) {
				case (1):
					con.print(0, 0, "Selected: %d Red Left", currAuton);
					break;
				case(2):
					con.print(0, 0, "Selected: %d Red Right", currAuton);
					break;
				case(3):
					con.print(0, 0, "Selected: %d Blue Left", currAuton);
					break;
				case(4):
					con.print(0, 0, "Selected: %d Blue Right", currAuton);
					break;
				case(5):
					con.print(0, 0, "Selected: %d None", currAuton);
					break;
			}
			// con.print(0, 0, "Selected: %d", currAuton);
		}
		localTime ++;
	}
}
	
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
void autonomous() {
	blueHalfAwpLeft();
	// if(currAuton == 1) {
	// 	redHalfAwpLeft();
	// }
	// if(currAuton == 2) {
	// 	redHalfAwpRight();
	// }
	// if(currAuton == 3) {
	// 	blueHalfAwpLeft();
	// }
	// if(currAuton == 4) {
	// 	blueHalfAwpRight();
	// }
	// if(currAuton == 5) {
		
	// }
}

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
	// imu.reset();
	// while(imu.is_calibrating()) {
	// 	delay(5);
	// }
	con.clear();
	delay(50);
	con.print(0, 0, "jeff don't int");

	bool autoRoll = false;
	bool hitToggle = false;
	bool hitToggleFSpeed = false;

	IDX.set_brake_mode(E_MOTOR_BRAKE_COAST);
	F1.set_brake_mode(E_MOTOR_BRAKE_COAST);
	F2.set_brake_mode(E_MOTOR_BRAKE_COAST);
	bool toggleFlyWheel = false;
	bool hitFlyWheelToggle = false;
	optical.set_led_pwm(25);
	
	double hue;
	double global_heading = 0;
	// delay(50);
	// con.print(1, 0, "stay under the speed limit-vip");
	// delay(50);
	// con.print(2, 0, "go over the speed limit-elkins");
	// delay(50);
	int flySpeed = 97;
	int count = 0;
	int setFSpeed = 0;
	int flywheelSpeeds = 2;
	int count2 = 0;
	while(true) {
		int power = con.get_analog(ANALOG_LEFT_Y); // left joystick y axis is powe
		int valForTurn = con.get_analog(ANALOG_RIGHT_X); // right joystick x axis controls turn
		
		double turn = (3000*valForTurn + 0.2*pow(valForTurn, 3)); 
		turn /= 4000;
		int left = power + turn; // implement turning
		int right = power - turn; 

		RF.move(right);
		RB.move(right); // hi
		LF.move(left);
		LB.move(left);

		//Display Flywheel speed
		if(count % 50 == 0) {
			
			con.clear();
			delay(50);
			con.print(0, 0, "%f", turn);
			delay(50);
			con.print(1, 0, "%d", flySpeed);
			
		}

		

		//Roller Control
		if(!autoRoll) { // if the autoroller is not on
			if(con.get_digital(E_CONTROLLER_DIGITAL_R1)) { // then allow for manual control through R1 and R2
				INTAKE.move(127);
			}
			else if(con.get_digital(E_CONTROLLER_DIGITAL_R2)){
				INTAKE.move(-127);
			}
			else INTAKE.move(0);
		}
		else { // otherwise run the code for autoroller
			hue = optical.get_hue();  // get the color that the optical is currently looking at
			if(hue < 70 || hue > 200) { // while the color is not within a certain range
				hue = optical.get_hue(); // get color again
				INTAKE.move(64); // move until the color is what we want
				delay(5);
			}
			else {
				autoRoll = false; // once the hue is the color we want, turn off the autoroller
				INTAKE.move(0); // and make the roller stop moving
			}
		}

		if(con.get_digital(E_CONTROLLER_DIGITAL_A)) { // toggle the autoroller
			if(!hitToggle) {
				hitToggle = true;
				autoRoll = !autoRoll;
			}
		}
		else hitToggle = false; // safeguard so that only one press will be registered at a time

		if(con.get_digital(E_CONTROLLER_DIGITAL_L1)) { // toggle the automatic flywheel
			if(!hitFlyWheelToggle) { 
				hitFlyWheelToggle = true;
				toggleFlyWheel = !toggleFlyWheel;
			}
		}
//mm robot yes monke
		else if(con.get_digital(E_CONTROLLER_DIGITAL_UP)) {
			if(!hitFlyWheelToggle) {
				hitFlyWheelToggle = true;
				flySpeed += 1;
				if(flySpeed > 127) {
					flySpeed = 0;
				}
			}
		}

		else if(con.get_digital(E_CONTROLLER_DIGITAL_DOWN)) { 
			if(!hitFlyWheelToggle) {
				hitFlyWheelToggle = true;
				flySpeed -= 1;
				if(flySpeed < 0) { 
				    flySpeed = 127;
				}
			}
		}

		else hitFlyWheelToggle = false;

		if(toggleFlyWheel) {
			F1.move(flySpeed);
			F2.move(flySpeed);
			count2 ++;
			if(count2 % 10000) {
				con.rumble(".");
			}
		} 
		else {
			F1.move(0);
			F2.move(0);
		}

		
		
		bool toggleIDX = false;
		bool checkToggleIDX = false;
		if(con.get_digital(E_CONTROLLER_DIGITAL_RIGHT)) { // toggle the automatic flywheel
			if(!checkToggleIDX) { 
				checkToggleIDX = true;
				toggleIDX = !toggleIDX;
			}
		}
		else checkToggleIDX = false;
		//Indexer 
		if(con.get_digital(E_CONTROLLER_DIGITAL_L2) && !toggleIDX) {
			IDX.move(-70);
		}
		else if(con.get_digital(E_CONTROLLER_DIGITAL_B)){
			IDX.move(-55);
		}
		// else if (con.get_digital(E_CONTROLLER_DIGITAL_L2) && toggleIDX) {
		// 	IDX.move(-55);
		// 	// spinIndexer(-420, 80);
		// }
		else {
			if(!toggleIDX) {
				IDX.move(0);
			}
		}

		if(con.get_digital(E_CONTROLLER_DIGITAL_Y)) {
			if(!hitToggleFSpeed) {
				hitToggleFSpeed = true;
				setFSpeed ++;
				if(setFSpeed >= flywheelSpeeds) { 
					setFSpeed = 0;
					flySpeed = 97;
				}
				else if (setFSpeed == 1) {
					flySpeed = 87;
				}
				
			}

		}
		else hitToggleFSpeed = false;

		 
		if(con.get_digital(E_CONTROLLER_DIGITAL_LEFT)) {
			con.rumble(". - .");
		}

		
		if(con.get_digital(E_CONTROLLER_DIGITAL_X)) {
			expansion.set_value(true);
		}
		else {
			
		}	

		


		count ++;
		delay(5);
	}	
}
