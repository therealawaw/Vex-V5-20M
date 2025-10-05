#include "main.h"
#include "lemlib/api.hpp"
//Define variables here

//Electronics
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({17, 13, -14}, pros::v5::MotorGears::blue);
pros::MotorGroup right_motors({18, 2, -15}, pros::v5::MotorGears::blue);

pros::Motor intake(-1, pros::v5::MotorGears::blue);
pros::Motor inde(-5, pros::v5::MotorGears::green);
pros::Motor score(-10, pros::v5::MotorGears::green);

pros::adi::DigitalOut basket(1);

pros::Imu imu(6);
pros::Optical optical(12);
pros::Rotation verticalRotation(20); //SET REVERSAL AT SCHOOL
pros::Rotation horizontalRotation(16); //SET REVERSAL AT SCHOOL

lemlib::TrackingWheel verticalWheel(&verticalRotation, lemlib::Omniwheel::NEW_2, 1); //SET DISTANCE AT SCHOOL
lemlib::TrackingWheel horizontalWheel(&verticalRotation, lemlib::Omniwheel::NEW_2, 1); //SET DISTANCE AT SCHOOL

lemlib::OdomSensors odomSensors(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);

lemlib::Drivetrain drivetrain(
	&left_motors, 
	&right_motors, 
	12.5, 
	lemlib::Omniwheel::NEW_275, 
	450, 
	8
);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
	0, // integral gain (kI)
	3, // derivative gain (kD)
	3, // anti windup
	1, // small error range, in inches
	100, // small error range timeout, in milliseconds
	3, // large error range, in inches
	500, // large error range timeout, in milliseconds
	20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
	0, // integral gain (kI)
	10, // derivative gain (kD)
	3, // anti windup
	1, // small error range, in degrees
	100, // small error range timeout, in milliseconds
	3, // large error range, in degrees
	500, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(
	drivetrain, 
	lateral_controller, 
	angular_controller, 
	odomSensors
);

//Constants

const float lowRed = 0;
const float highRed = 20;

const float lowBlue = 170;
const float highBlue = 220;

//Variables

bool allianceIsRed = false; //Assume red until set otherwise
bool intaking = false;
int scoring = 0; //0 = not scoring, 1 = bottom center, 2 = top center, 3 = long goal
int blocksPassing = 0;

//Start Functions here

//Color Senseing Functions

bool isRed() {
	return(optical.get_hue() >= lowRed && optical.get_hue() <= highRed);
}

bool isBlue() {
	return(optical.get_hue() >= lowBlue && optical.get_hue() <= highBlue);
}

bool isAllianceColor() {
	if (allianceIsRed) {
		return isRed();
	} else {
		return isBlue();
	}
}

//Roller Functions

void stopAll() {
	intake.brake();
	inde.brake();
	score.brake();
	intaking = false;
	scoring = 0;
	//std::cout << "Stopping all" << std::endl;
}

void spinIndex(bool correctColor) {
	blocksPassing += 1;
	inde.move(127 * (correctColor ? 1 : -1));
	score.move(127 * 0.2 * (correctColor ? -1 : 0));
	pros::delay(900);
	blocksPassing -= 1;
	if (blocksPassing==0)
	{
		inde.brake();
		score.brake();
	}
	std::cout << correctColor << std::endl;
}

void colorSort(bool intaking) {
	if (!intaking)
	{
		return;
	}
	std::cout << "Passed intake check" << std::endl;
	bool colorSense = isAllianceColor();
	
	spinIndex(colorSense);
}

void spinIntake() {
	intake.move(127*0.75);
	intaking = true;
	scoring = -1;
	basket.set_value(0);
}

void spinBottomCenter() {
	intake.move(-127);
	inde.move(-127);
	scoring = 1;
	intaking = false;
	basket.set_value(4095);
}

void spinTopCenter() {
	inde.move(-127);
	score.move(127);
	intake.move(127);
	scoring = 2;
	intaking = false;
	basket.set_value(4095);
}

void spinLongGoal() {
	inde.move(-127);
	score.move(-127);
	scoring = 3;
	intaking = false;
	basket.set_value(4095);
}

void drive() {
	int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

	chassis.arcade(-rightX, -leftY);
}

//Competition Functions

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


void initialize() {

	chassis.calibrate();

	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
}


void disabled() {}

void competition_initialize() {}

void autonomous() {}

void colorSortTask() {
	while (true) {
		colorSort(intaking);
	}
}

void opcontrol() {
	intake.move_velocity(600);
	inde.move_velocity(200);
	score.move_velocity(200);
	optical.set_led_pwm(100);

	pros::Task colorSort_thread(colorSortTask);

	while (true)
	{
		drive();
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && (scoring==1 || scoring==0)) {
			//std::cout << "Spinning bottom center" << std::endl;
			spinBottomCenter();

		}else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && (scoring==3 || scoring==0)) {
			//std::cout << "Spinning long goal" << std::endl;
			spinLongGoal();

		}else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && (scoring==2 || scoring==0)) {
			//std::cout << "Spinning top center" << std::endl;
			spinTopCenter();

		}else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && (scoring==-1 || scoring==0)) {
			//std::cout << "Spinning intake" << std::endl;
			spinIntake();
		} else {stopAll();}
	}
}
