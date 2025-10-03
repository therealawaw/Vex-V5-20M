#include "main.h"
#include "lemlib/api.hpp"

//Define variables here

//Electronics
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({17, 13, -14}, pros::v5::MotorGears::blue);
pros::MotorGroup right_motors({18, 2, -15}, pros::v5::MotorGears::blue);

pros::Motor intake(-1, pros::v5::MotorGears::blue);
pros::Motor index(5, pros::v5::MotorGears::green);
pros::Motor score(10, pros::v5::MotorGears::green);

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

bool allianceIsRed = true; //Assume red until set otherwise

//Start Functions here

//Color Senseing Functions

bool isRed() {
	//Jacob you do this
}

bool isBlue() {
	//Jacob you do this
}

bool isAllianceColor() {
	//Jacob you do this
}

//Roller Functions

void stopAll() {
	//Jacob you do this
}

void spinIndex() {
	//Jacob you do this
}

void spinIntake() {
	//Jacob you do this
}

void spinBottomCenter() {
	//Jacob you do this
}

void spinTopCenter() {
	//Jacob you do this
}

void spinLongGoal() {
	//Jacob you do this
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
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	chassis.calibrate();
}


void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true)
	{
		//Jacob you do this
	}
}
