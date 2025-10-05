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
pros::Rotation verticalRotation(-16);
pros::Rotation horizontalRotation(-20);

lemlib::TrackingWheel verticalWheel(&verticalRotation, lemlib::Omniwheel::NEW_2, -0.375);
lemlib::TrackingWheel horizontalWheel(&verticalRotation, lemlib::Omniwheel::NEW_2, -1.25);

lemlib::OdomSensors odomSensors(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);

lemlib::Drivetrain drivetrain(
	&left_motors, 
	&right_motors, 
	11, 
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
	0, // anti windup
	0, // small error range, in inches
	0, // small error range timeout, in milliseconds
	0, // large error range, in inches
	0, // large error range timeout, in milliseconds
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
const float highRed = 40;

const float lowBlue = 170;
const float highBlue = 240;

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
	//std::cout << "Hue: " << ((optical.get_hue() >= lowBlue && optical.get_hue() <= highBlue) ? "Blue" : "NO") << std::endl;
	return(optical.get_hue() >= lowBlue && optical.get_hue() <= highBlue);
}

bool isAllianceColor() {
	if (allianceIsRed) {
		return isRed();
	} else {
		return isBlue();
	}
}

bool isOpponentColor() {
	if (allianceIsRed) {
		return isBlue();
	} else {
		return isRed();
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

void spinIndex(void* correctColor) {
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
	
	//std::cout << "Finished sort" << std::endl;
}

void colorSort() {
	if (!intaking)
	{
		return;
	}

	bool allianceColor = isAllianceColor();
	bool opponentColor = isOpponentColor();
	bool isColor = allianceColor || opponentColor;

	if (isColor)
	{
		if (allianceColor && blocksPassing<3)
		{
			//std::cout << "Sorting alliance color" << std::endl;
			pros::Task sortTask(spinIndex, (void*)true);
		}
		else if (opponentColor && blocksPassing<3)
		{
			pros::Task sortTask(spinIndex, (void*)false);
		} else
		{
			//std::cout << "Sorting unknown color" << std::endl;
		}
	}
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

void printTrackingWheels() {
	while (true) {
		pros::lcd::print(0, "Vertical: %f", chassis.getPose().y);
		pros::lcd::print(1, "Horizontal: %f", chassis.getPose().x);
		pros::lcd::print(2, "Heading: %f", chassis.getPose().theta);
		std::cout << "Vertical: " << chassis.getPose().y << " Horizontal: " << chassis.getPose().x << " Heading: " << chassis.getPose().theta << std::endl;
		pros::delay(100);
	}
}

void initialize() {

	chassis.calibrate();

	pros::lcd::initialize();
	//pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);

	//std::cout << "Ran initialize" << std::endl;

	pros::Task trackingWheelPrinter(printTrackingWheels);
}


void disabled() {//std::cout << "Ran disable" << std::endl;
	}

void competition_initialize() { //std::cout << "ran comp initialize" << std::endl;
	}

void leavePark(){
	chassis.moveToPoint(0, 3, 100);
}

void rightSideAuton(){
	chassis.moveToPoint(0, 7.5, 100);
	chassis.turnToHeading(25, 100);
	spinIntake();
	chassis.moveToPoint(7.5, 17.5, 100);
	stopAll();
	chassis.turnToHeading(-45, 100);
	chassis.moveToPose(-7, 30.5, -45, 200);
	spinBottomCenter();
	pros::delay(1000);
	stopAll();
	pros::delay(500);
	spinBottomCenter();
	pros::delay(1000);
	stopAll();
	pros::delay(500);
	spinBottomCenter();
}

void testAngularPid(){
	// set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
	std::cout << "Set pose to 0, 0, 0" << std::endl;
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
	std::cout << "Turned to 90" << std::endl;
}

void autonomous() {
	//leavePark();
	//rightSideAuton();
	//testAngularPid();
	//std::cout << "Ran auton" << std::endl;
	chassis.moveToPoint(0, 24, 100);
}

void opcontrol() {
	autonomous();

	//std::cout << "Starting op" << std::endl;
	optical.set_led_pwm(100);

	while (true)
	{
		colorSort();

		//std::cout << "Starting loop" << std::endl;

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

		//std::cout << "Finished loop" << std::endl;
	}
}
