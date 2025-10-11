#include "main.h"
#include "lemlib/api.hpp"
//Define variables here

//Electronics
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-17, -13, 14}, pros::v5::MotorGears::blue);
pros::MotorGroup right_motors({18, 2, -15}, pros::v5::MotorGears::blue);

pros::Motor intake(-1, pros::v5::MotorGears::blue);
pros::Motor inde(-5, pros::v5::MotorGears::green);
pros::Motor score(-10, pros::v5::MotorGears::green);

pros::adi::DigitalOut basket(1);
pros::adi::DigitalOut scraper(2);
pros::adi::Led led1('C', 10);

pros::Imu imu(7);
pros::Optical optical(12);
pros::Rotation verticalRotation(-16); //SET REVERSAL AT SCHOOL
pros::Rotation horizontalRotation(-20); //SET REVERSAL AT SCHOOL

lemlib::TrackingWheel verticalWheel(&verticalRotation, 1.955, -0.375); //SET DISTANCE AT SCHOOL
lemlib::TrackingWheel horizontalWheel(&horizontalRotation, lemlib::Omniwheel::NEW_2, -1.25); //SET DISTANCE AT SCHOOL

lemlib::OdomSensors odomSensors(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);

lemlib::Drivetrain drivetrain(
	&left_motors, 
	&right_motors, 
	11, 
	lemlib::Omniwheel::NEW_325, 
	450, 
	8
);

lemlib::ControllerSettings lateral_controller(6, // proportional gain (kP)
	0, // integral gain (kI)
	3, // derivative gain (kD)
	3, // anti windup
	0, // small error range, in inches
	0, // small error range timeout, in milliseconds
	0, // large error range, in inches
	0, // large error range timeout, in milliseconds
	0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(
	5, // proportional gain (kP)
	0, // integral gain (kI)
	35, // derivative gain (kD)
	0, // anti windup
	0, // small error range, in degrees
	0, // small error range timeout, in milliseconds
	0, // large error range, in degrees
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

const float farthestBlock = 200;

//Variables

bool allianceIsRed = false; //Assume red until set otherwise
bool intaking = false;
int scoring = 0; //0 = not scoring, 1 = bottom center, 2 = top center, 3 = long goal
int blocksPassing = 0;
int autonSelect = 0; //left = 0, right = 1, skills = 2
int buttons = 000;

std::string autonNames[3] = {"Left", "Right", "Skills"};

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

void spinIndex(bool correctColor, bool opponentColor) {
    static bool running = false;
    static uint32_t startTime = 0;
    static int direction = 0;
	bool isColor = (correctColor || opponentColor) && (optical.get_proximity() > farthestBlock);
	
	//std::cout << "Proximity: " << optical.get_proximity() << ", isColor: " << isColor << ", correctColor: " << correctColor << ", opponentColor: " << opponentColor << ", In proximity: " << (optical.get_proximity() > farthestBlock) << ", Is a Color: " << (correctColor || opponentColor) << std::endl;

    // if not currently running, start a new sort
    if ((!running) && isColor) {
        running = true;
        startTime = pros::millis();
        direction = correctColor ? 1 : -1;

        inde.move(127 * direction);
        score.move(127 * 0.4 * (correctColor ? -1 : 0));
    }

    // if running, check if time has passed
    if (running && pros::millis() - startTime >= 250) {
        inde.brake();
        score.brake();
        running = false;
        std::cout << "Finished sort" << std::endl;
    }
}

void colorSort() {
	if (!intaking)
	{
		return;
	}

	bool allianceColor = isAllianceColor();
	bool opponentColor = isOpponentColor();
	
	spinIndex(allianceColor, opponentColor);
}

void spinIntake() {
	intake.move(127*0.9);
	intaking = true;
	scoring = -1;
	basket.set_value(0);
}

void spinIntakeFull() {
	intake.move(127*0.9);
	inde.move(127);
	score.move(-127*0.4);
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

	chassis.arcade(leftY, rightX);
}

//Competition Functions

void printValuesOnBrain() {
	while (true) {
		pros::lcd::print(0, "Heading: %.2f", imu.get_heading());
		pros::lcd::print(1, "X: %.2f", chassis.getPose().x);
		pros::lcd::print(2, "Y: %.2f", chassis.getPose().y);
		std::cout << "Heading: " << imu.get_heading() << ", X: " << chassis.getPose().x << ", Y: " << chassis.getPose().y << std::endl;
		pros::delay(100);
	}
}

void red_lights(){
	led1.set_all(0xFF0000);
}

void onLeftPress() {

}

void initialize() {

	red_lights();

	chassis.calibrate();

	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	//pros::Task printTask(printValuesOnBrain);

	std::cout << "Ran initialize" << std::endl;


}


void disabled() {std::cout << "Ran disable" << std::endl;}

void competition_initialize() {
	static bool debounce = false;

	std::cout << "Ran comp disable" << std::endl;

	pros::lcd::set_text(1, "Atuton: " + autonNames[autonSelect]);

	while (pros::competition::is_disabled)
	{
		unsigned int buttons = pros::lcd::read_buttons();
		printf("Buttons Bitmap: %d\n", pros::lcd::read_buttons());
		//std::cout << pros::lcd::read_buttons() << " // " << buttons << std::endl;
		if (buttons == LCD_BTN_LEFT && !debounce) // left button
		{
			//std::cout << "left" << std::endl;
			autonSelect = std::clamp(autonSelect-1, 0, 2);
			pros::lcd::set_text(1, "Auton: " + autonNames[autonSelect]);
			debounce = true;
		} else if (buttons == LCD_BTN_RIGHT && !debounce) // right button
		{
			autonSelect = std::clamp(autonSelect+1, 0, 2);
			pros::lcd::set_text(1, "Auton: " + autonNames[autonSelect]);
			debounce = true;
		} 
		else if (buttons == LCD_BTN_CENTER && !debounce) // center button
		{
			debounce = true;
			break;
		} else
		{
			debounce = false;
		}
	}
	
	while (pros::competition::is_disabled)
	{
		uint8_t buttons = pros::lcd::read_buttons();
		if (buttons == LCD_BTN_LEFT && !debounce) // left button
		{
			allianceIsRed = !allianceIsRed;
			pros::lcd::set_text(2, "Alliance: " + allianceIsRed ? "Red" : "Blue");
			debounce = true;
		} else if (buttons == LCD_BTN_RIGHT && !debounce) // right button
		{
			allianceIsRed = !allianceIsRed;
			pros::lcd::set_text(2, "Alliance: " + allianceIsRed ? "Red" : "Blue");
			debounce = true;
		} 
		else if (buttons == LCD_BTN_CENTER && !debounce) // center button
		{
			debounce = true;
			break;
		} else
		{
			debounce = false;
		}
	}
}

void leavePark(){
	chassis.moveToPoint(0, 3, 100);
}

void rightSideAuton(){
	//pros::Task colorSort_thread(colorSortTask);
	chassis.moveToPoint(0, 7.5, 100);
	chassis.turnToHeading(25, 100);
	spinIntakeFull();
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

void leftSideAuton(){
	//pros::Task colorSort_thread(colorSortTask);
	chassis.moveToPoint(0, 7.5, 100);
	chassis.turnToHeading(-25, 100);
	spinIntakeFull();
	chassis.moveToPoint(-7.5, 17.5, 100);
	stopAll();
	chassis.turnToHeading(45, 100);
	chassis.moveToPose(7, 30.5, 45, 200);
	spinTopCenter();
	pros::delay(1000);
	stopAll();
	pros::delay(500);
	spinTopCenter();
	pros::delay(1000);
	stopAll();
	pros::delay(500);
	spinTopCenter();
}

void skillsAuton(){
	chassis.moveToPoint(0, 31.5, 2000);
	chassis.turnToHeading(90, 1000);
	spinIntakeFull();
	scraper.set_value(true);
	chassis.moveToPose(10, 31.5, 90, 2000);
	pros::delay(1500);
	chassis.moveToPose(0, 31.5, 90, 2000);
	stopAll();
	chassis.turnToHeading(-90, 1000);
	chassis.moveToPose(-33.35, 31.5, -90, 4000);
	spinLongGoal();
	pros::delay(2000);
	stopAll();
	chassis.moveToPose(-25, 31.5, -90, 4000);
	scraper.set_value(false);
	chassis.turnToHeading(0, 1000);
	chassis.moveToPose(-25, 41.5, 0, 4000);
	chassis.turnToHeading(-90, 1000);
	chassis.moveToPoint(-97, 41.5, 4000);
	chassis.turnToHeading(180, 1000);
	chassis.moveToPoint(-97, 31.5, 4000);
	chassis.turnToHeading(-90, 1000);
	//Starts to get very unsure from here
	scraper.set_value(true);
	spinIntakeFull();
	chassis.moveToPose(-123, 31.5, -90, 4000);
	pros::delay(1500);
	chassis.moveToPose(-97, 31.5, -90, 4000);
	stopAll();
	scraper.set_value(false);
	chassis.turnToHeading(90, 1000);
	chassis.moveToPose(-97-24, 31.5, 90, 4000);
	spinLongGoal();
	pros::delay(2000);
	stopAll();
	spinIntakeFull();
	chassis.moveToPose(-97-24+23, 31.5-24, 135, 4000);
	pros::delay(1000);
	chassis.moveToPose(-97-24+23+24+14, 31.5-24+24-14, 135, 4000);
	spinTopCenter();

}

void testAngularPid(){
	// set position to x:0, y:0, heading:0
    
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
}

void testLateralPid() {
	chassis.moveToPose(24, 24, 90, 1000);
	//chassis.moveToPose(0, 5 , 0, 10000);
}

void autonomous() {
	chassis.setPose(0, 0, 0);
	//leavePark();
	//leftSideAuton();
	//testAngularPid();
	//testLateralPid();
	if (autonSelect == 0) {
		leftSideAuton();
	} else if (autonSelect == 1) {
		rightSideAuton();
	} else if (autonSelect == 2) {
		skillsAuton();
	}
	intake.move(127);
	pros::delay(5000);
	intake.brake();
	std::cout << "Ran auton" << std::endl;
}

void opcontrol() {

	std::cout << "Starting op" << std::endl;
	optical.set_led_pwm(100);

	while (true)
	{
		colorSort();

		//std::cout << "Starting loop" << std::endl;

		drive();

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && (scoring==1 || scoring==0)) {
			std::cout << "Spinning bottom center" << std::endl;
			spinBottomCenter();

		}else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && (scoring==3 || scoring==0)) {
			std::cout << "Spinning long goal" << std::endl;
			spinLongGoal();

		}else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && (scoring==2 || scoring==0)) {
			std::cout << "Spinning top center" << std::endl;
			spinTopCenter();

		}else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && (scoring==-1 || scoring==0)) {
			//std::cout << "Spinning intake" << std::endl;
			spinIntake();

		} else {stopAll();}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			scraper.set_value(true);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			scraper.set_value(false);
		}

		//std::cout << "Finished loop" << std::endl;
	}
}
