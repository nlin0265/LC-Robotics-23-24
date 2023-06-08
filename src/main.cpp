#include "main.h"
#include "cmath"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.h"
#include <iterator>
#include <memory>
#include <string>
#define _USE_MATH_DEFINES

using namespace pros;
using namespace std;
using namespace lcd;


//PROTOTYPES
//user
void moveUpdate();
void brake();
void coast();
void clawClose();
void clawOpen();

//auton
void advance(double moveInches);
void strafe(double moveInches);
void turn(double driveDegrees);
void runDrive(int forward);
void autonSkills();

//GLOBAL-SCOPE VARIABLES(Ports)
#define PORT_BACK_LEFT 1
#define PORT_BACK_RIGHT 11
#define PORT_FRONT_LEFT 15 
#define PORT_FRONT_RIGHT 17
#define PORT_MIDDLE 16
#define PORT_ROLLER 12
#define PORT_INTAKE 9
#define PORT_INDEXER 14
#define PORT_FLY_WHEEL 13


//FILE-SCOPE VARIABLES
Controller master (E_CONTROLLER_MASTER);
Motor motorBackLeft   (PORT_BACK_LEFT,   E_MOTOR_GEARSET_18, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorBackRight(PORT_BACK_RIGHT, E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_DEGREES);
Motor motorFrontLeft   (PORT_FRONT_LEFT,   E_MOTOR_GEARSET_18, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorFrontRight  (PORT_FRONT_RIGHT,  E_MOTOR_GEARSET_18, 1, E_MOTOR_ENCODER_DEGREES);
Motor motorMiddle     (PORT_MIDDLE,      E_MOTOR_GEARSET_18, 1, E_MOTOR_ENCODER_DEGREES);
const double ATOV = 200.0/(128.0/3.0); //analog input to velocity output (equals 4.6875)
const double KARSTANT = 0.552;
const double ABBYS_CONSTANT = 0.380;
const double ROBOT_DIAMETER = 17.334935823359;
const double WHEEL_DIAMETER = 4; 
const double PI = M_PI;
double AUTONOMOUS_SPEED = 100.0; 
bool isBreaking = false;
bool isFlyWheel = false;



//TOP-LEVEL FUNCTIONS

//always the first function called
void initialize() {
  if (!lcd::is_initialized()) {
    lcd::initialize();
    
  }
  ADIDigitalOut piston('A');
}


//called after initialize and before autonomous when connected to field or competition switch
void competition_initialize() {
  //initializes screen
  if(!lcd::is_initialized()){
    lcd::initialize();
  }
}


//called when operator control is selected
void opcontrol() {
  //initialize screen
  if(!lcd::is_initialized()){
    lcd::initialize();
  }

  //drive loop
  while (true) {
    
     //brakes
    if (master.get_digital(E_CONTROLLER_DIGITAL_L1)){
    brake();
    }
    else{
    moveUpdate();
    coast();
   }
   delay(3);
    }

  }



//called when autonomous is selected
void autonomous() {
  lcd::set_text(1, "Start auton");
}


//called after autonomous or opcontrol when connected to field or competition switch
void disabled() {
  
}



//SUB-FUNCTIONS
//Driver Controls

//Drive
void moveUpdate() {
	//accepts input from controller
	int forward = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
  int rotate = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
  int strafe = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
  motorBackLeft.move_velocity((-forward -rotate) * ATOV);
  motorBackRight.move_velocity((-forward +rotate) *ATOV);
  motorFrontRight.move_velocity((+forward -rotate) * ATOV);
  motorFrontLeft.move_velocity((+forward +rotate) * ATOV);
  motorMiddle.move_velocity((+strafe) * ATOV);
}

//Breaks
  void brake(){
    motorFrontRight.move_velocity(0);
   motorFrontLeft.move_velocity(0);
   motorBackLeft.move_velocity(0);
    motorBackRight.move(0);
   motorFrontRight.set_brake_mode(E_MOTOR_BRAKE_HOLD);
   motorFrontLeft.set_brake_mode(E_MOTOR_BRAKE_HOLD);
   motorBackLeft.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    motorBackRight.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  }

  void coast(){
    motorFrontRight.set_brake_mode(E_MOTOR_BRAKE_COAST);
    motorFrontLeft.set_brake_mode(E_MOTOR_BRAKE_COAST);
    motorBackLeft.set_brake_mode(E_MOTOR_BRAKE_COAST);
    motorBackRight.set_brake_mode(E_MOTOR_BRAKE_COAST);
  }

//Opening and Closing claw
void clawClose(){
  ADIDigitalOut piston('A');
  piston.set_value(true);
}
void clawOpen(){
	ADIDigitalOut piston('A');
	piston.set_value(false);
}

//Autonmous Period Sub-Functions
void advance(double moveInches) {
  //stop movement
  brake();

  //inch into degree
	double motorDegrees = moveInches / (WHEEL_DIAMETER * PI) * 360 * KARSTANT;

  //robot knows its relative positions
  motorBackRight.tare_position();
  motorBackLeft.tare_position();
  motorFrontLeft.tare_position();
  motorFrontRight.tare_position();

  //move robot
  motorBackLeft  .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
	motorBackRight .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
	motorFrontLeft  .move_relative(motorDegrees, AUTONOMOUS_SPEED);
	motorFrontRight .move_relative(motorDegrees, AUTONOMOUS_SPEED);

  //robot finish running move_relative before eliminates momentum
  while(!(motorFrontRight.get_position() < (motorDegrees + 2) && motorFrontRight.get_position() > (motorDegrees-2))){
    delay(20);
  }

  //elimainates momentum
  motorFrontLeft.move_velocity(0);
  motorFrontRight.move_velocity(0);
  motorBackLeft.move_velocity(0);
  motorBackRight.move_velocity(0);

  coast();
}

void strafe(double moveInches) {
	double motorDegrees = (moveInches/(WHEEL_DIAMETER*PI)) * 360 * KARSTANT;
  motorMiddle.move_relative(motorDegrees, AUTONOMOUS_SPEED);
}


void turn(double driveDegrees) {
  double motorDegrees = ((360 * driveDegrees * (PI/180) * (ROBOT_DIAMETER/2)) / (WHEEL_DIAMETER)) * ABBYS_CONSTANT;
 
  motorBackLeft  .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
	motorBackRight .move_relative(+motorDegrees, AUTONOMOUS_SPEED);
	motorFrontLeft  .move_relative(+motorDegrees, AUTONOMOUS_SPEED);
	motorFrontRight .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
  
  //robot knows its relative positions
  
  motorFrontLeft.tare_position();
  motorFrontRight.tare_position();

  //move robot
  motorBackLeft  .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
	motorBackRight .move_relative(+motorDegrees, AUTONOMOUS_SPEED);
	motorFrontLeft  .move_relative(+motorDegrees, AUTONOMOUS_SPEED);
	motorFrontRight .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
  //delay(1500);
  
  /*while(motorFrontLeft.get_position() < motorDegrees*5 || motorFrontLeft.get_actual_velocity() > 10){
    delay(20);
  }
  */
  //elimainates momentum
  while(!(motorFrontLeft.get_position() < (motorDegrees + 2) && motorFrontLeft.get_position() > (motorDegrees-2))){
    delay(20);
  }
  motorBackLeft.move_velocity(0);
  motorBackRight.move_velocity(0);
  motorFrontLeft.move_velocity(0);
  motorFrontRight.move_velocity(0);

  coast();
  lcd::set_text(5, "Ive stopped turning");
}

void runDrive(int forward){
  motorFrontRight.move_velocity(forward * AUTONOMOUS_SPEED);
  motorFrontLeft.move_velocity(forward * AUTONOMOUS_SPEED);
  motorBackLeft.move_velocity(-forward*AUTONOMOUS_SPEED);
  motorBackRight.move_velocity(-forward*AUTONOMOUS_SPEED);
}

//Auton Period Code
void autonL(){

}
void autonLeftHigh(){

}

void autonR(){
 
}

void autonSkills(){
  
}