#include "main.h"
#include "cmath"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/llemu.h"
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
void intake(int direction);
void stopIntake();
void clawClose();
void clawOpen();

//auton
void advance(double moveInches);
void strafe(double moveInches);
void turn(double driveDegrees);
void runDrive(int forward);
void autonSkills();

//GLOBAL-SCOPE VARIABLES(Ports)
#define PORT_LEFT_FRONT 10
#define PORT_LEFT_BACK 9
#define PORT_RIGHT_FRONT 20 
#define PORT_RIGHT_BACK 19
#define PORT_INTAKE_RIGHT 1
#define PORT_INTAKE_LEFT 11

//FILE-SCOPE VARIABLES
Controller master (E_CONTROLLER_MASTER);
Motor motorRightFront   (PORT_RIGHT_FRONT,   E_MOTOR_GEARSET_18, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorRightBack(PORT_RIGHT_BACK, E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_DEGREES);
Motor motorLeftFront   (PORT_LEFT_FRONT,   E_MOTOR_GEARSET_18, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorLeftBack  (PORT_LEFT_BACK,  E_MOTOR_GEARSET_18, 1, E_MOTOR_ENCODER_DEGREES);
Motor motorIntakeRight(PORT_INTAKE_RIGHT, E_MOTOR_GEARSET_18, 0,E_MOTOR_ENCODER_DEGREES);
Motor motorIntakeLeft(PORT_INTAKE_LEFT, E_MOTOR_GEARSET_18, 0,E_MOTOR_ENCODER_DEGREES);
const double ATOV = 200.0/(128.0/3.0); //analog input to velocity output (equals 4.6875)
const double KARSTANT = 0.552;
const double ABBYS_CONSTANT = 0.380;
const double ROBOT_DIAMETER = 17.334935823359;
const double WHEEL_DIAMETER = 4; 
const double PI = M_PI;
double AUTONOMOUS_SPEED = 100.0;
double motorTemperatureDrive;



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
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){/*This button is what allows our robot to break if necessary*/
      brake();
    }
    else{
      moveUpdate();
      motorTemperatureDrive = motorRightBack.get_temperature();//this allows us to check the motor temperature of the drive in order to help troubleshoot potential problems
      lcd::set_text(4, to_string(motorTemperatureDrive) + " F");
      coast();
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
    intake(1);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    intake(-1);
    }
    else{
      stopIntake();
    }
   pros::delay(5);//the delay allows for all the commands to be executed properly before the loop starts again, which reduces the chances of bugs happening within the code
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
  motorRightBack.move_velocity((+forward +rotate) * ATOV);
  motorRightFront.move_velocity((+forward +rotate) * ATOV);
  motorLeftBack.move_velocity((-forward +rotate) * ATOV);
  motorLeftFront.move_velocity((-forward +rotate) * ATOV);
  
}

//Breaks
  void brake(){
    motorRightFront.move_velocity(0);
   motorLeftFront.move_velocity(0);
   motorLeftBack.move_velocity(0);
    motorRightFront.move(0);
   motorRightFront.set_brake_mode(E_MOTOR_BRAKE_HOLD);
   motorLeftFront.set_brake_mode(E_MOTOR_BRAKE_HOLD);
   motorLeftBack.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    motorRightBack.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  }

  void coast(){
    motorRightFront.set_brake_mode(E_MOTOR_BRAKE_COAST);
    motorLeftFront.set_brake_mode(E_MOTOR_BRAKE_COAST);
    motorLeftBack.set_brake_mode(E_MOTOR_BRAKE_COAST);
    motorRightBack.set_brake_mode(E_MOTOR_BRAKE_COAST);
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

void intake(int direction){
  //allows for intake motors to spin the flex wheels in order to intake the triballs and also spin the other direction to output the triballs
motorIntakeRight.move_velocity(200*direction);
motorIntakeLeft.move_velocity(200*direction);
}

void stopIntake(){
  //this function allows the intake motors to stop and not run when we are not pressing the intake buttons
  motorIntakeRight.move_velocity(0);
  motorIntakeLeft.move_velocity(0);
}
//Autonmous Period Sub-Functions
void advance(double moveInches) {
  //stop movement
  brake();

  //inch into degree
	double motorDegrees = moveInches / (WHEEL_DIAMETER * PI) * 360 * KARSTANT;

  //robot knows its relative positions
  motorRightBack.tare_position();
  motorLeftBack.tare_position();
  motorLeftFront.tare_position();
  motorRightFront.tare_position();

  //move robot
  motorLeftBack  .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
	motorRightBack .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
	motorLeftFront  .move_relative(motorDegrees, AUTONOMOUS_SPEED);
	motorRightFront .move_relative(motorDegrees, AUTONOMOUS_SPEED);

  //robot finish running move_relative before eliminates momentum
  while(!(motorRightFront.get_position() < (motorDegrees + 2) && motorRightFront.get_position() > (motorDegrees-2))){
    delay(20);
  }

  //elimainates momentum
  motorLeftFront.move_velocity(0);
  motorRightFront.move_velocity(0);
  motorLeftBack.move_velocity(0);
  motorRightBack.move_velocity(0);

  coast();
}

void turn(double driveDegrees) {
  double motorDegrees = ((360 * driveDegrees * (PI/180) * (ROBOT_DIAMETER/2)) / (WHEEL_DIAMETER)) * ABBYS_CONSTANT;
 
  motorLeftBack  .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
	motorRightBack .move_relative(+motorDegrees, AUTONOMOUS_SPEED);
	motorLeftFront  .move_relative(+motorDegrees, AUTONOMOUS_SPEED);
	motorRightFront .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
  
  //robot knows its relative positions
  
  motorLeftFront.tare_position();
  motorRightFront.tare_position();

  //move robot
  motorLeftBack  .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
	motorRightBack .move_relative(+motorDegrees, AUTONOMOUS_SPEED);
	motorLeftFront  .move_relative(+motorDegrees, AUTONOMOUS_SPEED);
	motorRightFront .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
  //delay(1500);
  
  /*while(motorLeftFront.get_position() < motorDegrees*5 || motorLeftFront.get_actual_velocity() > 10){
    delay(20);
  }
  */
  //elimainates momentum
  while(!(motorLeftFront.get_position() < (motorDegrees + 2) && motorLeftFront.get_position() > (motorDegrees-2))){
    delay(20);
  }
  motorLeftBack.move_velocity(0);
  motorRightBack.move_velocity(0);
  motorLeftFront.move_velocity(0);
  motorRightFront.move_velocity(0);

  coast();
  lcd::set_text(5, "Ive stopped turning");
}

void runDrive(int forward){
  motorRightFront.move_velocity(forward * AUTONOMOUS_SPEED);
  motorLeftFront.move_velocity(forward * AUTONOMOUS_SPEED);
  motorLeftBack.move_velocity(-forward*AUTONOMOUS_SPEED);
  motorRightBack.move_velocity(-forward*AUTONOMOUS_SPEED);
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