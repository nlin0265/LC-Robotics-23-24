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
void opPuncher();
void stopOpPuncher();
void armHorizontal(int direction);
void armForward();
void armVertical(int direction);
void flapOpen();
void flapClose();


//auton
void puncher();//made two different catapult functions, one for autonomous(precise) and one for operator controls
void resetPuncher();
void advance(double moveInches);
void turn(double driveDegrees);
void autonSkills();
void autonFar();
void autonClose();

//GLOBAL-SCOPE VARIABLES(Ports)
#define PORT_LEFT_FRONT 1
#define PORT_LEFT_MIDDLE 2
#define PORT_LEFT_BACK 3
#define PORT_RIGHT_FRONT 11
#define PORT_RIGHT_MIDDLE 12
#define PORT_RIGHT_BACK 13
#define PORT_PUNCHER 9
#define PORT_ARM 10

//FILE-SCOPE VARIABLE
Controller master (E_CONTROLLER_MASTER);
Motor motorLeftFront   (PORT_LEFT_FRONT,   E_MOTOR_GEARSET_18, 1, E_MOTOR_ENCODER_DEGREES);
Motor motorLeftMiddle(PORT_LEFT_MIDDLE, E_MOTOR_GEARSET_18, 1, E_MOTOR_ENCODER_DEGREES);
Motor motorLeftBack  (PORT_LEFT_BACK,  E_MOTOR_GEARSET_18, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorRightFront(PORT_RIGHT_FRONT,   E_MOTOR_GEARSET_18, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorRightMiddle(PORT_RIGHT_MIDDLE, E_MOTOR_GEARSET_18, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorRightBack(PORT_RIGHT_BACK, E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_DEGREES);

Motor motorPuncher(PORT_PUNCHER, E_MOTOR_GEARSET_18, 0,E_MOTOR_ENCODER_DEGREES);
Motor motorArm(PORT_ARM, E_MOTOR_GEARSET_18, 1, E_MOTOR_ENCODER_DEGREES);

const double ATOV = 200.0/(128.0/3.0); //analog input to velocity output (equals 4.6875)
const double ADVANCE = 0.552; //Karstant
const double TURN = 0.380; //Abby's Constant
const double ROBOT_DIAMETER = 23; //inches
const double WHEEL_DIAMETER = 4; //inches
const double PI = M_PI;
double resetCount = 1;
double AUTONOMOUS_SPEED = 100.0;
double motorTemperatureDrive;
double motorArmTemp;
bool isLaunch = false;
bool isFlap = false;

//TOP-LEVEL FUNCTIONS

//always the first function called
void initialize() {
  if (!lcd::is_initialized()) {
    lcd::initialize();
    
  }
  ADIDigitalOut piston1('G');
  ADIDigitalOut piston2('H');
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
  motorPuncher.set_zero_position(motorPuncher.tare_position());
  motorArm.set_zero_position(motorArm.tare_position());
  armVertical(-1);
  delay(50);
  motorArm.move_velocity(0);
  while (true) {
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){/*This button is what allows our robot to break if necessary*/
      brake();
    }
    else{
      moveUpdate();
      motorTemperatureDrive = motorRightBack.get_temperature();//this allows us to check the motor temperature of the drive in order to help troubleshoot potential problems
      motorArmTemp = motorArm.get_temperature();
      lcd::set_text(4, to_string(motorTemperatureDrive) + " F");
      lcd::set_text(3, to_string(motorArmTemp) + " F");
      coast();
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
    if(!isLaunch){
      opPuncher();
      isLaunch=true;
    }
    else{
      stopOpPuncher();
      isLaunch=false;
    }
    }
    if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
    if(!isFlap){
      flapOpen();
      isFlap=true;
    }
    else{
      flapClose();
      isFlap=false;
    }
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
      armHorizontal(1);
    }
    else if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)){
      armHorizontal(-1);
    }

    lcd::set_text(5, to_string((resetCount)));
    delay(10);//the delay allows for all the commands to be executed properly before the loop starts again, which reduces the chances of bugs happening within the code
   }  

    }





//called when autonomous is selected
void autonomous() {
  lcd::set_text(1, "Start auton");
  autonFar();
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
  motorRightMiddle.move_velocity((+forward +rotate) * ATOV);
  motorRightFront.move_velocity((+forward +rotate) * ATOV);
  motorLeftBack.move_velocity((+forward -rotate) * ATOV);
  motorLeftMiddle.move_velocity((+forward -rotate) * ATOV);
  motorLeftFront.move_velocity((+forward -rotate) * ATOV);
}

//Breaks
  void brake(){
    motorRightFront.move_velocity(0);
   motorLeftFront.move_velocity(0);
   motorLeftBack.move_velocity(0);
    motorRightFront.move_velocity(0);
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
void flapOpen(){
  ADIDigitalOut piston1('G');
  ADIDigitalOut piston2('H');
  piston1.set_value(true);
  piston2.set_value(true);
}
void flapClose(){
	ADIDigitalOut piston1('G');
  ADIDigitalOut piston2('H');
	piston1.set_value(false);
  piston2.set_value(false);
}

void opPuncher(){
  motorPuncher.move_velocity(100);
}

void stopOpPuncher(){
  motorPuncher.move_velocity(0);
}


void armHorizontal(int direction){
  motorArm.move_relative(180*direction, 100);
}
void armForward(){
  motorArm.move_velocity(10);
}
void armVertical(int direction){
  motorArm.move_relative(90*direction, 100);
}
//Autonmous Period Sub-Functions

void puncher(){  
motorPuncher.move_absolute(720, 90);
if(resetCount == 5){
      resetPuncher();
      resetCount = 1;
    }
else{
motorPuncher.tare_position();
resetCount++;
}
}

void resetPuncher(){
  motorPuncher.move_absolute(-10, 100);
  motorPuncher.tare_position();
}

void advance(double moveInches) {
  //stop movement
  brake();

  //inch into degree
	double motorDegrees = 2*(moveInches) / (WHEEL_DIAMETER * PI) * 360 * ADVANCE;

  //robot knows its relative positions
  motorRightBack.tare_position();
  motorRightMiddle.tare_position();
  motorRightFront.tare_position();
  motorLeftBack.tare_position();
  motorLeftMiddle.tare_position();
  motorLeftFront.tare_position();
  
  //move robot
	motorRightBack .move_relative(motorDegrees, AUTONOMOUS_SPEED);
	motorRightMiddle.move_relative(motorDegrees, AUTONOMOUS_SPEED);
	motorRightFront .move_relative(motorDegrees, AUTONOMOUS_SPEED);
  motorLeftBack  .move_relative(motorDegrees, AUTONOMOUS_SPEED);
  motorLeftMiddle.move_relative(motorDegrees, AUTONOMOUS_SPEED);
  motorLeftFront  .move_relative(motorDegrees, AUTONOMOUS_SPEED);

  //robot finish running move_relative before eliminates momentum
  while(!(motorRightFront.get_position() < (motorDegrees + 2) && motorRightFront.get_position() > (motorDegrees-2))){
    delay(20);
  }

  //elimainates momentum
  
  motorRightFront.move_velocity(0);
  motorRightMiddle.move_velocity(0);
  motorRightBack.move_velocity(0);
  motorLeftBack.move_velocity(0);
  motorLeftMiddle.move_velocity(0);
  motorLeftFront.move_velocity(0);

  coast();
}

void turn(double driveDegrees) {
  double motorDegrees = ((driveDegrees*(PI*ROBOT_DIAMETER)/WHEEL_DIAMETER)/2)*TURN;
  
  //robot knows its relative positions
  
  motorLeftFront.tare_position();
  motorRightFront.tare_position();

  //move robot
  motorRightFront .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
  motorRightMiddle.move_relative(-motorDegrees, AUTONOMOUS_SPEED);
	motorRightBack .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
	motorLeftFront  .move_relative(+motorDegrees, AUTONOMOUS_SPEED);
  motorLeftMiddle.move_relative(+motorDegrees, AUTONOMOUS_SPEED);
  motorLeftBack .move_relative(+motorDegrees, AUTONOMOUS_SPEED);

  //elimainates momentum
  while(!(motorLeftFront.get_position() < (motorDegrees + 2) && motorLeftFront.get_position() > (motorDegrees-2))){
    delay(20);
  }
  
  motorRightBack.move_velocity(0);
  motorRightMiddle.move_velocity(0);
  motorRightFront.move_velocity(0);
  motorLeftFront.move_velocity(0);
  motorLeftMiddle.move_velocity(0);
  motorLeftBack.move_velocity(0);

  coast();
}


//Auton Period Code
void autonClose(){
turn(-45);
advance(-12);
turn(90);
advance(-7);
armHorizontal(1);
delay(1000);
turn(95);
turn(-15);
armVertical(-1);
delay(500);
turn(80);
advance(-16);
turn(15);
turn(25);
advance(-18);
armForward();
delay(1000);
motorArm.move_velocity(0);
lcd::set_text(5, "Auton Stop");

}
void autonFar(){
advance(-12);
armVertical(1);
delay(500);
motorArm.move_velocity(0);
}

void autonSkills(){
  opPuncher();
}