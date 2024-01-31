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
void runFlywheel(int direction);
void stopFlywheel();
void flapOpen();
void flapClose();
void runIntake(int direction);
void stopIntake();


//auton
void puncher();//made two different catapult functions, one for autonomous(precise) and one for operator controls
void resetPuncher();
void advance(double moveInches);
void turn(double driveDegrees);
void autonSkills();
void autonSkills2();
void autonFar();
void autonClose();

//GLOBAL-SCOPE VARIABLES(Ports)
#define PORT_LEFT_TOP 6
#define PORT_LEFT_BOTTOM 10
#define PORT_LEFT_BACK 4
#define PORT_RIGHT_TOP 16
#define PORT_RIGHT_BOTTOM 20 
#define PORT_RIGHT_BACK 15
#define PORT_INTAKE 11
#define PORT_FLYWHEEL 18

//FILE-SCOPE VARIABLE
Controller master (E_CONTROLLER_MASTER);
Motor motorLeftBottom   (PORT_LEFT_BOTTOM,   E_MOTOR_GEAR_BLUE, 1, E_MOTOR_ENCODER_DEGREES);
Motor motorLeftTop(PORT_LEFT_TOP, E_MOTOR_GEAR_BLUE, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorLeftBack  (PORT_LEFT_BACK,  E_MOTOR_GEAR_BLUE, 1, E_MOTOR_ENCODER_DEGREES);
Motor motorRightTop(PORT_RIGHT_TOP,   E_MOTOR_GEAR_BLUE, 1, E_MOTOR_ENCODER_DEGREES);
Motor motorRightBottom(PORT_RIGHT_BOTTOM, E_MOTOR_GEAR_BLUE, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorRightBack(PORT_RIGHT_BACK, E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
Motor motorIntake(PORT_INTAKE, E_MOTOR_GEAR_BLUE, 0,E_MOTOR_ENCODER_DEGREES);
Motor motorFlyWheel(PORT_FLYWHEEL, E_MOTOR_GEAR_BLUE, 0, E_MOTOR_ENCODER_DEGREES);

const double ATOV = 200.0/(128.0/3.0); //analog input to velocity output (equals 4.6875)
const double ADVANCE = 0.552; //Karstant
const double TURN = 0.380; //Abby's Constant
const double ROBOT_DIAMETER = 16; //inches
const double WHEEL_DIAMETER = 3; //inches
const double PI = M_PI;
double resetCount = 1;
double AUTONOMOUS_SPEED = 300.0;
double motorTemperatureDrive;
bool isFlyWheel = false;
bool isFlap = false;
bool isIntake = false;

//TOP-LEVEL FUNCTIONS

//always the first function called
void initialize() {
  if (!lcd::is_initialized()) {
    lcd::initialize();
    
  }
  ADIDigitalOut piston1('G');
  ADIDigitalOut intake('H');
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

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      runFlywheel(1);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      runFlywheel(-1);
    }
    else{
      stopFlywheel();
    }
    if(master.get_digital(E_CONTROLLER_DIGITAL_R1)){
     runIntake(1);
    }
    else if (master.get_digital(E_CONTROLLER_DIGITAL_R2)){
    runIntake(-1);
    }
    else{
    stopIntake();
    }
    }

    

    delay(10);//the delay allows for all the commands to be executed properly before the loop starts again, which reduces the chances of bugs happening within the code
    }
   

    





//called when autonomous is selected
void autonomous() {
  lcd::set_text(1, "Start auton");
  autonClose();
}


//called after autonomous or opcontrol when connected to field or competition switch
void disabled() {
  
}



//SUB-FUNCTIONS
//Driver Controls

//Drive
void moveUpdate() {
	//accepts input from controller
	int moveL = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y); 
  int moveR = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
 
  motorRightTop.move_velocity((+moveR)*ATOV);
  motorRightBottom.move_velocity((+moveR) * ATOV);
  motorRightBack.move_velocity((+moveR) * ATOV);
  motorLeftTop.move_velocity((+moveL)*ATOV);
  motorLeftBottom.move_velocity((+moveL) * ATOV);
  motorLeftBack.move_velocity((+moveL) * ATOV);
}

//Breaks
  void brake(){
    motorRightTop.move_velocity(0);
    motorLeftTop.move_velocity(0);
    motorLeftBack.move_velocity(0);
    motorRightBack.move_velocity(0);
    motorRightBottom.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    motorLeftBottom.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    motorLeftBack.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    motorRightBack.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  }

  void coast(){
    motorRightTop.set_brake_mode(E_MOTOR_BRAKE_COAST);
    motorLeftTop.set_brake_mode(E_MOTOR_BRAKE_COAST);
    motorRightBottom.set_brake_mode(E_MOTOR_BRAKE_COAST);
    motorLeftBottom.set_brake_mode(E_MOTOR_BRAKE_COAST);
    motorLeftBack.set_brake_mode(E_MOTOR_BRAKE_COAST);
    motorRightBack.set_brake_mode(E_MOTOR_BRAKE_COAST);
  }

//Opening and Closing claw
void runIntake(int direction){
  motorIntake.move_velocity(600*direction);
}

void stopIntake(){
  motorIntake.move_velocity(0);
}

void runFlywheel(int direction){
  motorFlyWheel.move_velocity(600 * direction);
}
void stopFlywheel(){
  motorFlyWheel.move_velocity(0);
}
//Autonmous Period Sub-Functions

void advance(double moveInches) {
  //stop movement
  brake();

  //inch into degree
	double motorDegrees = 2*(moveInches) / (WHEEL_DIAMETER * PI) * 360 * ADVANCE;

  //robot knows its relative positions
  motorRightBack.tare_position();
  motorRightTop.tare_position();
  motorRightBottom.tare_position();
  motorLeftBack.tare_position();
  motorLeftTop.tare_position();
  motorLeftBottom.tare_position();
  
  //move robot
	motorRightBack.move_relative(motorDegrees, AUTONOMOUS_SPEED);
	motorRightTop.move_relative(motorDegrees, AUTONOMOUS_SPEED);
	motorRightBottom.move_relative(motorDegrees, AUTONOMOUS_SPEED);
  motorLeftBack.move_relative(motorDegrees, AUTONOMOUS_SPEED);
  motorLeftTop.move_relative(motorDegrees, AUTONOMOUS_SPEED);
  motorLeftBottom.move_relative(motorDegrees, AUTONOMOUS_SPEED);

  //robot finish running move_relative before eliminates momentum
  while(!(motorRightBottom.get_position() < (motorDegrees + 2) && motorRightBottom.get_position() > (motorDegrees-2))){
    delay(20);
  }

  //elimainates momentum
  
  motorRightTop.move_velocity(0);
  motorRightBottom.move_velocity(0);
  motorRightBack.move_velocity(0);
  motorLeftBack.move_velocity(0);
  motorLeftTop.move_velocity(0);
  motorLeftBottom.move_velocity(0);

  coast();
}

void turn(double driveDegrees) {
  double motorDegrees = ((driveDegrees*(PI*ROBOT_DIAMETER)/WHEEL_DIAMETER)/2)*TURN;
  
  //robot knows its relative positions
  
  motorLeftBottom.tare_position();
  motorRightBottom.tare_position();

  //move robot

  motorRightTop.move_relative(-motorDegrees, AUTONOMOUS_SPEED);
  motorRightBottom.move_relative(-motorDegrees, AUTONOMOUS_SPEED);
	motorRightBack.move_relative(-motorDegrees, AUTONOMOUS_SPEED);
	motorLeftTop.move_relative(+motorDegrees, AUTONOMOUS_SPEED);
  motorLeftBottom.move_relative(+motorDegrees, AUTONOMOUS_SPEED);
  motorLeftBack.move_relative(+motorDegrees, AUTONOMOUS_SPEED);

  //elimainates momentum

  while(!(motorLeftBottom.get_position() < (motorDegrees + 2) && motorLeftBottom.get_position() > (motorDegrees-2))){
    delay(20);
  }
  
  motorRightBack.move_velocity(0);
  motorRightTop.move_velocity(0);
  motorRightBottom.move_velocity(0);
  motorLeftTop.move_velocity(0);
  motorLeftBottom.move_velocity(0);
  motorLeftBack.move_velocity(0);

  coast();
}


//Auton Period Code
void autonClose(){
advance(30);
turn(125);
advance(20);
runIntake(-1);
delay(800);
advance(-3);
advance(6);
stopIntake();
advance(-6);
//turn(180);
//advance(75);
//turn(90);
}
void autonFar(){
advance(20);
turn(-125);
advance(25);
runIntake(-1);
delay(800);
advance(-3);
advance(6);
stopIntake();
//advance(-6);
//turn(-100);
}

void autonSkills(){
  runFlywheel(-1);
}

void autonSkills2(){

}