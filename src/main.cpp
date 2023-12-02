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
void catapult();
void stopCatapult();
void lockCatapult();
void releaseCatapult();
void flapOpen();
void flapClose();


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
#define PORT_LEFT_FRONT 5
#define PORT_LEFT_MIDDLE 4
#define PORT_LEFT_BACK_MIDDLE 3
#define PORT_LEFT_BACK 2
#define PORT_RIGHT_FRONT 10
#define PORT_RIGHT_MIDDLE 9
#define PORT_RIGHT_BACK_MIDDLE 8
#define PORT_RIGHT_BACK 7
#define PORT_CATAPULT 6

//FILE-SCOPE VARIABLE
Controller master (E_CONTROLLER_MASTER);
Motor motorLeftFront   (PORT_LEFT_FRONT,   E_MOTOR_GEARSET_18, 1, E_MOTOR_ENCODER_DEGREES);
Motor motorLeftMiddle(PORT_LEFT_MIDDLE, E_MOTOR_GEARSET_18, 1, E_MOTOR_ENCODER_DEGREES);
Motor motorLeftBackMiddle  (PORT_LEFT_BACK_MIDDLE,  E_MOTOR_GEARSET_18, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorLeftBack(PORT_LEFT_BACK, E_MOTOR_GEARSET_18, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorRightFront(PORT_RIGHT_FRONT,   E_MOTOR_GEARSET_18, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorRightMiddle(PORT_RIGHT_MIDDLE, E_MOTOR_GEARSET_18, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorRightBackMiddle(PORT_RIGHT_BACK_MIDDLE, E_MOTOR_GEARSET_18, 1, pros::E_MOTOR_ENCODER_DEGREES);
Motor motorRightBack(PORT_RIGHT_BACK, E_MOTOR_GEARSET_18, 0, E_MOTOR_ENCODER_DEGREES);
Motor motorCatapult(PORT_CATAPULT, E_MOTOR_GEARSET_18, 1,E_MOTOR_ENCODER_DEGREES);

const double ATOV = 200.0/(128.0/3.0); //analog input to velocity output (equals 4.6875)
const double ADVANCE = 0.552; //Karstant
const double TURN = 0.380; //Abby's Constant
const double ROBOT_DIAMETER = 23; //inches
const double WHEEL_DIAMETER = 4; //inches
const double PI = M_PI;
double resetCount = 1;
double AUTONOMOUS_SPEED = 100.0;
double motorTemperatureDrive;
bool isLaunch = false;
bool isFlap = false;
bool isLock = false;

//TOP-LEVEL FUNCTIONS

//always the first function called
void initialize() {
  if (!lcd::is_initialized()) {
    lcd::initialize();
    
  }
  ADIDigitalOut piston1('H');
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
  flapClose();
  motorCatapult.set_zero_position(0);
  motorCatapult.move_relative(653, 90);
  delay(200);
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

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
    if(!isLaunch){
      catapult();
      isLaunch=true;
    }
    else{
      stopCatapult();
      isLaunch=false;
    }
    }
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
      lockCatapult();
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
    

    delay(10);//the delay allows for all the commands to be executed properly before the loop starts again, which reduces the chances of bugs happening within the code
   }  

    }





//called when autonomous is selected
void autonomous() {
  lcd::set_text(1, "Start auton");
  autonSkills();
}


//called after autonomous or opcontrol when connected to field or competition switch
void disabled() {
  
}



//SUB-FUNCTIONS
//Driver Controls

//Drive
void moveUpdate() {
	//accepts input from controller
	int moveR = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y); 
  int moveL = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
 
  motorRightBack.move_velocity((+moveR)*ATOV);
  motorRightBackMiddle.move_velocity((+moveR) * ATOV);
  motorRightMiddle.move_velocity((+moveR) * ATOV);
  motorRightFront.move_velocity((+moveR) * ATOV);
  motorLeftBack.move_velocity((+moveL)*ATOV);
  motorLeftBackMiddle.move_velocity((+moveL) * ATOV);
  motorLeftMiddle.move_velocity((+moveL) * ATOV);
  motorLeftFront.move_velocity((+moveL) * ATOV);
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
  ADIDigitalOut piston1('H');
  piston1.set_value(true);
}
void flapClose(){
  ADIDigitalOut piston1('H');
  piston1.set_value(false);
}

void catapult(){
  motorCatapult.move_velocity(90);
}

void stopCatapult(){
  motorCatapult.move_velocity(0);
}

void lockCatapult(){
  motorCatapult.move_relative(653, 50);
}
//Autonmous Period Sub-Functions

void advance(double moveInches) {
  //stop movement
  brake();

  //inch into degree
	double motorDegrees = 2*(moveInches) / (WHEEL_DIAMETER * PI) * 360 * ADVANCE;

  //robot knows its relative positions
  motorRightBackMiddle.tare_position();
  motorRightMiddle.tare_position();
  motorRightFront.tare_position();
  motorLeftBackMiddle.tare_position();
  motorLeftMiddle.tare_position();
  motorLeftFront.tare_position();
  
  //move robot
	motorRightBackMiddle .move_relative(motorDegrees, AUTONOMOUS_SPEED);
	motorRightMiddle.move_relative(motorDegrees, AUTONOMOUS_SPEED);
	motorRightFront .move_relative(motorDegrees, AUTONOMOUS_SPEED);
  motorLeftBackMiddle  .move_relative(motorDegrees, AUTONOMOUS_SPEED);
  motorLeftMiddle.move_relative(motorDegrees, AUTONOMOUS_SPEED);
  motorLeftFront  .move_relative(motorDegrees, AUTONOMOUS_SPEED);

  //robot finish running move_relative before eliminates momentum
  while(!(motorRightFront.get_position() < (motorDegrees + 2) && motorRightFront.get_position() > (motorDegrees-2))){
    delay(20);
  }

  //elimainates momentum
  
  motorRightFront.move_velocity(0);
  motorRightMiddle.move_velocity(0);
  motorRightBackMiddle.move_velocity(0);
  motorLeftBackMiddle.move_velocity(0);
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
	motorRightBackMiddle .move_relative(-motorDegrees, AUTONOMOUS_SPEED);
	motorLeftFront  .move_relative(+motorDegrees, AUTONOMOUS_SPEED);
  motorLeftMiddle.move_relative(+motorDegrees, AUTONOMOUS_SPEED);
  motorLeftBackMiddle .move_relative(+motorDegrees, AUTONOMOUS_SPEED);

  //elimainates momentum
  while(!(motorLeftFront.get_position() < (motorDegrees + 2) && motorLeftFront.get_position() > (motorDegrees-2))){
    delay(20);
  }
  
  motorRightBackMiddle.move_velocity(0);
  motorRightMiddle.move_velocity(0);
  motorRightFront.move_velocity(0);
  motorLeftFront.move_velocity(0);
  motorLeftMiddle.move_velocity(0);
  motorLeftBackMiddle.move_velocity(0);

  coast();
}


//Auton Period Code
void autonClose(){
advance(-8.5);
flapOpen();
delay(100);
advance(3);
delay(500);
flapClose();
turn(45);
advance(12);
turn(45);
advance(26);
flapOpen();

}
void autonFar(){
advance(26);
advance(-24);
turn(160);
advance(33);
flapOpen();






}

void autonSkills(){
  catapult();
}

void autonSkills2(){
  turn(-45);
  advance(-12);
  turn(90);
  advance(-6);
  delay(1000);
  turn(95);
  turn(-95);

  advance(-7);
  catapult();
}