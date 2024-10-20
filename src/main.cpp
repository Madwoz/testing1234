#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pid.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"  // IWYU pragma: keep
#include <cstdint>
#include <algorithm>
#include <cstdlib>
#include <vector>

pros::Controller Master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors({-10, -4, 3},
    pros::MotorGearset::blue); // left motors use 600 RPM cartrifges
pros::MotorGroup right_motors(
    {9, 7, -6},
    pros::MotorGearset::blue); // right motors use 600 RPM cartridges


// defines intake motor
pros::Motor Intake(8, pros::MotorGearset::blue);
// defines lift motor
pros::Motor Lift (1, pros::MotorGearset::green);
pros::adi::Pneumatics MOGO{'E', false}; // Create a Pneumatics object for mogo on port E
pros::adi::Pneumatics intakelift{'D',false}; // Create a Pneumatics object for intake on port D
pros::adi::Pneumatics doinker{'C', false}; // Create a Pneumatics object for Ring clamp on port C
pros::adi::DigitalIn liftswitch('F');
// ODOMETRY
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              13, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// vertical tracking wheel encoder
pros::adi::Encoder horizontal_encoder('A', 'B');
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -4.4);

// create an imu on port 5
pros::Imu imu(5);
lemlib::PID liftPID (4, 1, 3.3,0,true );
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
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

// angular PID controller
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
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);
void initialize() {
  chassis.calibrate();// calibrate chassis controller
  liftPID.reset();
  pros::lcd::initialize();\
        //while (true) {

           //pros::lcd::print(2, "Y: %f", chassis.getPose().y); // y
			//pros::lcd::print(1, "X: %f", chassis.getPose().x); // y
			//pros::lcd::print(3, "Theta: %f", chassis.getPose().theta); // y
			//pros::delay(10);
		//}
  pros::adi::DigitalOut MOGO ('a');
  pros::adi::DigitalOut intakelift ('b');
  pros::adi::DigitalOut doinker ('c');
  pros::adi::DigitalIn liftswitch('F');
  Lift.set_brake_mode(pros::MotorBrake::hold);
  Intake.set_brake_mode(pros::MotorBrake::coast);
}

void liftmoveload(){
    
    float ratio = 60/12;
    float targ = 51*ratio; //sets the target for the pid to reach
    while (true) { //forever loop to allow the PID to function until it reaches desired value
    pros::delay(10); //delay to save resources
    float liftPos = Lift.get_position();  // gets the lift position as the position from the IME
    float err = targ-liftPos; //finds the error between the desired value and the current value 
    if (std::abs(err) < 10) return; //defines the range within which the lift will not move 
    float input = liftPID.update(err); 
    float max = 100; float min = -100;
    input = std::clamp(input, min, max);
    Lift.move(input); //tells the lift to move with the voltage defined by the distance from target
    
  }}
void liftmovebottom(){
    
    float ratio = 60/12;
    float targ = 0*ratio; //sets the target for the pid to reach
    while ( (pros::c::adi_port_get_value('F') == 0)) 
    { //forever loop to allow the PID to function until it reaches desired value
    pros::delay(10); //delay to save resources
    float liftPos = Lift.get_position();  // gets the lift position as the position from the IME
    float err = targ-liftPos; //finds the error between the desired value and the current value 
    if (std::abs(err) < 10) return; //defines the range within which the lift will not move 
    float input = liftPID.update(err); 
    float max = 100; float min = -100;
    input = std::clamp(input, min, max);
    Lift.move(input); //tells the lift to move with the voltage defined by the distance from target
}}
  void liftmovescore(){
    
    float ratio = 60/12;
    float targ = 150*ratio; //sets the target for the pid to reach
   Intake.move(127);
    while (true) { //forever loop to allow the PID to function until it reaches desired value
    pros::delay(10); //delay to save resources
    float liftPos = Lift.get_position();  // gets the lift position as the position from the IME
    float err = targ-liftPos; //finds the error between the desired value and the current value 
    if (std::abs(err) < 10) return; //defines the range within which the lift will not move 
    float input = liftPID.update(err); 
    float max = 100; float min = -100;
    input = std::clamp(input, min, max);
    Lift.move(input); //tells the lift to move with the voltage defined by the distance from target
    
  }}

void disabled() {}
void competition_initialize() {}
void autonomous() {
}


void opcontrol() {



  while (true) {
        // get left y and right x positions
        int leftY = Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);
		 if (pros::c::adi_port_get_value('F') == 1) {
  Lift.tare_position();
} 
        // delay to save resources
        pros::delay(25);
  if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      Intake.move(127);
    } else if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      Intake.move(-127);
    } else
      Intake.move(0);
  if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    Lift.move(-127);
  } else if (Master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    Lift.move(127);
  } else
    Lift.move(0);
  if(Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      intakelift.toggle();
	    }
  if (Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){ 
    MOGO.toggle();
  }
  if (Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
    doinker.toggle();
  }
  if (Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
    pros::Task liftmovebottom1(liftmovebottom);
  }
if (Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
    pros::Task liftmoveload1(liftmoveload);
  }
  if (Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
    pros::Task liftmovescore1(liftmovescore);
  }
  }}
  
   
