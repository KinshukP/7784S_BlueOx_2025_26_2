#include "lemlib/api.hpp"
#include "main.h"

/*

                           USER CONFIG SECTION

*/

// ---- Motor Ports ----
pros::MotorGroup left_motors({-14, -15, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({11, 7, 17}, pros::MotorGearset::blue);
pros::Motor lift(10, pros::MotorGearset::blue, pros::MotorUnits::rotations);
pros::Motor intake(1, pros::MotorGearset::blue, pros::MotorUnits::rotations);
// ---- IMU Port ----
pros::Imu imu(6);  // 🔧 CHANGE if your IMU is on different port

// ---- Tracking Wheel Encoder Port ----
pros::Rotation parallel_encoder(9);  // 🔧 CHANGE to your rotation sensor port

// ---- Tracking Wheel Specs ----
constexpr float tracking_wheel_diameter = 2.125;  // 🔧 SET your tracking wheel diameter (inches)
constexpr float tracking_wheel_offset = 3.0;     // 🔧 Distance from center of rotation (inches)

// ---- Drivetrain Specs ----
constexpr float track_width = 10.5;   // 🔧 Measure left-to-right center distance
constexpr int drivetrain_rpm = 450;   // 🔧 Set based on cartridge
constexpr float horizontal_drift = 2; // 🔧 Leave 2 for tank unless tuned

// Pneumatic plugged into Brain 3-wire port A
pros::ADIDigitalOut matchLoader('B');
pros::ADIDigitalOut wing('A');
/*

                           TRACKING WHEEL

*/
// Parallel tracking wheel (forward measurement)
lemlib::TrackingWheel parallel_wheel(
    &parallel_encoder,
    tracking_wheel_diameter,
    tracking_wheel_offset
);

/*

                          DRIVETRAIN

*/

lemlib::Drivetrain drivetrain(
    &left_motors,
    &right_motors,
    track_width,
    lemlib::Omniwheel::OLD_325, // change if using new 4" omnis
    drivetrain_rpm,
    horizontal_drift
);

/*

                           ODOM SENSORS

*/

lemlib::OdomSensors sensors(
    &parallel_wheel,  // parallel wheel
    nullptr,          // second parallel (none)
    nullptr,          // perpendicular (none)
    nullptr,          // second perpendicular (none)
    &imu              // IMU
);

/*

                           CHASSIS

*/

// Controller settings (tune later)
lemlib::ControllerSettings linear_controller(
    10,   // kP
    0,    // kI
    90,    // kD
    0,    // anti windup
    0,    // small error range
    0,  // small error timeout
    0,    // large error range
    0,  // large error timeout
    0.50    // max accel
);
lemlib::ControllerSettings angular_controller(
    3,  // kP from Python turn PID
    0,    // kI
    40,    // kD
    0,
    0,
    0,
    0,
    0,
    2
);
lemlib::Chassis chassis(
    drivetrain,
    linear_controller,
    angular_controller,
    sensors
);

/*
 
                          INITIALIZE

*/

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "LemLib Odom Init");

    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(10);
    }

    chassis.calibrate();  // calibrates tracking + IMU
}

/*

                            AUTONOMOUS

*/
void score(){
    //intake.move(127);
    lift.move(-127);
    pros::delay(3000);
    // intake.move(0);
    lift.move(0);
}

void autonomous() {
    chassis.setPose(0, 0, 0);
    // imu.set_heading(0);

    // 🔹 BEFORE MOVE
    pros::lcd::set_text(1, "Before move");

    pros::lcd::set_text(3, "y: " + std::to_string(chassis.getPose().y));

    // 🔹 Step#1 
    // matchLoader.set_value(false);
    chassis.moveToPoint(0,-30, 2000,{.forwards=false});
    chassis.waitUntilDone();
    chassis.turnToHeading(90,1200);
    chassis.waitUntilDone();
    matchLoader.set_value(true);
    intake.move(127);
    chassis.moveToPoint(-18, -30, 3000, {.forwards = false});
    chassis.waitUntilDone();
    // //FINSIHes matchloadinghere and moves back
    intake.move(0); // stop intake after collecting
    // //rn aligner is
    chassis.moveToPoint(10, -30, 1500, {.forwards = true});
    chassis.waitUntilDone();
    matchLoader.set_value(false);
    chassis.moveToPoint(10, -46, 1500, {.forwards = true});
    chassis.waitUntilDone();
   
    chassis.turnToHeading(90, 1300);
    chassis.waitUntilDone();
    
    // // Move to Otherside
    chassis.moveToPoint(90, -40, 4200, {.forwards = true});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(90, -32, 1200, {.forwards = true});
    chassis.turnToHeading(270, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(33, -32, 1600, {.forwards = true});
    chassis.waitUntilDone();
   
    //score
    intake.move(127);
    score();
    //going back
    matchLoader.set_value(true);
    chassis.setPose(0,0,0);
    intake.move(127);
    chassis.moveToPoint(3, -50, 4000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    // intake.move(0);
    chassis.moveToPoint(0, 0, 1500, {.forwards = true});
    chassis.waitUntilDone();
    matchLoader.set_value(false);

    //scoring
    score();
    intake.move(0);

    // Move to Match loader 3
    chassis.moveToPose(20, -13, 180, 1500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(96,-20, 3500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 1300);
    chassis.waitUntilDone();
    intake.move(127);
    matchLoader.set_value(true);
    //chassis.moveToPose(120, -30, 290, 1500, {.forwards = false});
    chassis.moveToPoint(98, -58, 3000, {.forwards = false});
    chassis.waitUntilDone();
    // MOve back a little
    chassis.moveToPoint(98, -15, 1500, {.forwards = true});
    chassis.waitUntilDone();
    matchLoader.set_value(false);
    intake.move(0);
    // MOve to goal 2 other end
    chassis.turnToHeading(90, 1300);
    chassis.waitUntilDone();
    chassis.moveToPoint(105, -20, 1500, {.forwards = true});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 1300);
    chassis.waitUntilDone();
    chassis.moveToPoint(105, 85, 3000, {.forwards = true});
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 1300);
    chassis.waitUntilDone();
    chassis.moveToPoint(98, 85, 1500, {.forwards = true});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 1300);
    chassis.waitUntilDone();
    intake.move(127);
    // align goal 2
    chassis.moveToPoint(98, 45, 1500, {.forwards = true});
    chassis.waitUntilDone();
    score();
    intake.move(0);
    chassis.moveToPoint(50, 100, 1500, {.forwards = false});
    chassis.waitUntilDone();
    
    // chassis.moveToPoint(93, 55, 4000, {.forwards = true});
    // chassis.waitUntilDone();
    
    // chassis.moveToPoint(115, 90, 3000, {.forwards = false});
    // chassis.waitUntilDone();






    // 🔹 AFTER MOVE
    pros::lcd::set_text(4, "After move");
    pros::lcd::set_text(5, "wheel: " + std::to_string(parallel_wheel.getDistanceTraveled()));
    pros::lcd::set_text(6, "y: " + std::to_string(chassis.getPose().y));

}

/*

                         OP CONTROL

*/

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(forward, turn);

        pros::delay(20);
    }
    // chassis.setPose(0, 0, 0);
    // imu.set_heading(0);
    // chassis.turnToHeading(-90, 2000);
    // chassis.waitUntilDone();
}
