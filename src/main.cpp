#include "lemlib/api.hpp"
#include "main.h"

/*
|--------------------------------------------------------------------------
|                          🔧 USER CONFIG SECTION
|--------------------------------------------------------------------------
*/

// ---- Motor Ports ----
pros::MotorGroup left_motors({-14, -15, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({11, 7, 17}, pros::MotorGearset::blue);

// ---- IMU Port ----
pros::Imu imu(6);  // 🔧 CHANGE if your IMU is on different port

// ---- Tracking Wheel Encoder Port ----
pros::Rotation parallel_encoder(9);  // 🔧 CHANGE to your rotation sensor port

// ---- Tracking Wheel Specs ----
constexpr float tracking_wheel_diameter = 3.25;  // 🔧 SET your tracking wheel diameter (inches)
constexpr float tracking_wheel_offset = 3.0;     // 🔧 Distance from center of rotation (inches)

// ---- Drivetrain Specs ----
constexpr float track_width = 10.5;   // 🔧 Measure left-to-right center distance
constexpr int drivetrain_rpm = 450;   // 🔧 Set based on cartridge
constexpr float horizontal_drift = 2; // 🔧 Leave 2 for tank unless tuned

/*
|--------------------------------------------------------------------------
|                          🛠 TRACKING WHEEL
|--------------------------------------------------------------------------
*/

// Parallel tracking wheel (forward measurement)
lemlib::TrackingWheel parallel_wheel(
    &parallel_encoder,
    tracking_wheel_diameter,
    tracking_wheel_offset
);

/*
|--------------------------------------------------------------------------
|                          🚗 DRIVETRAIN
|--------------------------------------------------------------------------
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
|--------------------------------------------------------------------------
|                          🧭 ODOM SENSORS
|--------------------------------------------------------------------------
*/

lemlib::OdomSensors sensors(
    &parallel_wheel,  // parallel wheel
    nullptr,          // second parallel (none)
    nullptr,          // perpendicular (none)
    nullptr,          // second perpendicular (none)
    &imu              // IMU
);

/*
|--------------------------------------------------------------------------
|                          🤖 CHASSIS
|--------------------------------------------------------------------------
*/

// Controller settings (tune later)
lemlib::ControllerSettings linear_controller(
    10,   // kP
    0,    // kI
    3,    // kD
    3,    // anti windup
    1,    // small error range
    100,  // small error timeout
    3,    // large error range
    500,  // large error timeout
    20    // max accel
);

lemlib::ControllerSettings angular_controller(
    2, 0, 10, 3, 1, 100, 3, 500, 0
);

lemlib::Chassis chassis(
    drivetrain,
    linear_controller,
    angular_controller,
    sensors
);

/*
|--------------------------------------------------------------------------
|                          🚀 INITIALIZE
|--------------------------------------------------------------------------
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
|--------------------------------------------------------------------------
|                          🤖 AUTONOMOUS
|--------------------------------------------------------------------------
*/

void autonomous() {
    chassis.setPose(0, 0, 0);

    chassis.moveToPoint(24, 0, 2000);  // Move forward 24 inches
    chassis.turnToHeading(90, 2000);   // Turn to 90 degrees
    chassis.moveToPoint(24, 24, 2000);
}

/*
|--------------------------------------------------------------------------
|                          🎮 OP CONTROL
|--------------------------------------------------------------------------
*/

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(forward, turn);

        pros::delay(20);
    }
}