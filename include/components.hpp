#pragma once
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
inline pros::MotorGroup leftMotors({9, -20, -18},
                            pros::MotorGearset::blue); // left motor group - ports 10, 8 (reversed), 7
inline pros::MotorGroup rightMotors({-5, 8, 6},
                             pros::MotorGearset::blue); // right motor group - ports 1 (reversed), 2 , 3 (reversed)

inline lemlib::ControllerSettings armAngularController(2, // proportional gain (kP)
                                                0, // integral gain (kI)
                                                10, // derivative gain (kD)
                                                3, // anti windup
                                                0.5, // small error range, in degrees
                                                100, // small error range timeout, in milliseconds
                                                1, // large error range, in degrees
                                                500, // large error range timeout, in milliseconds
                                                0 // maximum acceleration (slew)
);

// Other Subsystems
inline std::unordered_map<std::string, std::shared_ptr<pros::Distance>> distances {
    {"left", std::make_shared<pros::Distance>(16)},
    {"right", std::make_shared<pros::Distance>(3)},
    {"front", std::make_shared<pros::Distance>(15)}};
inline ArmNamespace::Arm arm(std::make_shared<pros::Motor>(7, pros::v5::MotorGears::red), // arm - motor port 5 (reversed)
                      std::make_shared<pros::Rotation>(-11), // rotation sensor - port 17 (reversed)
                      armAngularController); // PID controller
inline SpinnerNamespace::Spinner intake(std::make_shared<pros::Motor>(17, pros::v5::MotorGears::green)); // intake - motor port 7
inline SpinnerNamespace::Spinner hooks(std::make_shared<pros::Motor>(4, pros::v5::MotorGears::green)); // hooks - motor port 11
inline ConveyorNamespace::Conveyor conveyor(&intake, // intake
                                     &hooks, // hooks
                                     std::make_shared<pros::Optical>(12), // optical sensor - port 14
                                     std::make_shared<pros::Distance>(1) // distance sensor - port 20
);
inline HolderNamespace::Holder holder(std::make_shared<pros::adi::DigitalOut>('A'),
                               std::make_shared<pros::Distance>(6)); // mobile goal holder - port 'B'
inline HolderNamespace::Holder doinker(std::make_shared<pros::adi::DigitalOut>('D')); // doinker - port 'A'
inline HolderNamespace::Holder hanger(std::make_shared<pros::adi::DigitalOut>('B')); // hanger - port 'H'
inline HolderNamespace::Holder intake_raiser(std::make_shared<pros::adi::DigitalOut>('H')); // intake peu - port 'C'

// Inertial Sensor on port 21
inline pros::Imu imu(2);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, reversed
inline pros::Rotation horizontalEnc(11);
// vertical tracking wheel encoder. Rotation sensor, port 19, reversed
inline pros::Rotation verticalEnc(3);
// horizontal tracking wheel. 2" diameter, 3.625" offset, back of the robot (negative)
inline lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -2.25);
// vertical tracking wheel. 2" diameter, 0.5" offset, left of the robot (negative)
inline lemlib::TrackingWheel vertical(&verticalEnc, 2, 0.5);

// drivetrain settings
inline lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.25, // 11.5 inch track width
                              3.25, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
inline lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            0.5, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            1, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
inline lemlib::ControllerSettings angularController(50, // proportional gain (kP)   // 2
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)   // 10 
                                             3, // anti windup
                                             0.5, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             1, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
inline lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu, // inertial sensor
                            &distances // distance sensors
);

// input curve for throttle input during driver control
inline lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
inline lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
inline lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);