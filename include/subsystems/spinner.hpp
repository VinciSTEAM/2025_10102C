#pragma once

#include "pros/motors.hpp"
#include "lemlib/timer.hpp"
#include "subsystem.hpp"

namespace SpinnerNamespace {

enum class State {  IDLE, FORWARD, BACKWARD, FORWARD_SLOW, BACKWARD_SLOW, FORWARD_MEDIUM, BACKWARD_MEDIUM, FORWARD_FAST, BACKWARD_FAST };

class Spinner : public subsystem<State, Spinner> {
    public:
        explicit Spinner(std::shared_ptr<pros::Motor> motor)
            : motor_(std::move(motor)) {
            motor_->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
            motor_->tare_position();
        }

        virtual ~Spinner() override = default; // Virtual destructor for safe inheritance

        void stuckDetected() {
            if (motor_->get_actual_velocity() < STUCK_RPM && motor_->get_current_draw() > STUCK_CURRENT) {
                stucked_ = true;
            }
        }
        bool getStuckedStatus() const { return stucked_; }

        // Get motor object
        std::shared_ptr<pros::Motor> getMotor() { return motor_; }
        void runTask() {
            stuckDetected();
            switch (currState) {
                case State::FORWARD: motor_->move(127); break;
                case State::FORWARD_SLOW: motor_->move(0.3 * 127); break;
                case State::FORWARD_MEDIUM: motor_->move(0.5 * 127); break;
                case State::FORWARD_FAST: motor_->move(0.8 * 127); break;
                case State::BACKWARD: motor_->move(-127); break;
                case State::BACKWARD_SLOW: motor_->move(-0.3 * 127); break;
                case State::BACKWARD_MEDIUM: motor_->move(-0.5 * 127); break;
                case State::BACKWARD_FAST: motor_->move(-0.8 * 127); break;
                case State::IDLE: motor_->move(0); break;
            }
        }
    protected:
        std::shared_ptr<pros::Motor> motor_; // Encapsulate member variable
        bool stucked_ = false;
        lemlib::Timer timer {0};

        void move_motor_with_timeout(pros::Motor& motor, double rotations, int timeout_ms, int32_t velocity, bool async = false) {
            if (async) {
                pros::Task task([&]() {
                    move_motor_with_timeout(motor, rotations, timeout_ms, velocity, false);
                });
                pros::delay(10);  // delay to give the task time to start
                return;            
            }
            motor.tare_position();  // Reset the motor's position
            motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);  // Set the brake mode
            // Calculate target position (VEX motors use degrees by default)
            double target_position = rotations * 360;          
            // Set the target position
            motor.move(velocity * 1.27);         
            timer.set(timeout_ms);  // Set the timer
            // Polling loop to check if the movement is complete or if timeout occurs
            while (motor.get_position() < target_position && !timer.isDone()) {
                pros::delay(20);  // Delay for 20 ms
            }
            motor.move(0);  // Stop the motor
        }

};

} // namespace SpinnerNamespace
