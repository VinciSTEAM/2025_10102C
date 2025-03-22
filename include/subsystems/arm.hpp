#pragma once

#include "conveyor.hpp"
#include "pros/motor_group.hpp"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"
#include "subsystem.hpp"
#include "spinner.hpp"

namespace ArmNamespace {

enum class State { DOWN, WAIT, UP, DISCORE_UP, SCORE_UP, IDLE };

class Arm : public subsystem<State, Arm> {
    public:
        Arm(std::shared_ptr<pros::Motor> motor, std::shared_ptr<pros::Rotation> rotation,
            lemlib::ControllerSettings armAngularController)
            : motor_(std::move(motor)),
              rotation_(std::move(rotation)),
              armAngularPID(armAngularController.kP, armAngularController.kI, armAngularController.kD,
                            armAngularController.windupRange, true) {
            motor_->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
            motor_->tare_position();
            motor_->set_brake_mode(pros::MotorBrake::coast);
            rotation_->set_position(0);
            rotation_->reset();
            currState = State::DOWN;
            prevState = State::DOWN;
        }

        ~Arm() override = default;

        double arm_state_to_degree(ArmNamespace::State state) const{
            switch (state) {
                case State::DOWN: return DOWN_DEGREES;
                case State::WAIT: return WAIT_DEGREES;
                case State::UP: return MAX_DEGREES;
                case State::SCORE_UP: return SCORE_DEGRESS;
                case State::DISCORE_UP: return DISCORE_DEGRESS;
                default: return 0;
            }
        }
        void move_arm_with_timeout(ArmNamespace::State state, int timeout_ms, int32_t velocity, bool async = false) {
            if (async) {
                pros::Task task([&]() {
                    move_arm_with_timeout(state, timeout_ms, velocity, false);
                });
                pros::delay(10);  // delay to give the task time to start
                return;            
            }        
            
            timer.set(timeout_ms);  // Set the timer
            float angularError = (arm_state_to_degree(state) - rotation_->get_position()) / 100.0;
            // Polling loop to check if the movement is complete or if timeout occurs
            if (fabs(angularError) > ARM_PID_THRESHOLD  && !timer.isDone()) {
                motor_->move(velocity * 1.27); 
            } else {
                motor_->move(0);  // Stop the motor
            }
        }

        void move_arm_with_pid_with_timeout(ArmNamespace::State state, int timeout_ms, int32_t velocity, bool async = false) {
            if (async) {
                pros::Task task([&]() {
                    move_arm_with_pid_with_timeout(state, timeout_ms, velocity, false);
                });
                pros::delay(10);  // delay to give the task time to start
                return;            
            }  
            if (timer.isDone()) {
                timer.set(timeout_ms);  // Set the timer
            }              
            float angularError = (arm_state_to_degree(state) - rotation_->get_position()) / 100.0;
            int sign = angularError > 0 ? 1 : -1;
            // Polling loop to check if the movement is complete or if timeout occurs
            if (fabs(angularError) > ARM_PID_THRESHOLD && !timer.isDone()) {
                // motor_->move(sign *velocity * 1.27);
                motor_->move(armAngularPID.update(angularError));
            } else {
                motor_->move(0);  // Stop the motor
            }
        }

        // Contrpositionols the arm based on button inputs
        void control(bool button_1, bool button_2, bool button_3) { // 2 Y right

            if (button_1) {
                motor_->set_brake_mode(pros::MotorBrake::hold);
                moveToState(State::UP);
            } else if (currState == State::UP) {
                motor_->set_brake_mode(pros::MotorBrake::coast);
                moveToState(State::DOWN);
            }

            if (button_2 && currState == State::DOWN) {
                motor_->set_brake_mode(pros::MotorBrake::hold);
                moveToState(State::WAIT);
                pros::delay(200);  
            } else if (button_2 && currState == State::WAIT) {
                motor_->set_brake_mode(pros::MotorBrake::hold);
                moveToState(State::SCORE_UP);
            } else if (button_2 && currState == State::SCORE_UP) {
                motor_->set_brake_mode(pros::MotorBrake::hold);
                moveToState(State::DOWN);
            }

            if (button_3) {
                motor_->set_brake_mode(pros::MotorBrake::hold);
                moveToState(State::DISCORE_UP);
            }
 
        }

        void changeVelocity(float ratio) { ratio_ = ratio; }

        bool getManual() { return manual_; }

        // Getter for rotation sensor
        std::shared_ptr<pros::Rotation> getRotation() const { return rotation_; }

        // Getter for the arm's state as a string
        std::string getState() const {
            switch (currState) {
                case State::UP: return "UP";
                case State::WAIT: return "WAIT";
                case State::DOWN: return "DOWN";
                case State::SCORE_UP: return "SCORE_UP";
                case State::DISCORE_UP: return "DISCORE_UP";
                default: return "UNKNOWN";
            }
        }
      
        void resetRotation(float rotation) { rotation_->set_position(rotation); }
        bool isTaskDone(double threshold) {
            return fabs(arm_state_to_degree(currState) - rotation_->get_position()) < threshold;
         }
        
        void runTask() {
            if (currState == State::DOWN) motor_->set_brake_mode(pros::MotorBrake::coast);
            else motor_->set_brake_mode(pros::MotorBrake::hold);

            switch (currState) {
                case State::UP:
                    if (rotation_->get_position() <= MAX_DEGREES) {
                        motor_->move(127 * ratio_);
                    } else {
                        motor_->move(0);
                    }
                    break;
                case State::WAIT:
                    if (float angularError = (WAIT_DEGREES - rotation_->get_position()) / 100.0;
                        fabs(angularError) > 2) {
                        motor_->move(armAngularPID.update(angularError));
                    } else {
                        motor_->move(0);
                    }
                    break;
                case State::DOWN:
                    if (rotation_->get_position() > DOWN_DEGREES) {
                        motor_->move(-127 * ratio_);
                    } else {
                        motor_->move(0);
                    }
                    break;

                case State::DISCORE_UP:
                    if (float angularError = (DISCORE_DEGRESS - rotation_->get_position()) / 100.0;
                        fabs(angularError) > 2) {
                        motor_->move(armAngularPID.update(angularError));
                    } else {
                        motor_->move(0);
                    }
                    break;
                case State::SCORE_UP:
                    move_arm_with_pid_with_timeout(State::SCORE_UP, 500, 100, false);
                    // if (float angularError = (SCORE_DEGRESS - rotation_->get_position()) / 100.0;
                    //     fabs(angularError) > 2) {
                    //     motor_->move(armAngularPID.update(angularError));
                    // } else {
                    //     motor_->move(0);
                    // }
                    break;
                case State::IDLE: motor_->move(0); break;
            }
        }
    private:
        std::shared_ptr<pros::Motor> motor_;
        std::shared_ptr<pros::Rotation> rotation_;
        float ratio_ = 1;
        bool manual_;
        // angular motion controller
        lemlib::PID armAngularPID;

};
} // namespace ArmNamespace