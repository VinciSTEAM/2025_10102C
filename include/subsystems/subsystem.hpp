#pragma once

#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#include "constants.hpp"

template <typename StateType, typename Derived> class subsystem {
    public:
        subsystem() {
            task = std::make_unique<pros::Task>([this] { taskRunner(); });
            task->resume();
        }

        virtual ~subsystem() {
            if (task) { task->remove(); }
        }

        StateType getCurrentState() const { return currState; }

        void moveToState(StateType newState) { 
            prevState = currState;
            currState = newState; 
        }

        void moveToState(StateType newState, int time) {
            timer.set(time);
            moveToState(newState);
        }

    protected:
        // CRTP
        // virtual void runTask() = 0;
        StateType prevState, currState;
        
        lemlib::Timer timer {0};
    
        std::unique_ptr<pros::Task> task; // Use a unique_ptr to manage the task

        // Internal task function that runs in the base class
        void taskRunner() {
            while (true) {
                static_cast<Derived*>(this)->runTask(); // Call the derived class's task logic
                pros::delay(10); // Adjust the delay as necessary
            }
        }
};