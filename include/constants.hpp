#pragma once

// Arm
constexpr float MAX_DEGREES = 30000.0f; 
constexpr float WAIT_DEGREES = 3000.0f; // 30 degrees
constexpr float DOWN_DEGREES = 1500.0f; // 15 degrees
constexpr float SCORE_DEGRESS = 17000.0f; 
constexpr float DISCORE_DEGRESS = 13000.0f; 
constexpr float ARM_PID_THRESHOLD = 10.0f;

// Holder
constexpr int AUTO_HOLD_TIMEOUT = 1000; // 1 sec

// Odom
constexpr int MAX_DIST_INCHES = 48;
constexpr int MAX_DIST_MM = 1000;

constexpr double BACK_OFFSET_INCHES = 5;
constexpr double LEFT_OFFSET_INCHES = 6;
constexpr double RIGHT_OFFSET_INCHES = 6;
constexpr double FRONT_OFFSET_INCHES = 6;

constexpr double FEILD_SIZE = 71;

// Conveyor
constexpr double HIGH_THRESHOLD = 330;
constexpr double RED_THRESHOLD = 30; // detect red (HIGH_THRESHOLD, 360) and (0, RED_THRESHOLD)
constexpr double BLUE_THRESHOLD = 160; // detect blue (BLUE_THRESHOLD, HIGH_THRESHOLD)
constexpr int STUCK_CURRENT = 2000;
constexpr int STUCK_RPM = 50;

constexpr double SPEEDRATIO = 1.0;
constexpr double DIST_THRESHOLD = 35;

constexpr bool AUTO_STARTED = true;

constexpr bool RUN_SKILLS = false;
