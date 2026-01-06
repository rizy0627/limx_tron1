/**
 * @file sim_demo.cpp
 * @brief Simple SDK Demo to control Tron1 in Gazebo
 * 
 * Complie with:
 * g++ -o sim_demo src/sim_demo.cpp -I src/limxsdk-lowlevel/include -L src/limxsdk-lowlevel/lib/amd64 -llimxsdk_lowlevel -lpthread -Wl,-rpath,/home/rizy/planner/tron1/limx_ws/src/limxsdk-lowlevel/lib/amd64
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <atomic>
#include <thread>
#include <chrono>
#include "limxsdk/pointfoot.h"

using namespace limxsdk;

int main(int argc, char** argv) {
    std::cout << "--- Tron1 Simulation Control Demo ---" << std::endl;

    // 1. Get SDK Instance
    PointFoot* robot = PointFoot::getInstance();

    // 2. Connect to Simulation (Localhost)
    std::string robot_ip = "127.0.0.1";
    if (!robot->init(robot_ip)) {
        std::cerr << "[Error] Failed to connect to robot at " << robot_ip << std::endl;
        std::cerr << "Make sure Gazebo simulation is running!" << std::endl;
        return -1;
    }
    
    uint32_t n = robot->getMotorNumber();
    std::cout << "[Info] Connected! Motor count: " << n << std::endl;

    // 3. Subscribe to State (Wait for first data)
    std::vector<float> init_q;
    bool ready = false;

    robot->subscribeRobotState([&](const RobotStateConstPtr& msg) {
        if (!ready) {
            init_q = msg->q;
            ready = true;
            std::cout << "[Info] Received initial robot state." << std::endl;
        }
    });

    std::cout << "[Info] Waiting for robot state..." << std::endl;
    while (!ready) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 4. Control Loop (Move joints to 0.0)
    std::cout << "[Info] Starting control loop. Moving to Zero Pose..." << std::endl;

    RobotCmd cmd(n);
    // Set PD Gains (Tune these if robot shakes or is too soft)
    for(uint32_t i=0; i<n; i++) {
        cmd.Kp[i] = 60.0f; 
        cmd.Kd[i] = 3.0f;
        cmd.tau[i] = 0.0f;
        cmd.dq[i] = 0.0f;
    }

    int steps = 0;
    const int DURATION_STEPS = 3000; // 3 seconds

    while (true) {
        float progress = (float)steps / DURATION_STEPS;
        if (progress > 1.0f) progress = 1.0f;

        for(uint32_t i=0; i<n; i++) {
            // Interpolate from Init -> 0.0
            cmd.q[i] = (1.0f - progress) * init_q[i] + progress * 0.0f;
        }

        robot->publishRobotCmd(cmd);

        steps++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms loop
    }

    return 0;
}

