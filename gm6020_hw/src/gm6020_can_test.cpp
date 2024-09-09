#include <stdio.h>
#include <gm6020_can.h>
#include <thread>
#include <chrono>
#include <iostream>

//////
// Basic C++ example showing how to use gm6020_can library. Corresponds to gm6020_can/examples/gm6020_can_test.rs
//////
/*
export LD_LIBRARY_PATH=/colcon_ws/src/gm6020_ros/gm6020_can/target/release
pushd gm6020_can && cargo build --release ; popd && g++ src/gm6020_can_test.cpp -I gm6020_can/include/ -L gm6020_can/target/release/ -lgm6020_can -lpthread -o test && ./test
*/

const unsigned int RATE = 100; // Should be above 100Hz. At slower rates the feedback values get weird and eventually the commands stop going out. Could be hardware-dependent.
const unsigned int PERIOD = (1.0/RATE)*1000;
const unsigned int INC = 10; // Time between commands
const int MAX = V_MAX * 10;  // Need the 10x multiplier so we can easily increment in for loops (can't increment floats).
const int ID = 1;            // Motor ID [1,7]
int main() {
    // Open SocketCAN device
    Gm6020Can * gmc = gm6020_can_init("can0");
    if (gmc == nullptr){
        std::cerr<<"Unable to open specified SocketCAN device"<<std::endl;
        return -1;
    }

    // Start another thread to collect feedback values
    gm6020_can_run(gmc, PERIOD);

    // Ramp up, ramp down, ramp up (negative), ramp down (negative)
    for (int voltage = 0; voltage <= MAX; voltage += 2) {
        gm6020_can_cmd_single(gmc, ID, CmdMode::Voltage, voltage / 10.0);
        std::this_thread::sleep_for (std::chrono::milliseconds(INC));
    }
    for (int voltage = MAX; voltage > 0; voltage -= 2) {
        gm6020_can_cmd_single(gmc, ID, CmdMode::Voltage, voltage / 10.0);
        std::this_thread::sleep_for (std::chrono::milliseconds(INC));
    }
    for (int voltage = 0; voltage >= -1*MAX; voltage -=2) {
        gm6020_can_cmd_single(gmc, ID, CmdMode::Voltage, voltage / 10.0);
        std::this_thread::sleep_for (std::chrono::milliseconds(INC));
    }
    for (int voltage = -1*MAX+1; voltage < 1; voltage += 2) {
        gm6020_can_cmd_single(gmc, ID, CmdMode::Voltage, voltage / 10.0);
        std::this_thread::sleep_for (std::chrono::milliseconds(INC));
    }

    // Send constant voltage command and read out position feedback
    gm6020_can_cmd_single(gmc, ID, CmdMode::Voltage, 1.0);
    while (true){
        std::this_thread::sleep_for (std::chrono::milliseconds(50));
        std::cout<<gm6020_can_get(gmc, ID, FbField::Position)<<std::endl;
    }

    return 0;
}
