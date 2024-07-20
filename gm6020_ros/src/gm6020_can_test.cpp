#include <stdio.h>
#include <gm6020_can.h>
#include <thread>
#include <chrono>
#include <iostream>
// export LD_LIBRARY_PATH=/home/$USER/phalanx/gm6020_ros/gm6020_ros/gm6020_can/target/release/:$LD_LIBRARY_PATH
// pushd gm6020_can && cargo build --release ; popd && g++ src/gm6020_can_test.cpp -I gm6020_can/include/ -L gm6020_can/target/release/ -lgm6020_can -lpthread -o test && ./test

const unsigned int RATE = 50; // Don't want it too high because CAN bus will run out of buffer
const unsigned int PERIOD = 1.0/RATE;
const unsigned int INC = 10;
const int MAX = V_MAX * 10;
const int ID = 1;
int main() {
    Gm6020Can * gmc = init("can0");
    run(gmc, PERIOD);

    for (int voltage = 0; voltage <= MAX; voltage += 2) {
        cmd_single(gmc, CmdMode::Voltage, ID, voltage / 10.0);
        std::this_thread::sleep_for (std::chrono::milliseconds(INC));
    }
    for (int voltage = MAX; voltage > 0; voltage -= 2) {
        cmd_single(gmc, CmdMode::Voltage, ID, voltage / 10.0);
        std::this_thread::sleep_for (std::chrono::milliseconds(INC));
    }
    for (int voltage = 0; voltage >= -1*MAX; voltage -=2) {
        cmd_single(gmc, CmdMode::Voltage, ID, voltage / 10.0);
        std::this_thread::sleep_for (std::chrono::milliseconds(INC));
    }
    for (int voltage = -1*MAX+1; voltage < 1; voltage += 2) {
        cmd_single(gmc, CmdMode::Voltage, ID, voltage / 10.0);
        std::this_thread::sleep_for (std::chrono::milliseconds(INC));
    }
    cmd_single(gmc, CmdMode::Voltage, ID, 1.0);
    while (true){
        std::this_thread::sleep_for (std::chrono::milliseconds(50));
        std::cout<<get(gmc, ID, FbField::Position)<<std::endl;
    }

    return 0;
}
