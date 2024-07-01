#include <stdio.h>
#include <gm6020_can.h>

// pushd gm6020_can && cargo build && popd && g++ src/gm6020_ros.cpp -I gm6020_can/include/ -L gm6020_can/target/debug/ -lgm6020_can -lpthread -o test && ./test

int main() {
    Gm6020Can * handle = init("can0");
    while(true){
        run_once(handle);
        cmd_single(handle, CmdMode::Voltage, 1, 5.0);
    }
    return 0;
}
