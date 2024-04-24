#include <stdio.h>
#include <gm6020_can.h>

int main() {
    int result = rust_function(1, 2);
    printf("Called Rust function, result: %d\n", result);
    return 0;
}
