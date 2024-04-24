I originally tried to make the gm6020_can package build natively with colcon but the colcon rust plugins are unmaintained so I couldn't get it working.
Instead it is built in the gm6020_ros CMakeLists.txt using Corrosion. This may seem to get stuck at 0% in the build process but that is normal.
The output from `cargo build` is buffered and only displayed when the build completes.
