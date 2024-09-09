cmake_minimum_required(VERSION 3.23)
project(gm6020_can LANGUAGES CXX)

include(FetchContent)
FetchContent_Declare(
    Corrosion
    GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
)
# Set any global configuration variables such as `Rust_TOOLCHAIN` before this line!
FetchContent_MakeAvailable(Corrosion)
# Import targets defined in a package or workspace manifest `Cargo.toml` file
corrosion_import_crate(MANIFEST_PATH Cargo.toml PROFILE release)

target_sources(${PROJECT_NAME} INTERFACE
        FILE_SET HEADERS
        BASE_DIRS include
        FILES
        include/${PROJECT_NAME}.h
)

corrosion_install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)

install(
        EXPORT ${PROJECT_NAME}Targets
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION lib/cmake/${PROJECT_NAME}
)

# Necessary for packaging helper commands
include(CMakePackageConfigHelpers)

# Configures the main config file that cmake loads
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
        NO_SET_AND_CHECK_MACRO
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# Config.cmake.in contains
# @PACKAGE_INIT@
# 
# include(${CMAKE_CURRENT_LIST_DIR}/RustLibTargetsCorrosion.cmake)
# include(${CMAKE_CURRENT_LIST_DIR}/RustLibTargets.cmake)

# Install all generated files
install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/corrosion/${PROJECT_NAME}TargetsCorrosion.cmake
        DESTINATION lib/cmake/${PROJECT_NAME}
)
