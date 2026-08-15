cmake_minimum_required(VERSION 3.16)

# NONE: nothing here is compiled by CMake. The binary is built by cargo, and
# declaring no languages skips the C and C++ compiler probes entirely.
project(octomap_server_rs NONE)

find_package(ament_cmake REQUIRED)

find_program(CARGO_EXECUTABLE cargo)
if(NOT CARGO_EXECUTABLE)
  message(FATAL_ERROR
    "cargo was not found. Install Rust (https://rustup.rs) and make sure "
    "~/.cargo/bin is on PATH for the shell running colcon.")
endif()

# Keep cargo's output inside the colcon build tree rather than beside the
# sources, so `colcon build --cmake-clean-cache` and a plain `rm -rf build/`
# actually clean it.
set(CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo")
set(CARGO_BINARY "${CARGO_TARGET_DIR}/release/octomap_server_rs")

# BYPRODUCTS lets Ninja know what this target produces; without it a rebuild
# after `rm` of the binary would be skipped.
add_custom_target(octomap_server_rs_cargo ALL
  COMMAND ${CMAKE_COMMAND} -E env
          "CARGO_TARGET_DIR=${CARGO_TARGET_DIR}"
          ${CARGO_EXECUTABLE} build --release
          --manifest-path "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml"
  BYPRODUCTS "${CARGO_BINARY}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  COMMENT "Building octomap_server_rs with cargo"
  VERBATIM
)

# lib/${PROJECT_NAME} is where ros2 run looks for a package's executables.
install(PROGRAMS "${CARGO_BINARY}"
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
