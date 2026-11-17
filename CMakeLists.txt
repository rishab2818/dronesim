cmake_minimum_required(VERSION 3.20)
project(drone_simulator VERSION 1.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(ENGINE_SOURCES
    engine/src/vector3.c engine/src/matrix3.c engine/src/quaternion.c
    engine/src/drone_config.c engine/src/motors.c engine/src/controls.c
    engine/src/controller.c engine/src/dynamics.c engine/src/integrator.c
    engine/src/ground.c engine/src/recorder.c engine/src/sim_api.c)

add_library(drone_engine STATIC ${ENGINE_SOURCES})
target_include_directories(drone_engine PUBLIC engine/include)
if(MSVC)
  target_compile_options(drone_engine PRIVATE /W4)
else()
  target_compile_options(drone_engine PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(drone_engine PUBLIC m)
endif()

add_executable(drone_cli engine/cli/main.c)
target_link_libraries(drone_cli PRIVATE drone_engine)

enable_testing()
add_executable(drone_tests engine/tests/test_suite.c)
target_link_libraries(drone_tests PRIVATE drone_engine)
add_test(NAME physics_suite COMMAND drone_tests)

