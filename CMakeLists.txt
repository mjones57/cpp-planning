cmake_minimum_required(VERSION 3.16)
project(cpp_planning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Emit compile_commands.json for the editor
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- the library: every algorithm lives here ---
add_library(planning
    src/grid_map.cpp
    src/bfs.cpp
    src/dijkstra.cpp
    src/astar.cpp
    src/planner_factory.cpp
    src/node_pool.cpp
)
target_include_directories(planning PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(planning PRIVATE -Wall -Wextra -Wpedantic)

# --- optional sanitizer build: -DENABLE_SANITIZERS=ON ---
option(ENABLE_SANITIZERS "Build with address and UB sanitizers" OFF)
if(ENABLE_SANITIZERS)
    target_compile_options(planning PUBLIC -fsanitize=address,undefined -g)
    target_link_options(planning PUBLIC -fsanitize=address,undefined)
endif()

# --- the demo executables ---
add_executable(bfs_demo apps/bfs_demo.cpp)
target_link_libraries(bfs_demo PRIVATE planning)
target_compile_options(bfs_demo PRIVATE -Wall -Wextra -Wpedantic)

add_executable(planner_compare apps/planner_compare.cpp)
target_link_libraries(planner_compare PRIVATE planning)
target_compile_options(planner_compare PRIVATE -Wall -Wextra -Wpedantic)

add_executable(frontier_compare apps/frontier_compare.cpp)
target_link_libraries(frontier_compare PRIVATE planning)
target_compile_options(frontier_compare PRIVATE -Wall -Wextra -Wpedantic)

add_executable(node_pool_demo apps/node_pool_demo.cpp)
target_link_libraries(node_pool_demo PRIVATE planning)
target_compile_options(node_pool_demo PRIVATE -Wall -Wextra -Wpedantic)

option(BUILD_TESTING_PLANNING "Build the unit tests" ON)
if(BUILD_TESTING_PLANNING)
    include(FetchContent)
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG                v1.15.2
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    add_executable(planning_tests
        tests/test_grid_map.cpp
        tests/test_planners.cpp
        tests/test_node_pool.cpp
    )
    target_link_libraries(planning_tests PRIVATE planning GTest::gtest_main)
    target_compile_options(planning_tests PRIVATE -Wall -Wextra)

    include(GoogleTest)
    gtest_discover_tests(planning_tests)
endif()