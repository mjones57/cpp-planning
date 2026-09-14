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