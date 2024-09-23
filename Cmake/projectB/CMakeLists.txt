# ProjectB/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(ProjectB)

# Add ProjectA's library using add_subdirectory()
add_subdirectory(../projectA build_A)

# Create the library
add_library(libB src/libB.cpp)

# Include the directory for the library
target_include_directories(libB PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Create the executable for ProjectB's main program
add_executable(ProjectB src/main.cpp)

# Link both libA and libB to the executable
target_link_libraries(ProjectB PRIVATE libA libB)
