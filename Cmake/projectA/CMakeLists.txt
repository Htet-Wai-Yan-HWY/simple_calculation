# ProjectA/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(ProjectA)

# Create the library
add_library(libA src/libA.cpp)

# Include the directory for the library
target_include_directories(libA PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Create the executable for ProjectA's main program
add_executable(ProjectA src/main.cpp)

# Link the library to the executable
target_link_libraries(ProjectA PRIVATE libA)

# Optional: Install targets
install(TARGETS libA DESTINATION lib)
install(DIRECTORY src/ DESTINATION include)
