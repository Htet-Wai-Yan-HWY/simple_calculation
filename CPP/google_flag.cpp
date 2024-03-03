
#include <gflags/gflags.h>
#include <iostream>

// Define your command-line flags
DEFINE_bool(verbose, false, "Display verbose output");
DEFINE_int32(repeat, 2, "Number of times to repeat");

int main(int argc, char* argv[]) {
    // Initialize the command-line flags
    google::ParseCommandLineFlags(&argc, &argv, true);

    // Access the parsed flags
    if (FLAGS_verbose) {
        std::cout << "Verbose mode is enabled\n";
    }

    std::cout << "Repeat value: " << FLAGS_repeat << std::endl;

    // Your program logic here

    return 0;
}