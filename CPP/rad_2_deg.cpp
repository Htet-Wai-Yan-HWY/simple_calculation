#include <iostream>
#include <cmath>

int main() {
    // Declare variables to store radians and degrees
    double radians, degrees;

    // Prompt the user for input
    std::cout << "Enter radians: ";
    std::cin >> radians;

    // Convert radians to degrees using the formula
    degrees = radians * (180.0 / M_PI);

    // Display the result
    std::cout << "Degrees: " << degrees << std::endl;

    return 0;
}
