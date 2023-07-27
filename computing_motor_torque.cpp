#include <iostream>

double calculate_motor_torque(int number_of_wheels, double robot_weight, double payload, double wheel_radius) {
    const double g = 9.81; // acceleration due to gravity in m/s^2
    double total_weight = robot_weight + payload;
    double motor_torque = total_weight * g * wheel_radius / number_of_wheels;
    return motor_torque;
}

int main() {
    int number_of_wheels;
    double robot_weight, payload, wheel_radius;

    std::cout << "Enter the number of wheels: ";
    std::cin >> number_of_wheels;

    std::cout << "Enter the robot's weight in kg: ";
    std::cin >> robot_weight;

    std::cout << "Enter the payload in kg: ";
    std::cin >> payload;

    std::cout << "Enter the wheel's radius in meters: ";
    std::cin >> wheel_radius;

    double motor_torque = calculate_motor_torque(number_of_wheels, robot_weight, payload, wheel_radius);
    std::cout << "Motor Torque: " << motor_torque << " Nm" << std::endl;

    return 0;
}