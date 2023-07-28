def calculate_motor_torque(number_of_wheels, robot_weight, payload, wheel_radius):
    g = 9.81  # acceleration due to gravity in m/s^2
    total_weight = robot_weight + payload
    motor_torque = total_weight * g * wheel_radius / number_of_wheels
    return motor_torque

if __name__ == "__main__":
    try:
        number_of_wheels = int(input("Enter the number of wheels: "))
        robot_weight = float(input("Enter the robot's weight in kg: "))
        payload = float(input("Enter the payload in kg: "))
        wheel_radius = float(input("Enter the wheel's radius in meters: "))
        
        motor_torque = calculate_motor_torque(number_of_wheels, robot_weight, payload, wheel_radius)
        print(f"Motor Torque: {motor_torque:.2f} Nm")
    except ValueError:
        print("Invalid input. Please enter valid numeric values.")