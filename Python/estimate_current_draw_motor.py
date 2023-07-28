def estimate_current_draw(stall_current, stall_load, target_load):
    estimated_current = (stall_current / stall_load) * target_load
    return estimated_current

def main():
    try:
        stall_current = float(input("Enter the stall current of the motor (in Kg) : "))
        stall_load = float(input("Enter the stall load of the motor (in A): "))
        target_load = float(input("Enter the target load for current estimation (in Kg): "))

        estimated_current = estimate_current_draw(stall_current, stall_load, target_load)
        print(f"Estimated current draw at {target_load} load: {estimated_current:.2f} A")
        
        if (estimated_current > stall_current):
            print("\033[0;31m W:Can't use mortor will burn")

    except ValueError:
        print("Invalid input. Please enter valid numbers.")

if __name__ == "__main__":
    main()