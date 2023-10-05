import math

# Function to convert radians to degrees
def radians_to_degrees(radians):
    degrees = radians * (180.0 / math.pi)
    return degrees

# Input the value in radians
radians = float(input("Enter radians: "))

# Convert radians to degrees using the function
degrees = radians_to_degrees(radians)

# Display the result
print("Degrees:", degrees)
