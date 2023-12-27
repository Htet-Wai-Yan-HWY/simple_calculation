import math

# Input geodetic coordinates
latitude = 22.01615
longitude = 95.2270
altitude = 0.0  # Assuming altitude is in meters

# Constants for the WGS84 ellipsoid
a = 6378137.0  # Semi-major axis in meters
e_squared = 0.00669437999014  # Eccentricity squared

# Step 1: Convert latitude and longitude to radians
lat_rad = math.radians(latitude)
lon_rad = math.radians(longitude)

# Step 2: Calculate the radius of curvature in the prime vertical (N)
N = a / math.sqrt(1 - e_squared * math.sin(lat_rad)**2)

# Step 3: Calculate ECEF coordinates
X_ecef = (N + altitude) * math.cos(lat_rad) * math.cos(lon_rad)
Y_ecef = (N + altitude) * math.cos(lat_rad) * math.sin(lon_rad)
Z_ecef = (N * (1 - e_squared) + altitude) * math.sin(lat_rad)

# Output ECEF coordinates
print(f'ECEF Coordinates: X = {X_ecef}, Y = {Y_ecef}, Z = {Z_ecef}')
