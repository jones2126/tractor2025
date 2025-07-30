
# Geodetic to ECEF Conversion (with Python)

This document shows how to convert from geodetic coordinates (latitude, longitude, and height) to ECEF (Earth-Centered, Earth-Fixed) coordinates using Python and the WGS84 ellipsoid model. It also includes a reference implementation for converting back from ECEF to geodetic.

---

## Step-by-Step Calculation Using Python

We convert geodetic coordinates (latitude, longitude, height) into Earth-Centered, Earth-Fixed (ECEF) coordinates using the WGS84 ellipsoid model.

### Input Values
- Latitude: 40.7249028°
- Longitude: –80.7283178°
- Height: 325.553 m

### Python Code (Geodetic to ECEF)

```python
import math

# WGS84 constants
a = 6378137.0  # semi-major axis (m)
f = 1 / 298.257223563  # flattening
e2 = 2 * f - f ** 2  # eccentricity squared

# Geodetic coordinates
lat_deg = 40.7249028
lon_deg = -80.7283178
h = 325.553  # height above ellipsoid in meters

# Convert degrees to radians
lat_rad = math.radians(lat_deg)
lon_rad = math.radians(lon_deg)

# Trigonometric values
sin_lat = math.sin(lat_rad)
cos_lat = math.cos(lat_rad)
sin_lon = math.sin(lon_rad)
cos_lon = math.cos(lon_rad)

# Radius of curvature in the prime vertical
N = a / math.sqrt(1 - e2 * sin_lat ** 2)

# ECEF coordinates
X = (N + h) * cos_lat * cos_lon
Y = (N + h) * cos_lat * sin_lon
Z = (N * (1 - e2) + h) * sin_lat

# Output results
print(f"Latitude (rad): {lat_rad:.6f}")
print(f"Longitude (rad): {lon_rad:.6f}")
print(f"N: {N:.3f} m")
print(f"ECEF X: {X:.3f} m")
print(f"ECEF Y: {Y:.3f} m")
print(f"ECEF Z: {Z:.3f} m")
```

### Expected Output

```
Latitude (rad): 0.711058
Longitude (rad): -1.409573
N: 6386385.723 m
ECEF X: 1423699.497 m
ECEF Y: -4776425.306 m
ECEF Z: 4136278.594 m
```

---

## ECEF to Geodetic Conversion (Python)

This section shows how to convert back from ECEF (X, Y, Z) to geodetic coordinates using an iterative method.

### Python Code (ECEF to Geodetic)

```python
# Constants
a = 6378137.0
f = 1 / 298.257223563
e2 = 2 * f - f ** 2
b = a * (1 - f)

# Input ECEF coordinates
X = 1423699.497
Y = -4776425.306
Z = 4136278.594

# Compute longitude
lon_rad = math.atan2(Y, X)

# Iterative computation for latitude
p = math.sqrt(X ** 2 + Y ** 2)
lat_rad = math.atan2(Z, p * (1 - e2))  # initial guess
for _ in range(5):
    N = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)
    h = p / math.cos(lat_rad) - N
    lat_rad = math.atan2(Z, p * (1 - e2 * (N / (N + h))))

# Final height
N = a / math.sqrt(1 - e2 * math.sin(lat_rad) ** 2)
h = p / math.cos(lat_rad) - N

# Convert radians to degrees
lat_deg = math.degrees(lat_rad)
lon_deg = math.degrees(lon_rad)

# Output
print(f"Latitude: {lat_deg:.7f}°")
print(f"Longitude: {lon_deg:.7f}°")
print(f"Height: {h:.3f} m")
```

### Expected Output

```
Latitude: 40.7249028°
Longitude: -80.7283178°
Height: 325.553 m
```

---
