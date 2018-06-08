import math

pi = math.pi

# Calculate approximate GPS accuracy
print("At the equator,")
# (at equator, where lattitude and longditude have = the same number of meters)
R = 6371000 	# radius of earth, m
C = 2 * pi * R	# circumference of earth at equator, m
# 360 deg = C m  -->  1 deg = C/360 m  -->  1 m = 360/C deg
metresPerDeg = C / 360
degsPerMetre = 360 / C

print(" 1 deg =", metresPerDeg, "m")
print(" 1 m =", degsPerMetre, "deg")

# 1 m = 8.993216059187306e-06 deg ~= 9e-06 deg
# 10 cm ~= 1e-06 deg
# 1 deg = 111194.92664455873 m ~= 111195 m
# so, a good threshold for waypoint reached is deviation < 5e-06 deg  (about +/- 0.5 m)
# metres = degrees * 9e-06
# degrees = metres / 9e-06

# ^^^^
# seems wrong!

# HAND DERIVED
# cf = ~9e-06
# D = ~9e-06 x M
# M = D / ~9e-06