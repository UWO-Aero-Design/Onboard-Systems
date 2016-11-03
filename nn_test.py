from sklearn.neural_network import MLPRegressor
from sim import simulate_drop
import random

# simulate_drop(m, x, y, z, vx, vy, vz, dx, dy, dz, wx, wy, wz, step)

m = 1
x = 0
y = 0
z = 35
vx = 10
vy = 12
vz = -2
dx = 0.005
dy = 0.005
dz = 0.001
wx = 10
wy = 5
wz = 0

print simulate_drop(m, x, y, z, vx, vy, vz, dx, dy, dz, wx, wy, wz, 0.0001)

nn = MLPRegressor((50, 50, 50, 50))

test_input = []
test_output = []


for i in range(500):
    
    z = random.uniform(25.0, 35.0)
    vx = random.uniform(-10.0, 10.0)
    vy = random.uniform(-10.0, 10.0)
    vz = random.uniform(-10.0, 10.0)

    test_input.append([m, x, y, z, vx, vy, vz, dx, dy, dz, wx, wy, wz])
    t, x, y, z = simulate_drop(m, x, y, z, vx, vy, vz, dx, dy, dz, wx, wy, wz, 0.0001)
    test_output.append([t, x, y, z])

nn.fit(test_input, test_output)

print "Neural network trained"

z = random.uniform(25.0, 35.0)
vx = random.uniform(-10.0, 10.0)
vy = random.uniform(-10.0, 10.0)
vz = random.uniform(-10.0, 10.0)

print nn.predict([m, x, y, z, vx, vy, vz, dx, dy, dz, wx, wy, wz])
print simulate_drop(m, x, y, z, vx, vy, vz, dx, dy, dz, wx, wy, wz, 0.0001)
