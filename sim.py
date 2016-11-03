def simulate_drop(m, x, y, z, vx, vy, vz, dx, dy, dz, wx, wy, wz, step = 0.01):
    t = 0
    g = 9.81

    while z > 0:

        t += step
        
        fx = dx * (vx-wx)**2
        fy = dy * (vy-wy)**2
        fz = dz * (vz-wz)**2 - g

        ax = fx / m
        ay = fy / m
        az = fz / m

        x += vx*step + 0.5*ax*step**2
        y += vy*step + 0.5*ay*step**2
        z += vz*step + 0.5*az*step**2

        vx += ax*step
        vy += ay*step
        vz += az*step

    return (t, x, y, z)
