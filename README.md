# px4_simulation
Gazebo simulation for NRF communication between drones and ground nodes.

The simulated NRF packet can store 10 bits. Actually not bits 10 floating points(Each floating point is either near to 0 or 1). Floating points are stored to simulate the noise as well.
## Eg: The published message is: [-0.056796, 1.001415, -0.051560, 0.119754, 0.888325, 0.944267, 1.143572, 1.011403, 0.905899, 1.021884]
