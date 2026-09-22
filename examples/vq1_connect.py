from target.aigp import SimulatorClient
from miniflight.vehicle import Vehicle


vehicle = Vehicle(SimulatorClient())

print("Connecting to VQ1...")
vehicle.connect()

try:
    print("Connected to VQ1")
    print(vehicle.position)
    print(vehicle.velocity)
finally:
    vehicle.disconnect()
    print("Disconnected from VQ1")
