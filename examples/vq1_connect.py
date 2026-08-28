from target.vq1 import VQ1
from miniflight.vehicle import Vehicle


vehicle = Vehicle(VQ1())

print("Connecting to VQ1...")
vehicle.connect()

try:
    print("Connected to VQ1")
    print(vehicle.position)
finally:
    vehicle.disconnect()
    print("Disconnected from VQ1")
