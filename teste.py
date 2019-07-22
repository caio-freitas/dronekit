import dronekit
from dronekit import VehicleMode
import time
# Connect to the Vehicle (in this case a UDP endpoint)
vehicle = dronekit.connect('127.0.0.1:14540', wait_ready=True)
#params = vehicle.parameters

# Connect to the Vehicle via USB telemetry
#vehicle = dronekit.connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

# vehicle is an instance of the Vehicle class
# while not vehicle.is_armable:
#     time.sleep(0.01)
#     vehicle.arm()
vehicle.disarm()
time.sleep(2)
vehicle.arm()
time.sleep(2)
bat = vehicle.battery
bat.current
bat.voltage
bat.level
gim = vehicle.gimbal
loc = dronekit.LocationGlobal(0,0)
#while not KeyboardInterrupt:
    #gim.target_location(loc)
    #gim.pitch(3)
    #i=1
#loc = dronekit.LocationGlobalRelative(90,90,alt=3)

vehicle.simple_takeoff(3)
crtime = time.time()
while time.time() - crtime < 5:
    print(vehicle.location.global_frame)
    print("Armed: " + str(vehicle.armed))
    time.sleep(0.2)
print("Setting AUTO mode")
#vehicle.mode = dronekit.VehicleMode('AUTO')
#vehicle.mode = VehicleMode("AUTO")
#vehicle.mode = dronekit.VehicleMode('LOITER')
precision_land_mode = mavlink_messages = vehicle.message_factory.command_long_encode(
    0, 0,    # target_system, target_component
    dronekit.mavutil.mavlink.MAV_CMD_NAV_LAND, #command
    0, #confirmation
    0, # param 1 - minimum target altitude (0 -> default)
    1, # param 2 - precision land mode ->
    0, # param 3 - Empty
    0, # param 4 - Yaw angle
    0, # param 5 - Latitude
    0, # param 6 - Longitude
    500) # param 7 - Ground level


# send command to vehicle
for i in range(30):
    vehicle.mode = VehicleMode("AUTO")
    print(vehicle.mode)
    vehicle.send_mavlink(precision_land_mode)
    time.sleep(0.01)
#params.set("PRECISION_LAND_MODE", 2)
# params.set(name="SYS_AUTOSTART", value = 14001)
# print("Param SYS_AUTOSTART to 14001") PRECISION_LAND_MODE

#params.set(name="MAV_CMD_NAV_LAND", value=2)
crtime = time.time()
while not KeyboardInterrupt():
    print(vehicle.location.global_frame)
    print("Armed: " + str(vehicle.armed))
    time.sleep(0.2)
vehicle.disarm()
#print(params.get(name="SYS_AUTOSTART"))
print(bat.current, bat)
