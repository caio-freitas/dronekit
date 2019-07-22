import dronekit
from pymavlink import mavutil
import time
from dronekit import Command

connection_str = '127.0.0.1:14540'

MAV_MODE_PREFLIGHT = 0
MAV_MODE_AUTO = 4

# Stabilize: Controle RC com estabilizacao
MAV_MODE_STABILIZE_DISARMED = 80
MAV_MODE_STABILIZE_ARMED = 208

# Guided: Controle Autonomo, manual setpoints
MAV_MODE_GUIDED_DISARMED = 88
MAV_MODE_GUIDED_ARMED = 216

# AUTO: Controle autonomo, trajetoria autonoma
MAV_MODE_AUTO_DISARMED = 92
MAV_MODE_AUTO_ARMED = 220

# Test: development
MAV_MODE_TEST_DISARMED = 66
MAV_MODE_TEST_ARMED = 194

#MAV_MODE_PRECISION_LANDING

log = dronekit.logging

def check_EKF(vehicle):
    while not vehicle.ekf_ok:
        log.warning("EKF not OK!")
        time.sleep(0.1)
    log.warning("\n\nEKF OK!\n\n")

def safe_arm(vehicle):
    while not vehicle.is_armable:
        log.warning("Vehicle not armable")
    vehicle.arm()

def set_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        vx, vy, vz, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

def hover(vehicle, t):
    f = 50.0    #50 Hz
    init_time = time.time()

    # msg = vehicle.message_factory.set_position_target_local_ned_encode(
    #     0,       # time_boot_ms (not used)
    #     0, 0,    # target system, target component
    #     mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, # frame
    #     1, #confirmation
    #     1, # param 1 - Speed Type: 0=air speed, 1=ground speed, 2=climb speed, 3=descent speed
    #     0, # param 2 - Speed
    #     -1, # param 3 - Throttle (-1 = no change)
    #     0, # param 4 - Relative (boolean)
    #     0, # param 5 - Empty
    #     0, # param 6 - Empty
    #     0) # param 7 - Empty

    while not time.time() - init_time > t:
        log.warning("Setting velocity to (0,0,0)")
        vehicle._master.mav.command_long_send(vehicle._master.target_system,
            vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            1, #confirmation
            1, # param 1 - Speed Type: 0=air speed, 1=ground speed, 2=climb speed, 3=descent speed
            0, # param 2 - Speed
            -1, # param 3 - Throttle (-1 = no change)
            0, # param 4 - Relative (boolean)
            0, # param 5 - Empty
            0, # param 6 - Empty
            0) # param 7 - Empty
        #vehicle.send_mavlink(msg)
        time.sleep(1/f)

def precision_land(vehicle):
    # precision_land_mode = mavlink_messages = vehicle.message_factory.command_long_encode(
    #     0, 0,    # target_system, target_component
    #     dronekit.mavutil.mavlink.MAV_CMD_NAV_LAND, #command
    #     0b00000100, #confirmation
    #     0, # param 1 - minimum target altitude (0 -> default)
    #     1, # param 2 - precision land mode ->
    #     0, # param 3 - Empty
    #     0, # param 4 - Yaw angle
    #     0, # param 5 - Latitude
    #     0, # param 6 - Longitude
    #     500) # param 7 - Ground level
    # print(dronekit.mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT)
    # # send command to vehicle
    # vehicle.send_mavlink(precision_land_mode)

    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
            1,
            0, # param 1 - minimum target altitude (0 -> default)
            1, # param 2 - precision land mode -> 1=opportunistic, 2=required
            0, # param 3 - Empty
            0, # param 4 - Yaw angle
            0, # param 5 - Latitude
            0, # param 6 - Longitude
            0) # param 7 - Ground level

    # cmds = vehicle.commands
    # precland = Command(vehicle._master.target_system,
    # vehicle._master.target_component,
    # 0, # sequence number within the mission (auto set by API)
    # mavutil.mavlink.MAV_FRAME_LOCAL_NED, # Frame
    # mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0,
    # 0, # param 1 - minimum target altitude (0 -> default)
    # 2, # param 2 - precision land mode -> 1=opportunistic, 2=required
    # 0, # param 3 - Empty
    # 0, # param 4 - Yaw angle
    # 0, # param 5 - Latitude
    # 0, # param 6 - Longitude
    # 0) # param 7 - Ground level
    # cmds.add(precland)
    # cmds.upload()

def PX4setMode(vehicle, mavMode):
    log.warning("Setting flight mode to " + str(mavMode))
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                                   mavMode,
                                               0, 0, 0, 0, 0, 0)

                                               
def main():
    vehicle = dronekit.connect(connection_str, wait_ready=True)
    #check_EKF(vehicle)
    #safe_arm(vehicle)
    vehicle.arm()

    cmd = vehicle.commands
    cmd.clear()

    time.sleep(2)
    PX4setMode(vehicle, MAV_MODE_AUTO_ARMED)
    time.sleep(0.2)
    vehicle.simple_takeoff(alt=500)
    #hover(vehicle, 30)
    PX4setMode(vehicle, MAV_MODE_AUTO_ARMED)
    time.sleep(10)
    precision_land(vehicle)
    #check_EKF()




if __name__ == "__main__":
    main()
