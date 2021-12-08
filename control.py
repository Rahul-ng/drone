from dronekit import *

vehicle = None 
from simple_pid import PID

USE_PID_YAW = True
USE_PID_ROLL = False

MAX_SPEED = 4       # M / s
MAX_YAW = 15        # Degrees / s 

P_YAW = 0.02 #orgineel 0.01
I_YAW = 0
D_YAW = 0

P_ROLL = 0.22
I_ROLL = 0
D_ROLL = 0

control_loop_active = True
pidYaw = None
pidRoll = None
movementYawAngle = 0
movementRollAngle = 0
inputValueYaw = 0
inputValueVelocityX = 0
control_loop_active = True

debug_yaw = None
debug_velocity = None

# Connect to the Vehicle (in this case a UDP endpoint)
def configure_PID(control):
    global pidRoll,pidYaw

    """ Creates a new PID object depending on whether or not the PID or P is used """ 

    print("Configuring control")

    if control == 'PID':
        pidYaw = PID(P_YAW, I_YAW, D_YAW, setpoint=0)       # I = 0.001
        pidYaw.output_limits = (-MAX_YAW, MAX_YAW)          # PID Range
        pidRoll = PID(P_ROLL, I_ROLL, D_ROLL, setpoint=0)   # I = 0.001
        pidRoll.output_limits = (-MAX_SPEED, MAX_SPEED)     # PID Range
        print("Configuring PID")
    else:
        pidYaw = PID(P_YAW, 0, 0, setpoint=0)               # I = 0.001
        pidYaw.output_limits = (-MAX_YAW, MAX_YAW)          # PID Range
        pidRoll = PID(P_ROLL, 0, 0, setpoint=0)             # I = 0.001
        pidRoll.output_limits = (-MAX_SPEED, MAX_SPEED)     # PID Range
        print("Configuring P")

def connect_drone(connection_string):
    global vehicle
    if vehicle == None:
        vehicle = connect(connection_string, wait_ready=True, baud=57600)
    print("drone connected")

def disconnect_drone():
    vehicle.close()

def get_version():
    global vehicle
    return vehicle.version

def get_mission():
    global vehicle
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    return 

def get_location():
    global vehicle
    return vehicle.location.global_frame

def get_altitude():
    global vehicle
    return vehicle.location.global_relative_frame.alt

def get_velocity():
    global vehicle
    return vehicle.velocity

def get_battery_info():
    global vehicle
    return vehicle.battery

def get_mode():
    global vehicle
    return vehicle.mode.name

def get_home_location():
    global vehicle
    return vehicle.home_location

def get_heading():
    global vehicle
    return vehicle.heading

def get_EKF_status():
    return vehicle.ekf_ok

def get_ground_speed():
    return vehicle.groundspeed

def read_channel(channel):
    return vehicle.channels[str(channel)]

def set_gimbal_angle(angle):
    global vehicle
    print("gimbal angle set to: " % angle)
    return vehicle.gimbal.rotate(0, angle, 0)

def set_groundspeed(speed):
    global vehicle
    print("groundspeed set to: " % speed)
    vehicle.groundspeed = speed

def arm():
    vehicle.groundspeed = 3

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("STABILIZE")
    vehicle.armed   = True

    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("ARMED! Let's take OFF")

def arm_and_takeoff(aTargetAltitude):
    global vehicle

    #set default groundspeed low for safety 
    print ("setting groundspeed to 3")
    vehicle.groundspeed = 3

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

def land():
    global vehicle
    print("Setting LAND mode...")
    vehicle.mode = VehicleMode("LAND")

def return_to_launch_location():
    #carefull with using this! It wont detect obstacles!
    vehicle.mode = VehicleMode("RTL")

def send_movement_command_YAW(heading):
    global vehicle
    speed = 0 
    direction = 1 #direction -1 ccw, 1 cw
    
    #heading 0 to 360 degree. if negative then ccw 
    
    print("Sending YAW movement command with heading: %f" % heading)

    if heading < 0:
        heading = heading*-1
        direction = -1

    #point drone into correct heading 
    msg = vehicle.message_factory.command_long_encode(
        0, 0,       
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0,          
        heading,    
        speed,      #speed deg/s
        direction,  
        1,          #relative offset 1
        0, 0, 0)    

    # send command to vehicle
    vehicle.send_mavlink(msg)
    #Vehicle.commands.flush()

def send_movement_command_XYZ(velocity_x, velocity_y, velocity_z):
    global vehicle

    #velocity_x positive = forward. negative = backwards
    #velocity_y positive = right. negative = left
    #velocity_z positive = down. negative = up (Yes really!)

    print("Sending XYZ movement command with v_x(forward/backward): %f v_y(right/left): %f v_z(height): %f" % (velocity_x,velocity_y,velocity_z))

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,      
        0, 0,    
        mavutil.mavlink.MAV_FRAME_BODY_NED, # relative to drone heading
        0b0000111111000111, 
        0, 0, 0,
        velocity_x, velocity_y, velocity_z, 
        0, 0, 0, 
        0, 0)    

    vehicle.send_mavlink(msg)
    vehicle.flush()

# connect_drone("tcp:127.0.0.1:5762")
# configure_PID("PID")
# arm_and_takeoff(10)
# for i in range(20):
#     movementRollAngle = (pidRoll(100) * -1)
    
#     send_movement_command_XYZ(0,10,0)
#     print(movementRollAngle)
#     print(" Velocity: %s" % vehicle.velocity)
#     print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
#     print(" Airspeed: %s" % vehicle.airspeed)
#     print(" Attitude: %s" % vehicle.attitude)
#     time.sleep(1)
# send_movement_command_YAW(pidYaw(15) * -1)
# time.sleep(3)
# land()