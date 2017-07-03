
import time
import serial
import math

'''
* Author : Srijal Poojari
* Description: This program connects with the V-REP remoteApi for 'virtual_dtto.ttt'
*              and is used to control the simulated robot in all manners. Run this script
*              after starting the simulation to connect. After that, it reads data from the
*              specified Serial COM port(which is from bluetooth, in our case)and animates
*              the Dtto modules as per the commands given by user.
*
*              Snake movement and addressing individual joint angles are currently supported.
*              Attaching/Detaching is to be added in future revisions. For any issues or confusions
*              contact me at srijal97@gmail.com.
* Functions:   listen_serial(), set_joint_angle(), cmd_set_angle(), run_sinusoidal().
* Version/Date: 03-Jul-2017
'''

# Initialize serial
port = "COM15"   # Change as per your PC!
baud = 9600

try:
    ser = serial.Serial(port,baud,timeout=0.5)   # Initialize serial
    ser_open = True
except:
    print "********************************************************************"
    print "Serial not initialized!"
    print "********************************************************************"
    ser_open = False

male_joints = [-1]    # Lists to store the joint object handles
female_joints = [-1]  # Index 0 is filled with -1 so that joint 1 gets appended at index 1

total_num_modules = 4

command = ""   # Global string to store received string

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP


# ---------------- Get serial data if available (Bluetooth) ------------------ #
def listen_serial():
    if ser_open:     # If serial open and data available
        if ser.in_waiting > 0:
            raw_data = ser.read(10)   # Read up to 10 bytes
            return raw_data
        else:
            return -1


# ------------------- Set Angle for a particular joint ------------------------ #
def set_joint_angle(final_angle, handle):
    if -90 > final_angle > 90:    # if angle not within -90 and 90,
        print "Invalid joint angle!"
        return

    # Get current joint position
    err_val, current_angle = vrep.simxGetJointPosition(clientID, handle, vrep.simx_opmode_blocking)

    # Check http://www.coppeliarobotics.com/helpFiles/en/apiFunctionListCategory.htm for the functions.
    # Check http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm for all other operating modes.

    current_angle_deg = int(math.degrees(current_angle))   # Convert to degrees

    if current_angle > final_angle:
        for angle in range(current_angle_deg, final_angle - 1, -1):   # slowly move to required angle
            err_val = vrep.simxSetJointTargetPosition(clientID, handle, math.radians(angle), vrep.simx_opmode_oneshot)
            time.sleep(0.01)  # 10ms per degree
    else:
        for angle in range(current_angle_deg, final_angle + 1, 1):    # slowly move to required angle
            err_val = vrep.simxSetJointTargetPosition(clientID, handle, math.radians(angle), vrep.simx_opmode_oneshot)
            time.sleep(0.01)  # 10ms per degree


# -------------- Prepare angle to be set from given command ------------------- #
def cmd_set_angle(cmd):
    module_num = cmd[0]

    if module_num == 'a':    # If for all modules,
        try:
            angle = int(cmd[3:])    # Get angle from command
        except:
            print "ValueError in retrieving angle from command!"
            return

        if cmd[2] == 'm':    # for male joints
            set_joint_angle(angle, male_joints[1])
            set_joint_angle(angle, male_joints[2])
            set_joint_angle(angle, male_joints[3])
            set_joint_angle(angle, male_joints[4])
        elif cmd[2] == 'f':    # for female joints
            set_joint_angle(angle, female_joints[1])
            set_joint_angle(angle, female_joints[2])
            set_joint_angle(angle, female_joints[3])
            set_joint_angle(angle, female_joints[4])


    else:
        module_num = int(module_num)   # Convert char to int
        if 1 > module_num > total_num_modules:  # If less than 1 and greater than number_of_modules
            print "Invalid module number!"
            return
        try:
            angle = int(cmd[3:])    # Get angle from command
        except:
            print "ValueError in retrieving angle from command!"
            return

        if cmd[2] == 'm':   # for male joints
            set_joint_angle(angle, male_joints[module_num])
        elif cmd[2] == 'f':    # for female joints
            set_joint_angle(angle, female_joints[module_num])


# ------------ Run sinusoidal motion or snake/caterpillar motion -------------- #
def run_sinusoidal(cmd):
    t = time.clock()    # Angles are set as per current time, so sine wave moves with time
    speed_factor = 2    # Multiplier for changing speed
    amp_factor = 30     # Angle will vary up to 'amp_factor' degrees from center on each side

    if cmd[2] == 'm':
        reverse_factor = 0  # Specifies movement direction
        # 0: male module forward, 1: female module forward
    elif cmd[2] == 'f':
        reverse_factor = 1  # Specifies movement direction
        # 0: male module forward, 1: female module forward
    else:
        print "Invalid direction for run sinusoidal command!"
        return

    if cmd[0] == 'a':    # If for all modules
        for i in range(1, total_num_modules + 1):
            '''
            * male_angle = amp_factor * sin(theta + phase + pi / 2) = amp_factor * cos(theta + phase)
            * female_angle = amp_factor * sin(theta + phase)
            * cos and sin is interchanged for reverse motion. This can be directly done using reverse_factor
            *
            * theta = time * speed_factor
            * phase = 180 * module_nume, i.e.it negates the sign of cos and sin for every alternate module,
            * since sin(a + pi) = -sin(a) and cos(a + pi) = -cos(a)
            *
            * amplitude factor scales the sin and cos values to degrees.
            '''

            male_joint_angle = amp_factor * math.sin((speed_factor * t) + (i * math.pi)
                                                     + (1 - reverse_factor) * math.pi / 2 )

            err_val = vrep.simxSetJointTargetPosition(clientID, male_joints[i],
                                                      math.radians(male_joint_angle), vrep.simx_opmode_oneshot)

            female_joint_angle = amp_factor * math.sin((speed_factor * t) + (i * math.pi)
                                                       + reverse_factor * math.pi / 2)
            err_val = vrep.simxSetJointTargetPosition(clientID, female_joints[i],
                                                      math.radians(female_joint_angle), vrep.simx_opmode_oneshot)

    else:    # If particular module number specified.
        try:
            module_num = int(cmd[0])
        except:
            print "ValueError in retrieving module_num from command!"
            return

        if 1 > module_num > total_num_modules:  # If less than 1 and greater than number_of_modules
            print "Invalid module number!"
            return

        male_joint_angle = amp_factor * math.sin((speed_factor * t) + (module_num * math.pi)
                                                 + (1 - reverse_factor) * math.pi / 2)
        err_val = vrep.simxSetJointTargetPosition(clientID, male_joints[module_num],
                                                  math.radians(male_joint_angle), vrep.simx_opmode_oneshot)

        female_joint_angle = amp_factor * math.sin((speed_factor * t) + (module_num * math.pi)
                                                   + reverse_factor * math.pi / 2)
        err_val = vrep.simxSetJointTargetPosition(clientID, female_joints[module_num],
                                                  math.radians(female_joint_angle), vrep.simx_opmode_oneshot)

# --------------------------------------------------------------------------------------------------------- #

# Program starts running here #
if clientID != -1:
    print ('Connected to remote API server')

    # Get handles for all joints
    for i in range(1, total_num_modules + 1):
        err_val, male_joint_handle = vrep.simxGetObjectHandle(clientID, str('Male_joint' + str(i)),
                                                           vrep.simx_opmode_blocking)
        err_val, female_joint_handle = vrep.simxGetObjectHandle(clientID, str('Female_joint' + str(i)),
                                                             vrep.simx_opmode_blocking)

        male_joints.append(male_joint_handle)    # Store obtained joints.
        female_joints.append(female_joint_handle)


    while True:   # Run until program stopped.
        raw_command = listen_serial()   # Check Serial

        if raw_command != -1:   # If something received
            command = raw_command

        if command != "":    # If not empty,

            if command[1] == 's':      # Set angle to joint
                cmd_set_angle(command)

            elif command[1] == 'r':    # Run sinusoidal motion
                run_sinusoidal(command)

            elif command[1] == 'e':    # Escape/End all movement
                command = ""
else:
    print ('Failed connecting to remote API server')


