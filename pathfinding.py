#MOTOR LIBRARIES-------------------------------------------------------------------------------------------------

# Import external libraries
import rcpy
import rcpy.motor as motor
import time                                     # only necessary if running this program as a loop
import numpy as np                              # for clip function

motor_l = 1 	                                # Left Motor (ch1)
motor_r = 2 	                                # Right Motor (ch2)\


#LIDAR LIBRARIES-------------------------------------------------------------------------------------------------

# Import external libraries
import numpy as np
# from numpy import exp, abs, angle
import time

# Import internal programs
# import L1_lidar as lidar

np.set_printoptions(precision=3)                    # after math operations, don't print long values


#MAGNEOMETRIC LIBRARIES---------------------------------------------------------------------------------------

import L1_mpu as mpu   

#DISPLACEMENT LIBRARIES--------------------------------------------------------------------------------------
import L1_encoder as enc  



#ARM LIBRARIES-----------------------------------------------------------------------------------------------

import L1_Arm as arm

#MOTOR SETUP-------------------------------------------------------------------------------------------------


# NOTE: THERE ARE 4 OUTPUTS.  3 & 4 ACCESSIBLE THROUGH diode & accy functions

rcpy.set_state(rcpy.RUNNING)                    # initialize the rcpy library


# define functions to command motors, effectively controlling PWM
def MotorL(speed):                              # takes argument in range [-1,1]
    motor.set(motor_l, speed)


def MotorR(speed):                              # takes argument in range [-1,1]
    motor.set(motor_r, speed)


def diode(state, channel):                      # takes argument in range [0,1]
    np.clip(state, 0, 1)                        # limit the output, disallow negative voltages
    motor.set(channel, state)


def accy(state, channel):                       # takes argument in range [-1,1]
    motor.set(channel, state)
    
    
# #LIDAR SETUP-------------------------------------------------------------------------------------------------
    
# def getValid(scan):                                 # remove the rows which have invalid distances
#     dist = scan[:, 0]                               # store just first column
#     angles = scan[:, 1]                             # store just 2nd column
#     valid = np.where(dist > 0.016)                  # find values 16mm
#     myNums = dist[valid]                            # get valid distances
#     myAng = angles[valid]                           # get corresponding valid angles
#     output = np.vstack((myNums, myAng))             # recombine columns
#     n = output.T                                    # transpose the matrix
#     return n


# def nearest(scan):                                  # find the nearest point in the scan
#     dist = scan[:, 0]                               # store just first column
#     column_mins = np.argmin(dist, axis=0)           # get index of min values along 0th axis (columns)
#     row_index = column_mins                         # index of the smallest distance
#     vec = scan[row_index, :]                        # return the distance and angle of the nearest object in scan
#     return vec                                      # contains [r, alpha]


# def polar2cart(r, alpha):                           # convert an individual vector to cartesian coordinates (in the robot frame)
#     alpha = np.radians(alpha)                       # alpha*(np.pi/180) # convert to radians
#     x = r * np.cos(alpha)                           # get x
#     y = r * np.sin(alpha)                           # get y
#     cart = np.round(np.array([x, y]), 3)            # vectorize and round
#     return cart


# def rotate(vec, theta):                             # describe a vector in global coordinates by rotating from body-fixed frame
#     c, s = np.cos(theta), np.sin(theta)             # define cosines & sines
#     R = np.array(((c, -s), (s, c)))                 # generate a rotation matrix
#     vecGlobal = np.matmul(R, vec)                   # multiply the two matrices
#     return vecGlobal


# def sumVec(vec, loc):                               # add two vectors. (origin to robot, robot to obstacle)
#     mySum = vec + loc                               # element-wise addition takes place
#     return mySum                                    # return [x,y]


# def getNearest():                                   # combine multiple functions into one.  Call to get nearest obstacle.
#     scan = lidar.polarScan()                        # get a reading in meters and degrees
#     valids = getValid(scan)                         # remove the bad readings
#     vec = nearest(valids)                           # find the nearest
#     return vec                                      # pass the closest valid vector [m, deg]
    
    
    
#ARM SETUP------------------------------------------------------------------------------------------------------------------------------
import time, math
import getopt, sys
import rcpy  # This automatically initizalizes the robotics cape
import rcpy.servo as servo
import rcpy.clock as clock	# For PWM period for servos

# INITIALIZE DEFAULT VARS
duty = 1.5		# Duty cycle (-1.5,1.5 is the typical range)
period = 0.02 	# recommended servo period: 20ms (this is the interval of commands)
ch1 = 1			# select channel (1 thru 8 are available)
ch2 = 2			# selection of 0 performs output on all channels
# ch3 = 3
# ch4 = 4
# ch5 = 5
ch6 = 6

rcpy.set_state(rcpy.RUNNING) # set state to rcpy.RUNNING
srvo1 = servo.Servo(ch1)	# Create servo object
srvo2 = servo.Servo(ch2)
# srvo3 = servo.Servo(ch3)
# srvo4 = servo.Servo(ch4)
# srvo5 = servo.Servo(ch5)
srvo6 = servo.Servo(ch6)
clck1 = clock.Clock(srvo1, period)	# Set PWM period for servos
clck2 = clock.Clock(srvo2, period)
# clck3 = clock.Clock(srvo3, period)
# clck4 = clock.Clock(srvo4, period)
# clck5 = clock.Clock(srvo5, period)
clck6 = clock.Clock(srvo6, period)

servo.enable()		# Enables 6v rail on beaglebone blue
clck1.start()		# Starts PWM
clck2.start()
# clck3.start()
# clck4.start()
# clck5.start()
clck6.start()

def move1(angle):
	srvo1.set(angle)
	
def move2(angle):
	srvo2.set(angle)
	
# def move3(angle):
# 	srvo3.set(angle)

# def move4(angle):
# 	srvo4.set(angle)

def move5(angle):
	srvo5.set(angle)

def move6(angle):
	srvo6.set(angle)


# #MAGNEOMETRIC SETUP --------------------------------------------------------------------------------------------------------------------
# xRange = np.array([-43, 58])                         # range must be updated for your device
# yRange = np.array([-54, 19])                        # range must be updated for your device


# def getXY():                                        # this function returns an average of several magnetometer readings for x and y
#     data = np.take(mpu.getMag(), [0, 1])            # take only the first two elements of the returned array
#     for i in range(10):                             # iterate 10 times (i will start at zero)
#         newData = np.take(mpu.getMag(), [0, 1])     # call getMag and take the first two elements
#         data = np.vstack((data, newData))           # vertically stack the new data array at bottom of existing data
#         time.sleep(0.002)                           # delay 5 ms
#     data_av = np.average(data, axis=0)              # take an average of the x's and y's to form new array
#     data_av = np.round(data_av, 3)                  # round the data
#     return(data_av)


# def scale(axes):                                    # convert raw values to range of [-1 1]

#     # re-scale the returned values to a ratio of the value to it's maximum value (0 to 1)
#     xScaled = (axes[0] - xRange[0]) / (xRange[1]-xRange[0])
#     yScaled = (axes[1] - yRange[0]) / (yRange[1]-yRange[0])

#     # re-center the values about zero, and expand the range to +/- 1
#     xCentered = (xScaled - 0.5) * 2
#     yCentered = (yScaled - 0.5) * 2
#     axes = np.array([xCentered, yCentered])
#     axes = np.round(axes, 2)
#     return(axes)                                    # returns scaled, centered axes in range [-1 1]


# def getHeading(myAxes):                             # convert scaled values to a heading
#     h = np.arctan2(myAxes[0], myAxes[1])            # atan2 uses all four quadrants to return [-180, 180] range
#     return(h)

#DISPLACEMENT SETUP----------------------------------------------------------------------------------------------------------
# define kinematics
R = 0.041                                   # radius in meters
L = 0.201                                   # half of wheelbase meters
res = (360/2**14)                           # resolution of the encoders
roll = int(360/res)                         # variable for rollover logic
gap = 0.5 * roll                            # degress specified as limit for rollover

A = np.array([[R/2, R/2], [-R/(2*L), R/(2*L)]])     # This matrix relates [PDL, PDR] to [XD,TD]
wait = 0.02                                 # wait time between encoder measurements (s)

def getTravel(deg0, deg1):                  # calculate the delta on Left wheel
    trav = deg1 - deg0                      # reset the travel reading
    if((-trav) >= gap):                     # if movement is large (has rollover)
        trav = (deg1 - deg0 + roll)         # forward rollover
    if(trav >= gap):
        trav = (deg1 - deg0 - roll)         # reverse rollover
    return(trav)

def getChassis(disp):                       # this function returns the chassis displacement
    B = disp                                # this array should store phi displacements (in radians)
    C = np.matmul(A, B)                     # perform matrix multiplication
    C = np.round(C, decimals=3)             # round the matrix
    return(C)                               # returns a matrix containing [dx, dTheta]

encoders = enc.read()                       # grabs the current encoder readings before beginning

# initialize variables at zero
x = 0                                       # x
t = 0                                       # theta
encL1 = 0
encR1 = 0


#SCRIPT SETUP----------------------------------------------------------------------------------------------------------------------------

#-107 = window
#-114 = diagonal
#-133 = island

print("hi")


y=0

# while(x < .7):
    
MotorL(1)
MotorR(1)
time.sleep(4)
MotorL(0)
MotorR(0)

    # encL0 = encL1                           # transfer previous reading.
    # encR0 = encR1                           # transfer previous reading.
    # encoders = enc.read()                   # grabs the current encoder readings, raw
    # encL1 = round(encoders[0], 1)           # reading, raw.
    # encR1 = round(encoders[1], 1)           # reading, raw.

    # # ---- movement calculations
    # travL = getTravel(encL0, encL1) * res   # grabs travel of left wheel, degrees
    # travL = -1 * travL                      # this wheel is inverted from the right side
    # travR = getTravel(encR0, encR1) * res   # grabs travel of right wheel, degrees

    # # build an array of wheel travels in rad/s
    # travs = np.array([travL, travR])        # store wheels travel in degrees
    # travs = travs * 0.5                     # pulley ratio = 0.5 wheel turns per pulley turn
    # travs = travs * 3.14 / 180              # convert degrees to radians
    # travs = np.round(travs, decimals=3)     # round the array
    
    # chass = getChassis(travs)               # convert the wheel travels to chassis travel
    # x = x + chass[0]                        # add the latest advancement(m) to the total
    # t = t + chass[1]
    # print(x)
    # #time.sleep(.1)
    # y=1
# print(x)

# MotorL(0)
# MotorR(0)
# arm.arm()



# while(x < 1.4):
#     MotorL(-1)
#     MotorR(-1)
#     encL0 = encL1                           # transfer previous reading.
#     encR0 = encR1                           # transfer previous reading.
#     encoders = enc.read()                   # grabs the current encoder readings, raw
#     encL1 = round(encoders[0], 1)           # reading, raw.
#     encR1 = round(encoders[1], 1)           # reading, raw.

#     # ---- movement calculations
#     travL = getTravel(encL0, encL1) * res   # grabs travel of left wheel, degrees
#     travL = -1 * travL                      # this wheel is inverted from the right side
#     travR = getTravel(encR0, encR1) * res   # grabs travel of right wheel, degrees

#     # build an array of wheel travels in rad/s
#     travs = np.array([travL, travR])        # store wheels travel in degrees
#     travs = travs * 0.5                     # pulley ratio = 0.5 wheel turns per pulley turn
#     travs = travs * 3.14 / 180              # convert degrees to radians
#     travs = np.round(travs, decimals=3)     # round the array
    
#     chass = getChassis(travs)               # convert the wheel travels to chassis travel
#     x = x + chass[0]                        # add the latest advancement(m) to the total
#     t = t + chass[1]
#     print(x)
#     time.sleep(.1)
    
# print("finished")
# MotorL(0)
# MotorR(0)
        
    # MotorL(0.6)                         # gentle speed for testing program. 0.3 PWM may not spin the wheels.
    # MotorR(0.6)
    # myVector = getNearest()                                 # call the function which utilizes several functions in this program
    # if myVector[0] < .25 and (myVector[1] < 0 and myVector[1] > -30):
    #     print("Turning Right")
    #     MotorL(0.6)                         
    #     MotorR(-0.6)
    #     time.sleep(2)
    #     MotorL(0.6)                         # gentle speed for testing program. 0.3 PWM may not spin the wheels.
    #     MotorR(0.6)
    #     time.sleep(2.5)
    #     MotorL(-0.6)
    #     MotorR(0.6)
    #     time.sleep(2)
    #     MotorL(0.6)                         
    #     MotorR(0.6)
    #     time.sleep(6)
    #     MotorL(-0.6)
    #     MotorR(0.6)
    #     time.sleep(2)
    #     MotorL(0.6)                         
    #     MotorR(0.6)
    #     time.sleep(2.5)
    #     MotorL(0.6)                         
    #     MotorR(-0.6)
    #     time.sleep(2)
        
    # elif myVector[0] < .25 and (myVector[1] > 0 and myVector[1] < 30):
    #     print("Turning Left")
    #     MotorL(-0.6)                         
    #     MotorR(0.6)
    #     time.sleep(2)
    #     MotorL(0.6)                         # gentle speed for testing program. 0.3 PWM may not spin the wheels.
    #     MotorR(0.6)
    #     time.sleep(2.5)
    #     MotorL(0.6)
    #     MotorR(-0.6)
    #     time.sleep(2)
    #     MotorL(0.6)                         # gentle speed for testing program. 0.3 PWM may not spin the wheels.
    #     MotorR(0.6)
    #     time.sleep(6)
    #     MotorL(0.6)
    #     MotorR(-0.6)
    #     time.sleep(2)
    #     MotorL(0.6)                         
    #     MotorR(0.6)
    #     time.sleep(2.5)
    #     MotorL(-0.6)                         
    #     MotorR(0.6)
    #     time.sleep(2)
    # else:
    #     print("Moving Forward")
    
