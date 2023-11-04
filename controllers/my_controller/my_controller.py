"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor, DistanceSensor, GPS, Camera, Compass, Emitter, Receiver
import math, random, time, struct


# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


# Define Constants
DEBUG = False
READ_SENSOR_DEBUG = False
LANE_FOLLOW_DEBUG = False
OBSTACLE_DEBUG = False

PI = math.pi

X = 0 #index for vector coordinates
Y = 1
Z = 2

NEG_Z = 0 #spawn position on map
POS_Z = 1
POS_X = 2

LEFT = 0 #intersection turning action
STRAIGHT = 1
RIGHT = 2

RED = 1 #LED colours
BLUE = 2

#opertation modes
LANE_FOLLOW_MODE = 1
OBSTACLE_AVOIDANCE_MODE = 2
CROSS_INTERSECTION_MODE = 3
COLLISION_AVOID_MODE = 4
THRU_INTER_MODE = 5

roadWidth = 0.5
intersectionBoundary = 0.6*roadWidth #square boundary where we are considered inside intersection

maxMotorSpeed = 22 #motor maximum is 22
maxAcceleration = 50 #for motor
maxMeasuredSpeed = 0.58
brakingAcceleration = 50
maxPivotAcceleration = -1 #for steering pivot
maxSteer = 1.14 #1.57 #max steering angle for pivot, in radians
curbDist = 10.0 #desired distance from wall

obsDist1 = 0 #initialisation of the obstacle distance; used in velocity calculations
minObstacleDist = 30 #distance from which an object is considered to be an obstacle
errVel = 0.08 #acceptable velocity error (ex: 0.02 = 2%)
slowObstacle = 0.5 #% of reduction of speed when an obstacle is detected (ex 0.75=75%)
wheelRadius = 1 #Radius of the wheels

class vehicle_variables:
    def __init__(self):
        #Sensor Readings
        self.laneDist = None #distance to right curb
        self.obstacleDist = None #distance to front obstacle, if any
        self.pos = None #coords of vehicle
        self.speed = None #velocity magnitude
        self.compassVector = None #compass reading, vectors wrt north (+X-axis)
        self.msg = None #this for now, Need  for receiver's messages
        
        #Vehicle Info / Preset Decisions / actuator outputs
        self.action = None #what car will do at intersection
        self.spawnPos = None #where car was spawned [0,1 or 2 --> check constants]
        self.desiredAngle = None #Angle that car drives towards, will change after crossing intersection
        self.desiredSpeed = maxMotorSpeed #current desired speed for driving, max speed by default
        self.obstacleSensorRange = None #max range given to sensor
        self.bearing = None
        self.movingSpeed = None
        self.motorSpeed = maxMotorSpeed
        self.ledColor = None
        self.ledOnOff = False
        
        #Priority Info
        self.timeToIntersection = None #Time to intersection
        self.distanceToIntersection = None #Distance to intersection
        self.firstToIntersection = True #Is the car going to reach the intersection first?
 
# End of constants 


# INITILIZATION
car = vehicle_variables() #create vehicle object (easier to use with read_sensors function I think)


# FUNCTIONS
def initialize():
    # ENABLE ROBOT SENSORS AND ACTUATORS
    global motor, pivot, lane_sensor, obstacle_sensor, gps, compass, emitter, receiver, camera, led

    # Actuators
    motor = robot.getDevice("motor")
    motor.setPosition(float('inf')) #set velocity mode
    motor.setAcceleration(maxAcceleration)
    motor.setVelocity(0)

    pivot = robot.getDevice("pivot")
    pivot.setPosition(0)
    pivot.setAcceleration(maxPivotAcceleration)

    emitter = robot.getDevice("emitter") #radio tranmission

    led = robot.getDevice("led")

    # Sensors
    lane_sensor = robot.getDevice("lane_sensor") #sensor of lane follow
    lane_sensor.enable(timestep)

    obstacle_sensor = robot.getDevice("obstacle_sensor") #front obstacles
    obstacle_sensor.enable(timestep)
    car.obstacleSensorRange = obstacle_sensor.getMaxValue()

    gps = robot.getDevice("gps")
    gps.enable(timestep)

    compass = robot.getDevice("compass") #for angle measurment
    compass.enable(timestep)

    receiver  = robot.getDevice("receiver") #radio reception
    receiver.enable(timestep)

    camera = robot.getDevice("camera") # "dashcam"
    camera.enable(timestep*3)

    # END OF INITIALIZATION

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def determine_info():
    #Determine vehicle info
        #spawn location, set driving direction, choose what to do at intersection
    if (car.pos[X] > roadWidth/2):
        car.spawnPos = POS_X
        car.desiredAngle = PI #Pi wrt to pos_x
        if (random.randint(0,1)): #random choose between turning left or right
            car.action = LEFT
        else:
            car.action = RIGHT
    elif (car.pos[Z] > roadWidth/2):
        car.spawnPos = POS_Z
        car.desiredAngle = 3*PI/2 #Pi/2 wrt to pos_x
        car.action = random.randint(1,2) #straight (1) or right (2)
    elif (car.pos[Z] < roadWidth/2):
        car.spawnPos = NEG_Z
        car.desiredAngle = PI/2 #3*Pi/2 wrt to pos_x
        car.action = random.randint(0,1) #left (0) or straight(1)
    else:
        print("ERROR")


    if (DEBUG):
        if (car.action == LEFT):
            action = "turning left"
        elif (car.action == RIGHT):
            action = "turning right"
        elif (car.action == STRAIGHT):
            action = "going straight"

        if (car.spawnPos == POS_X):
            spawn = "+X"
        elif (car.spawnPos == POS_Z):
            spawn = "+Z"
        elif (car.spawnPos == NEG_Z):
            spawn = "-Z"
        print("Car spawned at: " + spawn + ", driving angle (deg): " + str(round(math.degrees(car.desiredAngle),0)) + ", then "  + action)

def read_sensors():  
    car.laneDist = lane_sensor.getValue() #Side sensor
    car.obstacleDist = obstacle_sensor.getValue() #Front sensor
    car.pos = gps.getValues() #Robot Coordinates
    car.gpsSpeed = gps.getSpeed() #Velocity magnitude

    #update compass and calculate bearing
    car.compassVector = compass.getValues() #compass output
    
    car.bearing = math.atan2(car.compassVector[0], car.compassVector[2])
    while (car.bearing < 0.0):
        car.bearing += 2*PI
    
    # Update Car Time
    if car.spawnPos == POS_X:
        car.distanceToIntersection = abs(car.pos[X])
    elif car.spawnPos == POS_Z or car.spawnPos == NEG_Z:
        car.distanceToIntersection = abs(car.pos[Z])
    
    car.movingSpeed = translate(car.motorSpeed, 0, maxMotorSpeed, 0, maxMeasuredSpeed)
    try:
        car.timeToIntersection = float(car.distanceToIntersection/car.movingSpeed)
    except:
        pass
        
    if (DEBUG and READ_SENSOR_DEBUG):
        print("Distance to curb: "+ str(car.laneDist))
        print("Front obstacle distance: "+ str(car.obstacleDist))
        print("Coordinates: "+ str(car.pos))
        print("Speed: " + str(car.gpsSpeed))
        print("Compass reading: " + str(car.compassVector))
        print("-----")

def send_data():
    message = struct.pack("fiif", car.timeToIntersection, car.spawnPos, car.action, car.distanceToIntersection)
    emitter.send(message)

def receive_data():
    otherCars = []
    while receiver.getQueueLength() > 0:
        data = struct.unpack("fiif", receiver.getData())
        otherCars.append(data)
        receiver.nextPacket()
    return otherCars

def determine_order():
    car.firstToIntersection = True
    for otherCar in otherCars:
        if car.distanceToIntersection > otherCar[3]: #compare car to other car times and set false if larger
            car.firstToIntersection = False

def mode_selection():
    #Check if car is in intersection
    #if vehicle x and z coordinates are in the boundary preset for intersection area
    if(DEBUG and OBSTACLE_DEBUG):
        print("The obstacle distance is: "+str(car.obstacleDist))   
         
    if (abs(car.pos[X]) <= intersectionBoundary and abs(car.pos[Z]) <= intersectionBoundary):
        if (DEBUG):
            print("CROSS INTERSECTION MODE")
        return(CROSS_INTERSECTION_MODE)

    #ELSE IF: car detects turn will collide within a car's turn at same time
    elif (detect_collision() == True): #if car turn conflicts with another car and is within n seconds behind
        if (DEBUG):
            print("COLLISION AVOIDANCE MODE")
        return(COLLISION_AVOID_MODE)

    #ELSE IF: isInIntersection = false and isObstacle = false, lane_following mode
    elif (car.obstacleDist < minObstacleDist): #verify if a solid object in front is too close
        if (DEBUG):
            print("OBSTACLE AVOIDANCE MODE")
        return(OBSTACLE_AVOIDANCE_MODE)
        
    else:
        #if (DEBUG):
        #    print("LANE FOLLOWING MODE")
        #    car.desiredSpeed = maxMotorSpeed
        return(LANE_FOLLOW_MODE)
    #ELSE IF: not in intersection but obstace detected,  obstacle avoid mode
                
def lane_follow(desiredDistance, desiredAngle, speed, acceleration):
    #Desired distance from right wall, desired angle to travel towards, desired motor speed

    motor.setAcceleration(acceleration)
    motor.setVelocity(speed)

    if (desiredAngle == 0 and car.bearing > PI/2):
        desiredAngle = 2*PI
    elif (desiredAngle == 2*PI and car.bearing <= PI/2):
        desiredAngle = 0
            
    angleError = desiredAngle - car.bearing #car orientation error

    errorD = car.laneDist - desiredDistance #lane position error

    output = -0.012*errorD - 0.7*angleError #angle output to pivot, PD type response
    
    #correct output to respect maximum possible angles
    output = max(-maxSteer, output) #if angle smaller than smallest angle, take smallest angle
    output = min(maxSteer, output) #if angle is larger than max steer, take maxSteer

    pivot.setPosition(output) #set pivot angle

    if (DEBUG and LANE_FOLLOW_DEBUG):
        print("Angle wrt North (deg): " + str(math.degrees(rad)))
        print("Lane position error: " + str(errorD))
        print("Angle error (deg): " + str(math.degrees(car.desiredAngle-rad)))
    
def avoid_obstacle(speed,posObject1):
    
    
    
    #Determine the obstacle's velocity to react properly
    
    posObject2 = car.obstacleDist #Second measurement

    waitTime = timestep/1000
    
    relVelocity = (posObject1 - posObject2)/waitTime/100 #Velocity of the object relative to the car (positive when toward the car)
    carVelocity=car.gpsSpeed
    
    motor.setVelocity(slowObstacle*speed) #slow down the car immediately
    
    if(DEBUG and OBSTACLE_DEBUG):
         print("The obstacle distance relative to the car is "+str(car.obstacleDist))
         print("The relative velocity of the obstacle is "+str(relVelocity))
         print("The velocity of the car is "+str(carVelocity))
         print("Absolute velocity of object is " + str(carVelocity-relVelocity))
         print("The first distance measurement is "+str(posObject1))
         print("The second distance measurement is "+str(posObject2))
         print("The wait time is "+str(waitTime))
         print("-------------")
         
    minVelCrit=(1-errVel)*carVelocity
    maxVelCrit=(1+errVel)*carVelocity
    
    if (relVelocity <= maxVelCrit and relVelocity >= minVelCrit): #Condition to be considered an immobile obstacle
            motor.setAcceleration(brakingAcceleration)
            newTargetSpeed = 0
            if(DEBUG and OBSTACLE_DEBUG):
                print("Immobile obstacle")

    elif (relVelocity < 0): #object is going away from the car
            newTargetSpeed = car.motorSpeed #return to max speed
            if(DEBUG and OBSTACLE_DEBUG):
                print("Obstacle going away")

    elif (relVelocity > maxVelCrit): #obstacle is moving toward the car
            newTargetSpeed = 0
            if(DEBUG and OBSTACLE_DEBUG):
                print("Obstacle going toward")

    elif (relVelocity >= 0 and relVelocity < minVelCrit): #obstacle is moving away from the car, but at lower velocity
           newTargetSpeed = (carVelocity-relVelocity)/(PI*2*wheelRadius) #use front car's speed 
           lane_follow(curbDist, car.desiredAngle, newTargetSpeed, 4*maxAcceleration) #to ensure car stays in lane
           if(DEBUG and OBSTACLE_DEBUG):
                print("Slower Obstacle")
    else:
            print ("Obstacle Avoidance Error")
    motor.setVelocity(newTargetSpeed)
 
    return (newTargetSpeed)

def detect_collision():

    longerGapNeeded = False
    #determine if your turn conflicts with another car
    if car.firstToIntersection == False:
        conflict = False
        for otherCar in otherCars:
            if car.spawnPos == POS_Z and otherCar[1] == NEG_Z:
                if otherCar[2] == LEFT:
                    conflict = True
                    longerGapNeeded = True

                    #print(car.firstToIntersection)
                    #print(car.motorSpeed)
                    #print(car.timeToIntersection)
                    #print(otherCar[0])
            
            elif car.spawnPos == POS_Z and otherCar[1] == POS_X:
                if car.action == STRAIGHT:
                    conflict = True
                            
            elif car.spawnPos == POS_X and otherCar[1] == POS_Z:
                if otherCar[2] == STRAIGHT:
                    conflict = True

            elif car.spawnPos == POS_X and otherCar[1] == NEG_Z:
                if car.action != RIGHT:
                    conflict = True
                    
            elif car.spawnPos == NEG_Z and otherCar[1] == POS_Z:
                if car.action  == LEFT:
                    conflict = True
                    longerGapNeeded = True

            elif car.spawnPos == NEG_Z and otherCar[1] == POS_X:
                if otherCar[2] == LEFT:
                    conflict = True 

        #if conflict, determine if car is within n seconds to next car
        gapTime = 0.5
        if longerGapNeeded:
            gapTime = 1.5
        if conflict:
            possibleSpeeds = []
            for otherCar in otherCars:
                timeDifference = car.timeToIntersection - otherCar[0]
                #print(timeDifference)
                if timeDifference > 0 and timeDifference < gapTime:
                    possibleSpeeds.append(car.distanceToIntersection/(otherCar[0]+gapTime))
            #print("possible speeds: " + str(possibleSpeeds))
            if len(possibleSpeeds) > 0:
                car.motorSpeed = int(translate(min(possibleSpeeds), 0, maxMeasuredSpeed, 0, maxMotorSpeed))
                #print(car.motorSpeed)
                car.ledColor = RED
                car.ledOnOff = True            
    else:
        car.motorSpeed = maxMotorSpeed
        car.ledOnOff = False

def turn(init):
    motor.setVelocity(maxMotorSpeed)
    if car.action == LEFT:
        pivot.setPosition(15*PI/180) #degree to rad
        if init:
            car.desiredAngle -= PI/2 #Pi/2 wrt to pos_x
            if car.desiredAngle < 0:
                car.desiredAngle += 2*PI
            return True

    elif car.action ==RIGHT:
        pivot.setPosition(-35*PI/180)
        if init:
             car.desiredAngle += PI/2 #Pi/2 wrt to pos_x
             if car.desiredAngle >= 2*PI:
                 car.desiredAngle -= 2*PI
             return True

   
# END OF FUNCTIONS


# vvv Main function vvv
initialize()
robot.step(timestep)

read_sensors()
determine_info()
#targetSpeed = maxSpeed

prev_mode = None
thru_inter = False

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
   
    read_sensors()

    #print("-----")
    #print("Spawn Pos: " + str(car.spawnPos))
    #print("first: " + str(car.firstToIntersection))
    #print("thru: " + str(thru_inter))
    #print("motor speed: " + str(car.motorSpeed))
    
    if not(thru_inter):
        send_data()
        otherCars = receive_data()       
        determine_order()
    
    mode = mode_selection()
    
    mode_chg_init = (mode != prev_mode)
    prev_mode = mode
   
    if (mode == LANE_FOLLOW_MODE):
        lane_follow(curbDist, car.desiredAngle, car.motorSpeed, maxAcceleration) # regular lane following mode
    elif (mode == COLLISION_AVOID_MODE):
        lane_follow(curbDist, car.desiredAngle, car.motorSpeed, maxAcceleration)
        led.set(1)
    elif (mode == OBSTACLE_AVOIDANCE_MODE):
        car.desiredSpeed = avoid_obstacle(car.desiredSpeed,obsDist1)
        car.ledColor = BLUE
        car.ledOnOff = True
    elif (mode == CROSS_INTERSECTION_MODE):
        if mode_chg_init:
            thru_inter = True
        turn(mode_chg_init)

    if (mode != COLLISION_AVOID_MODE):
        led.set(0)

    if (car.ledOnOff):
        led.set(car.ledColor)
    else:
        led.set(0)

     
    obsDist1 = car.obstacleDist #Get the distance of the obstacle to be used in the next loop

    pass