#A two link Robotic arm,
# the connecting point between each arm is called as the Elbow point.
# the end point of the second link (hand) is the end effector.

#imports
import matplotlib
matplotlib.use('TkAgg')
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animate
import numpy as np
#Initial values
current_angle1=0
current_angle2=0

#user defined functions
def  forward_kinematics(l1,l2,deg_1,deg_2):
    """Provides the x and y coordinates for corresponding elbow and hand angles """
    x1 = l1 * math.cos(math.radians(deg_1)) # x coordinate for elbow
    y1 = l1 * math.sin(math.radians(deg_1)) # y coordinate for the elbow
    x2 = x1 + (l2 * math.cos(math.radians(deg_1) + math.radians(deg_2))) # x coordinate for the hand
    y2 = y1 + (l2 * math.sin(math.radians(deg_1)+ math.radians(deg_2))) # y coordinate for the hand
    return (round(x1,2),round(y1,2)),(round(x2,2),round(y2,2)) # tuple form = (x elbow, y elbow),(x hand, y hand)

def distance_to_target(x, y):
    """Provides hypotenuse/shortest distance from origin to the target"""
    return math.sqrt((x**2)+(y**2)) #calculating hypotenuse: shortest distance.

def is_reachable(l1, l2, r):
    """Provides Boolean values for reachability."""
    max_reach= l1 + l2
    min_reach = abs(l1 - l2)
    if r<=max_reach and r>=min_reach:
        return True
    else:
        return False

def elbow_angle(l1, l2 ,r ):
    """For provided link lengths and distance to target(r), gives the angle at the elbow:theta1"""
    cos_theta2_unclamped = ((l1**2) + (l2**2) - (r**2)) / (2*l1*l2) #Theta2 is the angle that the link 2 makes with the line connecting origin and elbow.
    cos_theta2 = max(-1, min (1,cos_theta2_unclamped)) #Clamping for constraining into feasible cosine values.
    return (180 - math.degrees(math.acos(cos_theta2))) #provides the elbow angle: interior angle between both the links.

def base_angle(l1,l2,x,y,angle2):
    """Provides the base angle 1: the angle of link 1 with the horizontal (theta_1)."""
    cos_angle2 = math.cos(math.radians(angle2))
    sin_angle2 = math.sin(math.radians(angle2))
    A= l2 * cos_angle2
    B= l2* sin_angle2
    numerator = (y* (l1 + A) - (x * B))
    denominator= x * (l1+A)+(y*B)
    return math.degrees(math.atan2(numerator,denominator))

def inverse_kinematics(l1,l2,x,y):
    """Provides angle1 and angle2 for a given X,Y coordinate."""
    r= distance_to_target(x,y) #distance to target from the origin.
    if not is_reachable(l1,l2,r): #is test for reachability.
        return None #to be processed if the provided coordinate is out of reach for the robot.
    else:
        angle2= elbow_angle(l1,l2,r)
        angle1= base_angle(l1,l2,x,y,angle2)
        return angle1,angle2 #returning in form - tuple

def interpolation(start_angle,end_angle,steps=40):
    """Provides a list of  'steps' number of  intermediate angles between start and end angles-
     -required for smooth animations"""
    return np.linspace(start_angle,end_angle,steps)

#Setting the canvas.
screen, axes = plt.subplots() # subplots.() provides two things, a canvas and an axes system on the canvas. unpacking the canvas to draw in into 'screen' and unpacking the axes into ax(the actual drawing area in the screen), a single screen can contain multiple axes
axes.set_xlim(-2.2, 2.2) # limits the x axis scale.
axes.set_ylim(-2.2, 2.2) #limits the y axis scale.
axes.grid(True) #provides a grid on the graph.
axes.set_aspect('equal') #adjusting scale to avoid screwedness.
line, = axes.plot([], [], 'o-') #stores the object line2D from plot() function applied on to axes. , accepts only one item.

#intial pose values
angle1_path = interpolation(current_angle1, current_angle1) #provides intermediate values
angle2_path = interpolation(current_angle2, current_angle2) #provides intermediate values

def update(frame_number):
    elbow, end_effector = forward_kinematics(1,1,angle1_path[frame_number],angle2_path[frame_number]) #Stores (x1,y1),(x2,y2) coordinates for elbow and end effector in corresponding variables.
    xs = [0, elbow[0], end_effector[0]] #stores x values of origin, elbow and end effector
    ys = [0, elbow[1], end_effector[1]] #stores y values of origin, elbow and end effector
    line.set_data(xs, ys) #creates line2D for provided coordinates
    if frame_number == len(angle1_path) - 1:
        global current_angle1, current_angle2
        current_angle1 = angle1_path[frame_number]
        current_angle2 = angle2_path[frame_number]
    return line,

ani=animate.FuncAnimation(screen,update,frames=len(angle1_path),interval=50,blit=True,repeat=False) #animates the first pose, initial position with both angles = 0 degrees.

def on_click(event):
    print(f"Clicked Coordinates: X:{event.xdata}, Y:{event.ydata}") # gets the coordinates of the clicked point.
    x,y=event.xdata, event.ydata #Stores the clicked location coordinates into variables.
    result=inverse_kinematics(1,1,x,y) #stores the corresponding angles of the clicked coordinates into a tuple.
    if result is None:
        print("Target unreachable")
    else:
        angle1, angle2 = result #unpacking the tuple.
        global angle1_path,angle2_path
        angle1_path = interpolation(current_angle1, angle1)
        angle2_path = interpolation(current_angle2, angle2)
        global ani
        ani = animate.FuncAnimation(screen, update, frames=len(angle1_path), interval=50, blit=True, repeat=False)

screen.canvas.mpl_connect('button_press_event', on_click)
plt.show()









