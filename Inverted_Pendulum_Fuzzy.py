from Box2D import (b2World, b2Vec2, b2WeldJointDef, b2EdgeShape, b2PolygonShape, b2CircleShape, b2FixtureDef, b2RevoluteJointDef)
import numpy as np
import pygame
import skfuzzy as fuzz
from skfuzzy import control as crtl
from matplotlib import pyplot as plt


# Initializing pygame
pygame.init()
Screen_Width = 800
Screen_Height = 600
screen = pygame.display.set_mode((Screen_Width, Screen_Height))

# Box2D to pygame logic
PixPerMetre = 20.0
Time_Step = 1.0/60
def to_pygame(pos):
    return int(pos[0]*PixPerMetre), int(Screen_Height- pos[1]*PixPerMetre)


# Defining world in Box2D and the two bodies
world = b2World(gravity=(0,-10))
ground = world.CreateStaticBody(
    position =(0,0),
    shapes=b2EdgeShape(vertices=[(-20,2),(200,2)])
    )

# Creating the planck upon which all the bodies rest on
planck = world.CreateDynamicBody(
    position =(20,7),
    shapes=b2PolygonShape(box=(10,1)))


# Creating the two wheels and their fixtures
circle_body_1 = world.CreateDynamicBody(
    position =(12,4)
    )


circle_fixture_1=circle_body_1.CreateCircleFixture(shape=b2CircleShape(radius=2),
    density =10.0,
    restitution=0.5,
    friction=0.3) 


circle_body_2 = world.CreateDynamicBody(
    position =(28,4)
    )


circle_fixture_2=circle_body_2.CreateCircleFixture(shape=b2CircleShape(radius=2),
    density =10.0,
    restitution=0.5,
    friction=0.3)


# Creating the pendulum


pendulum = world.CreateDynamicBody(
    position=(20,18),
    fixtures=b2FixtureDef(
        shape = b2PolygonShape(box=(0.5,10)),
        density = 1.0,
        friction = 0.3
    ))

# Defining Weld joints such that the wheels move with the planck

pendulum.isBullet = True

wheel_joint_def_1 = b2WeldJointDef( # Defining a Weld joint def
    bodyA = planck,
    bodyB=circle_body_1,
    anchor = (20,6), 
)
wheel_joint_def_2 = b2WeldJointDef( # Defining a Weld joint def
    bodyA = planck,
    bodyB=circle_body_2,
    anchor = (20,6), 
)

# Defining Revolute joint

joint_def = b2RevoluteJointDef( # Defining a Revolute joint def
    bodyA = planck,
    bodyB=pendulum,
    anchor = (20,6), # Point about which Dynamic_body will rotate
    # X axis is chosen such that the body is in equilibrium when it is stationary
    enableMotor=False
)
weld_joint_1= world.CreateJoint(wheel_joint_def_1)
weld_joint_2= world.CreateJoint(wheel_joint_def_2)
revolute_joint= world.CreateJoint(joint_def)

# Initializing Antecedent and Consequent statements to implement fuzzy on

Angular_Displacement = crtl.Antecedent(0.01*np.arange(-314,315),"Angular Displacement")
Angular_Velocity = crtl.Antecedent(0.01*np.arange(-100,100), "Angular Velocity")

Applied_Force = crtl.Consequent(0.01*np.arange(-100,100),"Force")


# Describing Fuzzy Membership Functions

Angular_Displacement["Negative_high"] = fuzz.trimf(Angular_Displacement.universe,[-3.14,-3.14,-0.0104667])
Angular_Displacement["Negative_low"] = fuzz.trimf(Angular_Displacement.universe,[-2.09334,-0.0104667,0])
Angular_Displacement["low"] = fuzz.trimf(Angular_Displacement.universe,[-0.0104667,0,0.0104667])
Angular_Displacement["Positive_low"] = fuzz.trimf(Angular_Displacement.universe,[0,0.0104667,2.09334])
Angular_Displacement["Positive_high"] = fuzz.trimf(Angular_Displacement.universe,[0.0104667,3.14,3.14])

Angular_Velocity["Negative_high"] = fuzz.trimf(Angular_Velocity.universe,[-1,-1,-0.333])
Angular_Velocity["Negative_low"] = fuzz.trimf(Angular_Velocity.universe,[-0.666,-0.333,0])
Angular_Velocity["low"] = fuzz.trimf(Angular_Velocity.universe,[-0.333,0,0.333])
Angular_Velocity["Positive_low"] = fuzz.trimf(Angular_Velocity.universe,[0,0.333,0.666])
Angular_Velocity["Positive_high"] = fuzz.trimf(Angular_Velocity.universe,[0.333,1,1])

Applied_Force["Negative_high"] = fuzz.trimf(Applied_Force.universe,[-1,-1,-0.333])
Applied_Force["Negative_low"] = fuzz.trimf(Applied_Force.universe,[-0.666,-0.333,0])
Applied_Force["low"] = fuzz.trimf(Applied_Force.universe,[-0.333,0,0.333])
Applied_Force["Positive_low"] = fuzz.trimf(Applied_Force.universe,[0,0.333,0.666])
Applied_Force["Positive_high"] = fuzz.trimf(Applied_Force.universe,[0.333,1,1])

# Defining Fuzzy Rules 

Rule_1 = crtl.Rule(Angular_Displacement["Negative_high"] | Angular_Velocity["Negative_high"], Applied_Force["Positive_high"])
Rule_2 = crtl.Rule(Angular_Displacement["Negative_low"] | Angular_Velocity["Negative_low"], Applied_Force["Positive_low"])
Rule_3 = crtl.Rule(Angular_Displacement["low"] | Angular_Velocity["low"], Applied_Force["low"])
Rule_4 = crtl.Rule(Angular_Displacement["Positive_low"] | Angular_Velocity["Positive_low"], Applied_Force["Negative_low"])
Rule_5 = crtl.Rule(Angular_Displacement["Positive_high"] | Angular_Velocity["Positive_high"], Applied_Force["Negative_high"])

# Creating the Control System

Fuzzy_Crtl = crtl.ControlSystem([Rule_1,Rule_2,Rule_3,Rule_4,Rule_5])
Fuzzy_Sys = crtl.ControlSystemSimulation(Fuzzy_Crtl)

# Defining a function which will help in rendering of shape in pygame

def draw_circle_shape(circle, circlefix):
    circle_radius = int(circlefix.shape.radius * PixPerMetre)
    circle_position = to_pygame(circle.position)
    pygame.draw.circle(screen, (255,255, 255), circle_position, circle_radius)
count = 0

# Rendering Box2D physics in pygame

clock = pygame.time.Clock()
running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
   
    world.Step(Time_Step,6,2)
    screen.fill((0, 0, 0))
    world.ClearForces()


    draw_circle_shape(circle_body_1,circle_fixture_1)
    draw_circle_shape(circle_body_2,circle_fixture_2)
    
    # For Checking functionality of Fuzzy

    # if (count/100)%2==0 and count>100:
    #     pendulum.ApplyLinearImpulse(b2Vec2(-10,0),pendulum.worldCenter,True)
    # elif(count/100-1)%2==0:
    #     pendulum.ApplyLinearImpulse(b2Vec2(10,0),pendulum.worldCenter,True)
    # count += 1

# Rendering the EdgeShape
    vertex1, vertex2 = ground.fixtures[0].shape.vertices
    v1_screen = to_pygame(ground.transform * vertex1)
    v2_screen = to_pygame(ground.transform * vertex2)
    # Draw the edge as a line in Pygame
    pygame.draw.line(screen, (255, 255, 255), v1_screen, v2_screen, 2)
   
    for fixture in planck.fixtures:
        shape = fixture.shape
        vertices = [(planck.transform * v) for v in shape.vertices]
        vertices = [to_pygame(v) for v in vertices]
        pygame.draw.polygon(screen, (255,255,255), vertices)
   
    for fixture in pendulum.fixtures:
        shape = fixture.shape
        vertices = [(pendulum.transform * v) for v in shape.vertices]
        vertices = [to_pygame(v) for v in vertices]
        pygame.draw.polygon(screen, (255,255,255), vertices)
    
    # Getting the anglle and angular velocity of the pendulum to implement fuzzy
    
    angle = pendulum.angle
    angle = (angle+np.pi)%(2*np.pi) - np.pi
    A_Velocity = pendulum.angularVelocity
    
    # Implementing Fuzzy

    Fuzzy_Sys.input["Angular Displacement"] = angle
    Fuzzy_Sys.input["Angular Velocity"] = A_Velocity
    
    Fuzzy_Sys.compute()

    planck.ApplyForce(force = b2Vec2(10000*Fuzzy_Sys.output["Force"],0),point=planck.worldCenter,wake=True) 

    print(Fuzzy_Sys.output["Force"], planck.worldCenter)
    
    pygame.display.flip()

    clock.tick(60)
pygame.quit() 

# Plotting the Memebership functions
Angular_Displacement.view()
Angular_Velocity.view()
Applied_Force.view()

plt.show()
