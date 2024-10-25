from Box2D import (b2World, b2WeldJointDef, b2EdgeShape, b2PolygonShape, b2CircleShape, b2FixtureDef, b2RevoluteJointDef)
import pygame


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
+

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
        shape = b2PolygonShape(box=(1,10)),
        density = 1.0,
        friction = 0.3
    ))

pendulum.isBullet = True
# Defining Revolute joint
wheel_joint_def_1 = b2WeldJointDef( # Defining a Revolute joint def
    bodyA = planck,
    bodyB=circle_body_1,
    anchor = (20,6), # Point about which Dynamic_body will rotate
)
wheel_joint_def_2 = b2WeldJointDef( # Defining a Revolute joint def
    bodyA = planck,
    bodyB=circle_body_2,
    anchor = (20,6), # Point about which Dynamic_body will rotate
)


joint_def = b2RevoluteJointDef( # Defining a Revolute joint def
    bodyA = planck,
    bodyB=pendulum,
    anchor = (20,5), # Point about which Dynamic_body will rotate
    # X axis is chosen such that the body is in equilibrium when it is stationary
    enableMotor=False
)
weld_joint_1= world.CreateJoint(wheel_joint_def_1)
weld_joint_2= world.CreateJoint(wheel_joint_def_2)
revolute_joint= world.CreateJoint(joint_def)

velcount = 0
def linearvel():
    global velcount
    if velcount!=30:
        velcount+=1
        planck.linearVelocity = (0.01*velcount,0)
        circle_body_1.linearVelocity = (0.01*velcount,0)
        circle_body_2.linearVelocity = (0.01*velcount,0)
    else:
        velcount = -31


# Defining a function which will help in rendering of shape in pygame
def draw_circle_shape(circle, circlefix):
    circle_radius = int(circlefix.shape.radius * PixPerMetre)
    circle_position = to_pygame(circle.position)
    pygame.draw.circle(screen, (0, 255, 0), circle_position, circle_radius)
count = 0
#Rendering Box2D physics in pygame
clock = pygame.time.Clock()
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
   
    world.Step(Time_Step,6,2)
    screen.fill((255, 255, 255))
    world.ClearForces()


    draw_circle_shape(circle_body_1,circle_fixture_1)
    draw_circle_shape(circle_body_2,circle_fixture_2)


# Rendering the EdgeShape
    vertex1, vertex2 = ground.fixtures[0].shape.vertices
    v1_screen = to_pygame(ground.transform * vertex1)
    v2_screen = to_pygame(ground.transform * vertex2)
    # Draw the edge as a line in Pygame
    pygame.draw.line(screen, (50, 50, 0), v1_screen, v2_screen, 2)
   
    for fixture in planck.fixtures:
        shape = fixture.shape
        vertices = [(planck.transform * v) for v in shape.vertices]
        vertices = [to_pygame(v) for v in vertices]
        pygame.draw.polygon(screen, (0,250,250), vertices)
   
    for fixture in pendulum.fixtures:
        shape = fixture.shape
        vertices = [(pendulum.transform * v) for v in shape.vertices]
        vertices = [to_pygame(v) for v in vertices]
        pygame.draw.polygon(screen, (0,250,250), vertices)
    pygame.display.flip()

    linearvel()

    clock.tick(60)
pygame.quit()
