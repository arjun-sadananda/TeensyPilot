import pygame
import numpy as np
from math import *
import serial
import time


q1 = 1
q2 = q3 = q4 = 0

DCM = np.matrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ])


WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (40, 40, 40)

WIDTH, HEIGHT = 800, 600

scale = 100
circle_pos = [WIDTH/2, HEIGHT/2]  # x, y


points = []

# all the cube vertices
points.append(np.matrix([-1, -1, .3]))
points.append(np.matrix([1, -1, .3]))
points.append(np.matrix([1,  1, .3]))
points.append(np.matrix([-1, 1, .3]))
points.append(np.matrix([-1, -1, -.3]))
points.append(np.matrix([1, -1, -.3]))
points.append(np.matrix([1, 1, -.3]))
points.append(np.matrix([-1, 1, -.3]))

pygame.display.set_caption("3D projection in pygame!")
screen = pygame.display.set_mode((WIDTH, HEIGHT))

clock = pygame.time.Clock()

teensyPilot = serial.Serial(port='COM4',   baudrate=57600, timeout=.1)

projected_points = [
    [n, n] for n in range(len(points))
]


def connect_points(i, j, points, color):
    pygame.draw.line(
        screen, color, (points[i][0], points[i][1]), (points[j][0], points[j][1]), 3)

def serial_read():
    global q1, q2, q3, q4
    # arduino.write(bytes(x,   'utf-8'))
    # time.sleep(0.05)
    data = teensyPilot.readline().decode("utf-8").strip()
    vals = data.split(',')
    print(data)
    if len(vals) == 4:
        q1 = float(vals[0])
        q2 = float(vals[1])
        q3 = float(vals[2])
        q4 = float(vals[3])
    else: 
        q1 = 1
        q2 = q3 = q4 = 0
    return

def assign_DCM():
    global DCM

    q3q3 = q3 * q3
    q3q4 = q3 * q4
    q2q2 = q2 * q2
    q2q3 = q2 * q3
    q2q4 = q2 * q4
    q1q2 = q1 * q2
    q1q3 = q1 * q3
    q1q4 = q1 * q4
    q4q4 = q4 * q4

    max = 1.0-2.0*(q3q3 + q4q4)
    may = 2.0*(q2q3 - q1q4)
    maz = 2.0*(q2q4 + q1q3)
    mbx = 2.0*(q2q3 + q1q4)
    mby = 1.0-2.0*(q2q2 + q4q4)
    mbz = 2.0*(q3q4 - q1q2)
    mcx = 2.0*(q2q4 - q1q3)
    mcy = 2.0*(q3q4 + q1q2)
    mcz = 1.0-2.0*(q2q2 + q3q3)

    # DCM = np.matrix([
    #     [max, may, maz],
    #     [mbx, mby, mbz],
    #     [mcx, mcy, mcz],
    # ])
    DCM = np.matrix([
        [max, may, maz],
        [mbx, mby, mbz],
        [mcx, mcy, mcz],
    ])


while True:

    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                pygame.quit()
                exit()

    # update stuff
    # angle = int.from_bytes(serial_read(), "big")
    serial_read()
    
    # print(str(q1) + " " + str(q2) + " " + str(q3) + " " +  str(q4))
    assign_DCM()

    screen.fill(BLACK)
    # drawining stuff

    i = 0
    
    for point in points:
        rotated2d = np.dot(DCM, point.reshape((3, 1)))
        x = float(rotated2d[0][0])
        y = float(rotated2d[1][0])
        z = float(rotated2d[2][0])
        # print(str(int(x)) + " " + str(int(y)) + " " + str(int(z)))
        rotated2d[0][0] = y
        rotated2d[1][0] = -z
        rotated2d[2][0] = -x
        distance = 10
        z = 1/(distance - float(rotated2d[2][0]))
        projection_matrix = np.matrix([
            [z, 0, 0],
            [0, z, 0]
        ])
        projected2d = np.dot(projection_matrix, rotated2d)

        x = int(projected2d[0][0] * scale *distance) + circle_pos[0]
        y = int(projected2d[1][0] * scale *distance) + circle_pos[1]

        projected_points[i] = [x, y]
        # print(float(point.reshape((3, 1))[0]))
        if float(point.reshape((3, 1))[0]) < 0:
            pygame.draw.circle(screen, RED, (x, y), 5)
        else:
            pygame.draw.circle(screen, GREEN, (x, y), 5)
        i += 1

    for p in range(4):
        connect_points(p, (p+1) % 4, projected_points, WHITE)
        connect_points(p+4, ((p+1) % 4) + 4, projected_points, WHITE)
        connect_points(p, (p+4), projected_points, WHITE)

    pygame.display.update()
