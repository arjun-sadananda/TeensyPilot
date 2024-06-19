import pygame
import numpy as np
from math import *
import serial
import time


q1_triad = 1
q2_triad = q3_triad = q4_triad = 0
q1_mekf = 1
q2_mekf = q3_mekf = q4_mekf = 0
q1_mekf2 = 1
q2_mekf2 = q3_mekf2 = q4_mekf2 = 0
q1_mt = 1
q2_mt = q3_mt = q4_mt = 0

DCM_triad = np.matrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ])
DCM_mekf = np.matrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ])
DCM_mekf2 = np.matrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ])
DCM_mt = np.matrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ])


WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (40, 40, 40)

WIDTH, HEIGHT = 1500, 600

c1 = [.15*WIDTH, HEIGHT/2]  # x, y
c2 = [.375*WIDTH, HEIGHT/2]  # x, y
c3 = [.625*WIDTH, HEIGHT/2]  # x, y
c4 = [.85*WIDTH, HEIGHT/2]  # x, y


pygame.init()

font = pygame.font.Font('freesansbold.ttf', 32)
label_triad = font.render('TRIAD', True, WHITE, BLACK)
label_mekf = font.render('MEKF', True, WHITE, BLACK)
label_mekf2 = font.render('MEKF2', True, WHITE, BLACK)
label_mekf2_triad = font.render('MEKF2+TRIAD', True, WHITE, BLACK)


labelRect_triad = label_triad.get_rect()
labelRect_mekf = label_mekf.get_rect()
labelRect_mekf2 = label_mekf2.get_rect()
labelRect_mekf2_triad = label_mekf2_triad.get_rect()

labelRect_triad.center = (c1[0], .1*HEIGHT)
labelRect_mekf.center = (c2[0], .1*HEIGHT)
labelRect_mekf2.center = (c3[0], .1*HEIGHT)
labelRect_mekf2_triad.center = (c4[0], .1*HEIGHT)

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

teensyPilot = serial.Serial(port='COM3',   baudrate=57600, timeout=.1)

proj_points_1 = [
    [n, n] for n in range(len(points))
]
proj_points_2 = [
    [n, n] for n in range(len(points))
]
proj_points_3 = [
    [n, n] for n in range(len(points))
]
proj_points_4 = [
    [n, n] for n in range(len(points))
]


def connect_points(i, j, points, color):
    pygame.draw.line(
        screen, color, (points[i][0], points[i][1]), (points[j][0], points[j][1]), 3)

def serial_read():
    global q1_triad, q2_triad, q3_triad, q4_triad, q1_mekf, q2_mekf, q3_mekf, q4_mekf, q1_mekf2, q2_mekf2, q3_mekf2, q4_mekf2, q1_mt, q2_mt, q3_mt, q4_mt
    # arduino.write(bytes(x,   'utf-8'))
    # time.sleep(0.05)
    data = teensyPilot.readline().decode("utf-8").strip()
    vals = data.split(',')
    # print(data)
    if len(vals) == 4*4:
        q1_triad    = float(vals[0])
        q2_triad    = float(vals[1])
        q3_triad    = float(vals[2])
        q4_triad    = float(vals[3])
        q1_mekf     = float(vals[4])
        q2_mekf     = float(vals[5])
        q3_mekf     = float(vals[6])
        q4_mekf     = float(vals[7])
        q1_mekf2    = float(vals[8])
        q2_mekf2    = float(vals[9])
        q3_mekf2    = float(vals[10])
        q4_mekf2    = float(vals[11])
        q1_mt       = float(vals[12])
        q2_mt       = float(vals[13])
        q3_mt       = float(vals[14])
        q4_mt       = float(vals[15])
        print(sqrt(q1_mekf**2+q2_mekf**2+q3_mekf**2+q4_mekf**2))
    # else: 
        # q1_triad = 1
        # q2_triad = q3_triad = q4_triad = 0
    return

def get_DCM(q1, q2, q3, q4):

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

    return DCM

def get_proj_xy(DCM, point, center):
    rotated2d  = np.dot(DCM, point.reshape((3, 1)))
    x = float(rotated2d[0][0])
    y = float(rotated2d[1][0])
    z = float(rotated2d[2][0])
    # print(str(int(x)) + " " + str(int(y)) + " " + str(int(z)))
    rotated2d[0][0] = y
    rotated2d[1][0] = -z
    rotated2d[2][0] = -x
    
    scale = 100
    distance = 10
    z = 1/(distance - float(rotated2d[2][0]))
    projection_matrix = np.matrix([
        [z, 0, 0],
        [0, z, 0]
    ])
    projected2d = np.dot(projection_matrix, rotated2d)

    x = int(projected2d[0][0] * scale *distance) + center[0]
    y = int(projected2d[1][0] * scale *distance) + center[1]

    return [x, y]

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
    DCM_triad   = get_DCM(q1_triad, q2_triad, q3_triad, q4_triad)
    DCM_mekf    = get_DCM(q1_mekf , q2_mekf , q3_mekf , q4_mekf)
    DCM_mekf2   = get_DCM(q1_mekf2, q2_mekf2, q3_mekf2, q4_mekf2)
    DCM_mt      = get_DCM(q1_mt, q2_mt, q3_mt, q4_mt)

    screen.fill(BLACK)
    # drawining stuff

    i = 0
    
    for point in points:

        proj_points_1[i] = get_proj_xy(DCM_triad, point, c1)
        proj_points_2[i] = get_proj_xy(DCM_mekf, point, c2)
        proj_points_3[i] = get_proj_xy(DCM_mekf2, point, c3)
        proj_points_4[i] = get_proj_xy(DCM_mt, point, c4)

        # print(float(point.reshape((3, 1))[0]))
        if float(point.reshape((3, 1))[0]) < 0:
            pygame.draw.circle(screen, RED, (proj_points_1[i][0], proj_points_1[i][1]), 5)
            pygame.draw.circle(screen, RED, (proj_points_2[i][0], proj_points_2[i][1]), 5)
            pygame.draw.circle(screen, RED, (proj_points_3[i][0], proj_points_3[i][1]), 5)
            pygame.draw.circle(screen, RED, (proj_points_4[i][0], proj_points_4[i][1]), 5)
        else:
            pygame.draw.circle(screen, GREEN, (proj_points_1[i][0], proj_points_1[i][1]), 5)
            pygame.draw.circle(screen, GREEN, (proj_points_2[i][0], proj_points_2[i][1]), 5)
            pygame.draw.circle(screen, GREEN, (proj_points_3[i][0], proj_points_3[i][1]), 5)
            pygame.draw.circle(screen, GREEN, (proj_points_4[i][0], proj_points_4[i][1]), 5)
        i += 1

    for p in range(4):
        connect_points(p, (p+1) % 4, proj_points_1, WHITE)
        connect_points(p+4, ((p+1) % 4) + 4, proj_points_1, WHITE)
        connect_points(p, (p+4), proj_points_1, WHITE)
        
        connect_points(p, (p+1) % 4, proj_points_2, WHITE)
        connect_points(p+4, ((p+1) % 4) + 4, proj_points_2, WHITE)
        connect_points(p, (p+4), proj_points_2, WHITE)
        
        connect_points(p, (p+1) % 4, proj_points_3, WHITE)
        connect_points(p+4, ((p+1) % 4) + 4, proj_points_3, WHITE)
        connect_points(p, (p+4), proj_points_3, WHITE)
        
        connect_points(p, (p+1) % 4, proj_points_4, WHITE)
        connect_points(p+4, ((p+1) % 4) + 4, proj_points_4, WHITE)
        connect_points(p, (p+4), proj_points_4, WHITE)


    screen.blit(label_triad, labelRect_triad)
    screen.blit(label_mekf, labelRect_mekf)
    screen.blit(label_mekf2, labelRect_mekf2)
    screen.blit(label_mekf2_triad, labelRect_mekf2_triad)

    pygame.display.update()
