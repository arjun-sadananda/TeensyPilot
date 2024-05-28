import pygame
import numpy as np
from math import *
import serial
import time

pygame.init()

q1_m = 1
q2_m = q3_m = q4_m = 0
q1_ma = 1
q2_ma = q3_ma = q4_ma = 0
q1_mt = 1
q2_mt = q3_mt = q4_mt = 0
a_m_x = a_m_y = a_m_z = m_m_x = m_m_y = m_m_z = 0
a_p_x = a_p_y = a_p_z = m_p_x = m_p_y = m_p_z = 0
a_m_x_ma = a_m_y_ma = a_m_z_ma = m_m_x_ma = m_m_y_ma = m_m_z_ma = 0
a_p_x_ma = a_p_y_ma = a_p_z_ma = m_p_x_ma = m_p_y_ma = m_p_z_ma = 0
a_m_x_mt = a_m_y_mt = a_m_z_mt = m_m_x_mt = m_m_y_mt = m_m_z_mt = 0
a_p_x_mt = a_p_y_mt = a_p_z_mt = m_p_x_mt = m_p_y_mt = m_p_z_mt = 0

eye = np.matrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ])
DCM_mekf2 = np.matrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ])
DCM_mekf2_acc = np.matrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ])
DCM_mekf2_triad = np.matrix([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ])

WHITE = (255, 255, 255)
L_RED = (255, 200, 150)
L_GREEN = (150, 255, 150)
L_BLUE = (150, 200, 255)
RED   = (255, 0, 0)
GREEN = (0, 255, 0)
CYAN  = (0, 100, 255)
BLACK = (40, 40, 40)

WIDTH, HEIGHT = 1500, 800

c_cube = [.2*WIDTH, HEIGHT/2]  # x, y
c_needle1 = [.47*WIDTH, .3*HEIGHT]  # x, y
c_needle2 = [.47*WIDTH, .65*HEIGHT]  # x, y
c_needle3 = [.65*WIDTH,   .3*HEIGHT]  # x, y
c_needle4 = [.65*WIDTH,   .65*HEIGHT]  # x, y
c_needle5 = [.83*WIDTH,   .3*HEIGHT]  # x, y
c_needle6 = [.83*WIDTH,   .65*HEIGHT]  # x, y

font = pygame.font.Font('freesansbold.ttf', 32)
label_mekf = font.render('MEKF2', True, L_RED, BLACK)
label_mekf_Rm = font.render('MEKF2 Rm>Ra', True, L_GREEN, BLACK)
label_mekf_triad = font.render('MEKF2_TRIAD', True, L_BLUE, BLACK)
font = pygame.font.Font('freesansbold.ttf', 24)
text_x = font.render('x', True, RED, BLACK)
text_y = font.render('y', True, GREEN, BLACK)
text_z = font.render('z', True, CYAN, BLACK)
font = pygame.font.Font('freesansbold.ttf', 20)
label_m = font.render('Reference Vector Measurements', True, RED, BLACK)
label_p = font.render('Predicted Measurements', True, CYAN, BLACK)
label_body = font.render('Body Frame', True, WHITE, BLACK)
label_global = font.render('Global Frame', True, WHITE, BLACK)

label_body = pygame.transform.rotate(label_body, 270)
label_global = pygame.transform.rotate(label_global, 270)

textRect_x = text_x.get_rect()
textRect_y = text_y.get_rect()
textRect_z = text_z.get_rect()
labelRect_mekf = label_mekf.get_rect()
labelRect_mekf_Rm = label_mekf_Rm.get_rect()
labelRect_mekf_triad = label_mekf_triad.get_rect()
labelRect_m = label_m.get_rect()
labelRect_p = label_p.get_rect()
labelRect_body = label_body.get_rect()
labelRect_global = label_global.get_rect()

labelRect_mekf.center = (c_needle1[0], .1*HEIGHT)
labelRect_mekf_Rm.center = (c_needle3[0], .1*HEIGHT)
labelRect_mekf_triad.center = (c_needle5[0], .1*HEIGHT)
labelRect_m.center = (c_needle3[0], .9*HEIGHT)
labelRect_p.center = (c_needle3[0], .95*HEIGHT)
labelRect_body.center = (.95*WIDTH, c_needle1[1])
labelRect_global.center = (.95*WIDTH, c_needle2[1])

points = []

# all the cube vertices
points.append(np.matrix([-1, -1, .4]))
points.append(np.matrix([1, -1, .4]))
points.append(np.matrix([1,  1, .4]))
points.append(np.matrix([-1, 1, .4]))
points.append(np.matrix([-1, -1, -.4]))
points.append(np.matrix([1, -1, -.4]))
points.append(np.matrix([1, 1, -.4]))
points.append(np.matrix([-1, 1, -.4]))

pygame.display.set_caption("3D projection in pygame!")
screen = pygame.display.set_mode((WIDTH, HEIGHT))

clock = pygame.time.Clock()

teensyPilot = serial.Serial(port='COM3',   baudrate=115200, timeout=.1)

proj_points_q = [
    [n, n] for n in range(len(points))
]
proj_points_q_ma = [
    [n, n] for n in range(len(points))
]
proj_points_q_mt = [
    [n, n] for n in range(len(points))
]


def connect_points(i, j, points, color):
    pygame.draw.line(
        screen, color, (points[i][0], points[i][1]), (points[j][0], points[j][1]), 3)

def serial_read():
    global q1_m, q2_m, q3_m, q4_m, q1_ma, q2_ma, q3_ma, q4_ma, q1_mt, q2_mt, q3_mt, q4_mt
    global a_m_x, a_m_y, a_m_z, m_m_x, m_m_y, m_m_z, a_p_x, a_p_y, a_p_z, m_p_x, m_p_y, m_p_z
    global a_m_x_ma, a_m_y_ma, a_m_z_ma, m_m_x_ma, m_m_y_ma, m_m_z_ma, a_p_x_ma, a_p_y_ma, a_p_z_ma, m_p_x_ma, m_p_y_ma, m_p_z_ma
    global a_m_x_mt, a_m_y_mt, a_m_z_mt, m_m_x_mt, m_m_y_mt, m_m_z_mt, a_p_x_mt, a_p_y_mt, a_p_z_mt, m_p_x_mt, m_p_y_mt, m_p_z_mt
    # arduino.write(bytes(x,   'utf-8'))
    # time.sleep(0.05)
    data = teensyPilot.readline().decode("utf-8").strip()
    vals = data.split(',')
    # print(data)
    if len(vals) == 48:
        q1_m     = float(vals[0])
        q2_m     = float(vals[1])
        q3_m     = float(vals[2])
        q4_m     = float(vals[3])

        q1_ma   = float(vals[4])
        q2_ma   = float(vals[5])
        q3_ma   = float(vals[6])
        q4_ma   = float(vals[7])

        q1_mt   = float(vals[8])
        q2_mt   = float(vals[9])
        q3_mt   = float(vals[10])
        q4_mt   = float(vals[11])


        a_m_x   = -float(vals[12])
        a_m_y   = -float(vals[13])
        a_m_z   = -float(vals[14])
        m_m_x   = -float(vals[15])
        m_m_y   = -float(vals[16])
        m_m_z   = -float(vals[17])

        a_p_x   = -float(vals[18])
        a_p_y   = -float(vals[19])
        a_p_z   = -float(vals[20])
        m_p_x   = -float(vals[21])
        m_p_y   = -float(vals[22])
        m_p_z   = -float(vals[23])


        a_m_x_ma   = -float(vals[24])
        a_m_y_ma   = -float(vals[25])
        a_m_z_ma   = -float(vals[26])
        m_m_x_ma   = -float(vals[27])
        m_m_y_ma   = -float(vals[28])
        m_m_z_ma   = -float(vals[29])

        a_p_x_ma   = -float(vals[30])
        a_p_y_ma   = -float(vals[31])
        a_p_z_ma   = -float(vals[32])
        m_p_x_ma   = -float(vals[33])
        m_p_y_ma   = -float(vals[34])
        m_p_z_ma   = -float(vals[35])


        a_m_x_mt   = -float(vals[36])
        a_m_y_mt   = -float(vals[37])
        a_m_z_mt   = -float(vals[38])
        m_m_x_mt   = -float(vals[39])
        m_m_y_mt   = -float(vals[40])
        m_m_z_mt   = -float(vals[41])

        a_p_x_mt   = -float(vals[42])
        a_p_y_mt   = -float(vals[43])
        a_p_z_mt   = -float(vals[44])
        m_p_x_mt   = -float(vals[45])
        m_p_y_mt   = -float(vals[46])
        m_p_z_mt   = -float(vals[47])
        
        # print(sqrt(q1_mekf**2+q2_mekf**2+q3_mekf**2+q4_mekf**2))
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

def get_proj_xy(DCM, point, center, scale, distance):
    rotated2d  = np.dot(DCM, point.reshape((3, 1)))
    x = float(rotated2d[0][0])
    y = float(rotated2d[1][0])
    z = float(rotated2d[2][0])
    # print(str(int(x)) + " " + str(int(y)) + " " + str(int(z)))
    rotated2d[0][0] = y
    rotated2d[1][0] = -z
    rotated2d[2][0] = -x
    
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
    DCM_mekf2      = get_DCM(q1_m, q2_m, q3_m, q4_m)
    DCM_mekf2_acc    = get_DCM(q1_ma , q2_ma , q3_ma , q4_ma)
    DCM_mekf2_triad    = get_DCM(q1_mt , q2_mt , q3_mt , q4_mt)
    screen.fill(BLACK)
    # drawining stuff

    i = 0
    
    for point in points:

        proj_points_q[i]  = get_proj_xy(DCM_mekf2, point, c_cube, 150, 50)
        proj_points_q_ma[i] = get_proj_xy(DCM_mekf2_acc, point, c_cube, 200, 50)
        proj_points_q_mt[i] = get_proj_xy(DCM_mekf2_triad, point, c_cube, 250, 50)

        # print(float(point.reshape((3, 1))[0]))
        if float(point.reshape((3, 1))[0]) < 0:
            pygame.draw.circle(screen, RED, (proj_points_q[i][0], proj_points_q[i][1]), 5)
            pygame.draw.circle(screen, RED, (proj_points_q_ma[i][0], proj_points_q_ma[i][1]), 5)
            pygame.draw.circle(screen, RED, (proj_points_q_mt[i][0], proj_points_q_mt[i][1]), 5)
        else:
            pygame.draw.circle(screen, GREEN, (proj_points_q[i][0], proj_points_q[i][1]), 5)
            pygame.draw.circle(screen, GREEN, (proj_points_q_ma[i][0], proj_points_q_ma[i][1]), 5)
            pygame.draw.circle(screen, GREEN, (proj_points_q_mt[i][0], proj_points_q_mt[i][1]), 5)
        i += 1

    for p in range(4):
        connect_points(p, (p+1) % 4, proj_points_q, L_RED)
        connect_points(p+4, ((p+1) % 4) + 4, proj_points_q, L_RED)
        connect_points(p, (p+4), proj_points_q, L_RED)
        
        connect_points(p, (p+1) % 4, proj_points_q_ma, L_GREEN)
        connect_points(p+4, ((p+1) % 4) + 4, proj_points_q_ma, L_GREEN)
        connect_points(p, (p+4), proj_points_q_ma, L_GREEN)

        connect_points(p, (p+1) % 4, proj_points_q_mt, L_BLUE)
        connect_points(p+4, ((p+1) % 4) + 4, proj_points_q_mt, L_BLUE)
        connect_points(p, (p+4), proj_points_q_mt, L_BLUE)

    ball_size = 100

    # MEKF2 needles in body frame
    pygame.draw.circle(screen, WHITE, c_needle1, ball_size)
    pygame.draw.circle(screen, BLACK, c_needle1, ball_size-2)
    textRect_x.center = (c_needle1[0], c_needle1[1])
    textRect_y.center = (c_needle1[0] + ball_size + textRect_y.width, c_needle1[1])
    textRect_z.center = (c_needle1[0], c_needle1[1] + ball_size + textRect_z.height/2)
    screen.blit(text_x, textRect_x)
    screen.blit(text_y, textRect_y)
    screen.blit(text_z, textRect_z)
    a_m_proj  = get_proj_xy(eye, np.matrix([a_m_x, a_m_y, a_m_z]), c_needle1, 100, 5)
    m_m_proj  = get_proj_xy(eye, np.matrix([m_m_x, m_m_y, m_m_z]), c_needle1, 100, 5)
    a_p_proj  = get_proj_xy(eye, np.matrix([a_p_x, a_p_y, a_p_z]), c_needle1, 100, 5)
    m_p_proj  = get_proj_xy(eye, np.matrix([m_p_x, m_p_y, m_p_z]), c_needle1, 100, 5)
    pygame.draw.line(screen, RED, c_needle1, a_m_proj, 3)
    pygame.draw.line(screen, RED, c_needle1, m_m_proj, 3)
    pygame.draw.line(screen, CYAN, c_needle1, a_p_proj, 3)
    pygame.draw.line(screen, CYAN, c_needle1, m_p_proj, 3)

    # MEKF2 needles in global frame
    pygame.draw.circle(screen, WHITE, c_needle2, ball_size)
    pygame.draw.circle(screen, BLACK, c_needle2, ball_size-2)
    textRect_x.center = (c_needle2[0], c_needle2[1])
    textRect_y.center = (c_needle2[0] + ball_size + textRect_y.width, c_needle2[1])
    textRect_z.center = (c_needle2[0], c_needle2[1] + ball_size + textRect_z.height/2)
    screen.blit(text_x, textRect_x)
    screen.blit(text_y, textRect_y)
    screen.blit(text_z, textRect_z)
    a_m_proj  = get_proj_xy(DCM_mekf2, np.matrix([a_m_x, a_m_y, a_m_z]), c_needle2, 100, 5)
    m_m_proj  = get_proj_xy(DCM_mekf2, np.matrix([m_m_x, m_m_y, m_m_z]), c_needle2, 100, 5)
    a_p_proj  = get_proj_xy(DCM_mekf2, np.matrix([a_p_x, a_p_y, a_p_z]), c_needle2, 100, 5)
    m_p_proj  = get_proj_xy(DCM_mekf2, np.matrix([m_p_x, m_p_y, m_p_z]), c_needle2, 100, 5)
    pygame.draw.line(screen, RED, c_needle2, a_m_proj, 3)
    pygame.draw.line(screen, RED, c_needle2, m_m_proj, 3)
    pygame.draw.line(screen, CYAN, c_needle2, a_p_proj, 3)
    pygame.draw.line(screen, CYAN, c_needle2, m_p_proj, 3)

    # MEKF2_acc needles in body frame
    
    pygame.draw.circle(screen, WHITE, c_needle3, ball_size)
    pygame.draw.circle(screen, BLACK, c_needle3, ball_size-2)
    textRect_x.center = (c_needle3[0], c_needle3[1])
    textRect_y.center = (c_needle3[0] + ball_size + textRect_y.width, c_needle3[1])
    textRect_z.center = (c_needle3[0], c_needle3[1] + ball_size + textRect_z.height/2)
    screen.blit(text_x, textRect_x)
    screen.blit(text_y, textRect_y)
    screen.blit(text_z, textRect_z)
    a_m_proj  = get_proj_xy(eye, np.matrix([a_m_x_ma, a_m_y_ma, a_m_z_ma]), c_needle3, 100, 5)
    m_m_proj  = get_proj_xy(eye, np.matrix([m_m_x_ma, m_m_y_ma, m_m_z_ma]), c_needle3, 100, 5)
    a_p_proj  = get_proj_xy(eye, np.matrix([a_p_x_ma, a_p_y_ma, a_p_z_ma]), c_needle3, 100, 5)
    m_p_proj  = get_proj_xy(eye, np.matrix([m_p_x_ma, m_p_y_ma, m_p_z_ma]), c_needle3, 100, 5)
    pygame.draw.line(screen, RED, c_needle3, a_m_proj, 3)
    pygame.draw.line(screen, RED, c_needle3, m_m_proj, 3)
    pygame.draw.line(screen, CYAN, c_needle3, a_p_proj, 3)
    pygame.draw.line(screen, CYAN, c_needle3, m_p_proj, 3)

    # MEKF2_acc needles in global frame
    
    pygame.draw.circle(screen, WHITE, c_needle4, ball_size)
    pygame.draw.circle(screen, BLACK, c_needle4, ball_size-2)
    textRect_x.center = (c_needle4[0], c_needle4[1])
    textRect_y.center = (c_needle4[0] + ball_size + textRect_y.width, c_needle4[1])
    textRect_z.center = (c_needle4[0], c_needle4[1] + ball_size + textRect_z.height/2)
    screen.blit(text_x, textRect_x)
    screen.blit(text_y, textRect_y)
    screen.blit(text_z, textRect_z)
    a_m_proj  = get_proj_xy(DCM_mekf2_triad, np.matrix([a_m_x_ma, a_m_y_ma, a_m_z_ma]), c_needle4, 100, 5)
    m_m_proj  = get_proj_xy(DCM_mekf2_triad, np.matrix([m_m_x_ma, m_m_y_ma, m_m_z_ma]), c_needle4, 100, 5)
    a_p_proj  = get_proj_xy(DCM_mekf2_triad, np.matrix([a_p_x_ma, a_p_y_ma, a_p_z_ma]), c_needle4, 100, 5)
    m_p_proj  = get_proj_xy(DCM_mekf2_triad, np.matrix([m_p_x_ma, m_p_y_ma, m_p_z_ma]), c_needle4, 100, 5)
    pygame.draw.line(screen, RED, c_needle4, a_m_proj, 3)
    pygame.draw.line(screen, RED, c_needle4, m_m_proj, 3)
    pygame.draw.line(screen, CYAN, c_needle4, a_p_proj, 3)
    pygame.draw.line(screen, CYAN, c_needle4, m_p_proj, 3)

    # MEKF2_TRIAD needles in body frame
    
    pygame.draw.circle(screen, WHITE, c_needle5, ball_size)
    pygame.draw.circle(screen, BLACK, c_needle5, ball_size-2)
    textRect_x.center = (c_needle5[0], c_needle5[1])
    textRect_y.center = (c_needle5[0] + ball_size + textRect_y.width, c_needle5[1])
    textRect_z.center = (c_needle5[0], c_needle5[1] + ball_size + textRect_z.height/2)
    screen.blit(text_x, textRect_x)
    screen.blit(text_y, textRect_y)
    screen.blit(text_z, textRect_z)
    a_m_proj  = get_proj_xy(eye, np.matrix([a_m_x_mt, a_m_y_mt, a_m_z_mt]), c_needle5, 100, 5)
    m_m_proj  = get_proj_xy(eye, np.matrix([m_m_x_mt, m_m_y_mt, m_m_z_mt]), c_needle5, 100, 5)
    a_p_proj  = get_proj_xy(eye, np.matrix([a_p_x_mt, a_p_y_mt, a_p_z_mt]), c_needle5, 100, 5)
    m_p_proj  = get_proj_xy(eye, np.matrix([m_p_x_mt, m_p_y_mt, m_p_z_mt]), c_needle5, 100, 5)
    pygame.draw.line(screen, RED, c_needle5, a_m_proj, 3)
    pygame.draw.line(screen, RED, c_needle5, m_m_proj, 3)
    pygame.draw.line(screen, CYAN, c_needle5, a_p_proj, 3)
    pygame.draw.line(screen, CYAN, c_needle5, m_p_proj, 3)

    # MEKF2_TRIAD needles in global frame
    
    pygame.draw.circle(screen, WHITE, c_needle6, ball_size)
    pygame.draw.circle(screen, BLACK, c_needle6, ball_size-2)
    textRect_x.center = (c_needle6[0], c_needle6[1])
    textRect_y.center = (c_needle6[0] + ball_size + textRect_y.width, c_needle6[1])
    textRect_z.center = (c_needle6[0], c_needle6[1] + ball_size + textRect_z.height/2)
    screen.blit(text_x, textRect_x)
    screen.blit(text_y, textRect_y)
    screen.blit(text_z, textRect_z)
    a_m_proj  = get_proj_xy(DCM_mekf2_triad, np.matrix([a_m_x_mt, a_m_y_mt, a_m_z_mt]), c_needle6, 100, 5)
    m_m_proj  = get_proj_xy(DCM_mekf2_triad, np.matrix([m_m_x_mt, m_m_y_mt, m_m_z_mt]), c_needle6, 100, 5)
    a_p_proj  = get_proj_xy(DCM_mekf2_triad, np.matrix([a_p_x_mt, a_p_y_mt, a_p_z_mt]), c_needle6, 100, 5)
    m_p_proj  = get_proj_xy(DCM_mekf2_triad, np.matrix([m_p_x_mt, m_p_y_mt, m_p_z_mt]), c_needle6, 100, 5)
    pygame.draw.line(screen, RED,  c_needle6, a_m_proj, 3)
    pygame.draw.line(screen, RED,  c_needle6, m_m_proj, 3)
    pygame.draw.line(screen, CYAN, c_needle6, a_p_proj, 3)
    pygame.draw.line(screen, CYAN, c_needle6, m_p_proj, 3)
    
    # Labels

    screen.blit(label_mekf, labelRect_mekf)
    screen.blit(label_mekf_Rm, labelRect_mekf_Rm)
    screen.blit(label_mekf_triad, labelRect_mekf_triad)
    screen.blit(label_m, labelRect_m)
    screen.blit(label_p, labelRect_p)
    
    screen.blit(label_body, labelRect_body)
    screen.blit(label_global, labelRect_global)

    pygame.display.update()
