import numpy as np
import matplotlib.pyplot as plt
import serial
import time
import csv


SERIAL_PORT = 'COM4'
BAUD_RATE = 57600 #115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=.1)


# L = 800
mx = []
my = []
mz = []
mx_cal = []
my_cal = []
mz_cal = []

# offset = [281.3, 1002.3, -322.54] #[344.1, 904.5, -152]
# offset = [-4.351436, 744.005033, -102.588960]

# offset = [-720.637, 1365.0, 2270.015]       # LSM9DS1
offset = [-587.808893, 1041.364679, 2152.803319]    #LSM 2

# offset = [-711.036762, 1417.547336, 2278.896953]

# cal = [0.675370, 0.702583, 0.519064, 0.009404, 0.081287, 0.000380]
# cal = [1.29, 1.31, 1.28, 0.027, -0.0698, 0.01394] #[1.46988, 1.4670, 1.4472, 0.0032, -.0512, .050277]
# cal = [1.320546, 1.152157, 1.301622, 0.000088, -0.026157, 0.122299] 
# cal = [0.670999, 0.688844, 0.518978, 0.008017, 0.080208, -0.002301]     #LSM9Ds1
cal = [0.547561, 0.553483, 0.590261, -0.023879, -0.005508, -0.002054] #LSM 2
# cal = [1.294398, 1.277933, 1.369600, 0.002373, -0.005930, -0.039244]
# 0 3 4
#   1 5
#     2

skip = 5
i =0
N=0
def read_and_process_data():
    global mx, my, mz, i, skip, N, mx_cal, my_cal, mz_cal
    try:
        # while(ser.in_waiting>6):
        # if ser.in_waiting>6:
        line = ser.readline().decode('utf-8').strip()
        # lines = ser.read_all().decode('utf-8')
        # val_lists = lines.split('\n')

        # for val_list in val_lists:
        # vals = line.split(',')
        # mx.append(int(line))
        # my.append(float(vals[1]))
        # mz.append(float(vals[2]))
        if i<skip:
            i+=1
        else:
            vals = line.split(',')
            # print(vals)
            mx.append(int(vals[0]))
            my.append(int(vals[1]))
            mz.append(int(vals[2]))
            print(vals, "count: ", N)
            i=0
            N+=1

            mx_cal_t = int(vals[0]) - offset[0]
            my_cal_t = int(vals[1]) - offset[1]
            mz_cal_t = int(vals[2]) - offset[2]
            mx_cal_t = cal[0] * mx_cal_t + cal[3] * my_cal_t + cal[4] * mz_cal_t
            my_cal_t = cal[3] * mx_cal_t + cal[1] * my_cal_t + cal[5] * mz_cal_t
            mz_cal_t = cal[4] * mx_cal_t + cal[5] * my_cal_t + cal[2] * mz_cal_t

            mx_cal.append(mx_cal_t)
            my_cal.append(my_cal_t)
            mz_cal.append(mz_cal_t)

            
            print(mx_cal[N-1], my_cal[N-1], mz_cal[N-1], "count: ", N)


        # if N==150 or N==300:
        #     skip = 
        # # Limit x and y lists to 20 items
        # theta_gyro = theta_gyro[-L:]
        # theta_accel = theta_accel[-L:]
        # theta_k = theta_k[-L:]

    except:
        #do nothing
        print("nothing")


# while True:
#     read_and_process_data()
#     # time.sleep(0.05)
#     if N == 750:
#         break

# csv.reader
# file = open('mag_read.csv')
# csvreader = csv.reader(file)
# for row in csvreader:
#     # temp = next(csvreader)
#     print(row)
#     mx.append(int(row[0]))
#     my.append(int(row[1]))
#     mz.append(int(row[2]))

#     mx_cal_t = int(row[0]) - offset[0]
#     my_cal_t = int(row[1]) - offset[1]
#     mz_cal_t = int(row[2]) - offset[2]
#     mx_cal_t = cal[0] * mx_cal_t + cal[3] * my_cal_t + cal[4] * mz_cal_t
#     my_cal_t = cal[3] * mx_cal_t + cal[1] * my_cal_t + cal[5] * mz_cal_t
#     mz_cal_t = cal[4] * mx_cal_t + cal[5] * my_cal_t + cal[2] * mz_cal_t

#     mx_cal.append(mx_cal_t)
#     my_cal.append(my_cal_t)
#     mz_cal.append(mz_cal_t)

#     print(mx_cal[N-1], my_cal[N-1], mz_cal[N-1], "count: ", N)
# file.close()

filename = "mag_read.txt"
for i in range(N):
    with open(filename, 'a', newline='') as f:
        writer = csv.writer(f, delimiter=' ')
        writer.writerow([mx[i], my[i], mz[i]])

fig_raw = plt.figure()
ax_raw = fig_raw.add_subplot(projection='3d')
ax_raw.set_xlim3d(-3000, 3000)
ax_raw.set_ylim3d(-3000, 3000)
ax_raw.set_zlim3d(-3000, 3000)
ub = 4000
lb = -4000
ax_raw.set_box_aspect([ub - lb for lb, ub in (getattr(ax_raw, f'get_{a}lim')() for a in 'xyz')])
# ax_raw.scatter(mx[0:int(N/3)],my[0:int(N/3)],mz[0:int(N/3)],color='red')
# ax_raw.scatter(mx[int(N/3):2*int(N/3)],my[int(N/3):2*int(N/3)],mz[int(N/3):2*int(N/3)],color='green')
# ax_raw.scatter(mx[2*int(N/3):N],my[2*int(N/3):N],mz[2*int(N/3):N],color='blue')
ax_raw.scatter(mx,my,mz,color='red')
ax_raw.scatter(mx_cal,my_cal,mz_cal,color='blue')

plt.show()



# filename = 'mag_read.csv'
# with open(filename, 'w', newline="") as file:
#     csvwriter = csv.writer(file) # 2. create a csvwriter object
#     for i in range(N):
#         csvwriter.writerow([mx[i], my[i], mz[i]])
        
    



### Dump

# def update_plot(frame):
#     plt.cla()
#     plt.plot(range(len(theta_gyro)), theta_gyro, label="theta_gyro")
#     plt.plot(range(len(theta_accel)), theta_accel, label="theta_accel")
#     plt.plot(range(len(theta_k)), theta_k, label="theta_k")
#     plt.xlabel('iteration  number')
#     plt.ylabel('theta in degrees')

# def on_close(event):
#     # nothing

# fig, ax = plt.subplots()
# fig.canvas.mp1_connect('close_event', on_close)

# ani = FuncAnimation(fig, update_plot, interval=1)

# plt.show()