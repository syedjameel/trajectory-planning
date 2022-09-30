import numpy as np
from art import tprint
from matplotlib.pyplot import *
import time

j = []                  # [j1, j2, j3, j4, j5, .... so on till NUM_JOINTS]
j_copy = []
NUM_JOINTS = 6          # Number of joints

j1 = [0, 90, 5, 2]      # Joint 1 = [q0, qf, velocity_max, acceleration_max]
j2 = [0, 60, 5, 2]      # Joint 2 = [q0, qf, velocity_max, acceleration_max]
j3 = [0, 30, 5, 2]      # Joint 3 = [q0, qf, velocity_max, acceleration_max]
j4 = [0, 40, 5, 2]      # Joint 4 = [q0, qf, velocity_max, acceleration_max]
j5 = [0, 30, 5, 2]      # Joint 5 = [q0, qf, velocity_max, acceleration_max]
j6 = [0, 30, 5, 2]      # Joint 6 = [q0, qf, velocity_max, acceleration_max]

# Automatically adding all the j1, j2, j3 ... lists to j till NUM_JOINTS
num_joints = list()
for d in range(NUM_JOINTS):
    num_joints.append(str(int(d + 1)))
    te = tuple(num_joints)
for k, v in list(globals().items()):
    if k.startswith('j') and k.endswith(te):
        j.append(v.copy())
        j_copy.append(v.copy())
# j = [j1, j2, j3, j4, j5, j6]

dq = []         # qf - q0
vel_max = []    # sqrt(acc_max * dq)
times = []      # contains [tb, T, tf] for each joint

q = []          # Displacement curve
v = []          # Velocity curve
a = []          # Acceleration curve
bq = []         # Buffer Displacement curve
bv = []         # Buffer Velocity curve
ba = []         # Buffer Acceleration curve
time_intervals = []     # Time Intervals from 0 to tf for each Joint to move

Q0 = 0          # Index for q0
QF = 1          # Index for qf
VEL = 2         # Index for velocity max
ACC = 3         # Index for acceleration max
T_B = 4         # Index for tb
T_T = 5         # Index for T
T_F = 6         # Index for tf


def get_times_trajectory(j):
    dq = []
    vel_max = []
    times = []
    for index in range(len(j)):
        dq.append(j[index][QF]-j[index][Q0])
        vel_max.append(np.sqrt(j[index][ACC] * dq[index]))

    for index in range(len(vel_max)):
        if vel_max[index] <= j[index][VEL]:    # Triangular Profile
            tb = np.sqrt(dq[index] / j[index][ACC])
            T = tb
            tf = 2 * tb
        elif vel_max[index] > j[index][VEL]:      # Trapezoidal Profile
            tb = j[index][VEL] / j[index][ACC]      # tb = vel_max / acc_max
            T = dq[index] / j[index][VEL]         # T = tf-tb = (qf-q0) / vel
            tf = T + tb                         # tf = T + tb
        else:
            tb, T, tf = None, None, None
        j[index].append(tb)
        j[index].append(T)
        j[index].append(tf)
        times.append([tb, T, tf])

    for index in range(len(j)):
        print(f'For Joint {index+1} : tb = {j[index][4]:.2f},  tf-tb = {j[index][5]:.2f},  tf = {j[index][6]:.2f}')

    return j, times

def plan_trajectories(j):
    t0 = 0
    time_intervals.clear()
    time_stamp = 0
    q.clear()
    v.clear()
    a.clear()
    for index in range(len(j)):
        time_stamp = np.linspace(0, j[index][T_F], 3000)
        time_intervals.append(list(time_stamp))
        for t in time_stamp:
            if j[index][T_B] == j[index][T_T]:  # If it is Triangular Profile
                if t <= j[index][T_B]:
                    qi = j[index][Q0] + (0.5 * j[index][ACC] * (t-t0)**2)
                    q02 = qi
                    vi = j[index][ACC] * t
                    v02 = vi
                    ai = j[index][ACC]

                elif j[index][T_B] < t <= j[index][T_F]:
                    vi = j[index][ACC] * (j[index][T_F] - t)
                    qi = j[index][QF] - (0.5 * j[index][ACC] * (t - j[index][T_F]) ** 2)
                    ai = -j[index][ACC]
            else:                              # If it is Trapezoidal Profile
                if t <= j[index][T_B]:
                    qi = j[index][Q0] + (0.5 * j[index][ACC] * (t-t0)**2)
                    q02 = qi
                    vi = j[index][ACC] * t
                    v02 = vi
                    ai = j[index][ACC]

                elif j[index][T_B] < t <= j[index][T_T]:
                    vi = j[index][VEL]
                    qi = q02 + v02 * (t - j[index][T_B])
                    ai = 0

                elif t > j[index][T_T]:
                    vi = j[index][ACC] * (j[index][T_F] - t)
                    qi = j[index][QF] - (0.5 * j[index][ACC] * (t - j[index][T_F]) ** 2)
                    ai = -j[index][ACC]

            bq.append(qi)
            bv.append(vi)
            ba.append(ai)
        q.append(bq.copy())
        v.append(bv.copy())
        a.append(ba.copy())
        bq.clear()
        bv.clear()
        ba.clear()
    return time_intervals, q, v, a, j

def plot_trajectories(t, q, v, a, j):

    pack = [q, v, a]
    tf = []
    for index in range(len(j)):
        tf.append(j[index][T_F])

    count = 0
    figure(figsize=(26, 8))
    for index in range(len(j)):
        for i in range(len(pack)):
            count += 1
            if i == 0:
                label = r"q(t) ($\degree$)"
                ylimit = [0, max(pack[i][index]) + 10]
                slabel = "q"
                mlabel = r"$q_{2}$"
            elif i == 1:
                label = r"v(t) ($\degree$/s)"
                ylimit = [0, max(pack[i][index]) + 1]
                slabel = "v"
                mlabel = r"$v_{max}$"

            elif i == 2:
                label = r"a(t) ($\degree/s^2$)"
                ylimit = [min(pack[i][index])-1, max(pack[i][index])+1]
                slabel = "a"

            subplot(int(3*100+3*10+count))
            plot(t[index], pack[i][index], linewidth=2, label=slabel)  # pack[i][index]
            xlabel('t (s)', fontsize=18)
            ylabel(label, fontsize=18)
            grid(color='black', linestyle='--', linewidth=1.0, alpha=0.7)
            grid(True)
            xlim([0, max(tf)])
            ylim(ylimit)
            if i == 0:
                hlines(j[index][QF], 0, j[index][T_F], linestyles='--', color='r', label=mlabel)
                vlines([j[index][T_B], j[index][T_T]], 0, ylimit[1], linestyles='--', linewidth=2)
            elif i == 1:
                hlines(j[index][VEL], 0, j[index][T_F], linestyles='--', color='r', label=mlabel)
                vlines([j[index][T_B], j[index][T_T]], 0, ylimit[1], linestyles='--', linewidth=2)
            elif i == 2:
                vlines([j[index][T_B], j[index][T_T]], 0, ylimit, linestyles='--', linewidth=2)
            legend()
            if count == 9:
                show()
                count = 0
                figure(figsize=(26, 8))

    show()
    return None

def synchronize_trajectories(j):
    tb_all = []  # tb (Rise) time of all the joints
    T_all = []  # tf-tb time of all the joints
    tf_all = []  # tf (final) time of all the joints
    dwell_all = []  # tf-tb-tb (dwell) time of all the joints
    for index in range(len(j)):
        tb_all.append(j[index][T_B])
        T_all.append(j[index][T_T])
        tf_all.append(j[index][T_F])
        dwell_all.append(T_all[index] - tb_all[index])

    tb = max(tb_all)
    for index in range(len(tb_all)):
        print(f"Joint {index+1} rise time : {tb_all[index]}")
    print(f"Synchronized rise time : {tb}\n")

    dwell = max(dwell_all)
    for index in range(len(dwell_all)):
        print(f"Joint {index+1} dwell time : {dwell_all[index]}")
    print(f"Synchronized dwell time : {dwell}\n")

    T = dwell + tb
    tf = T + tb
    print(f"Synchronized trajectory time = {tb} + {dwell} + {tb} = {tf} sec\n")

    # Update the new tb, T, tf, vel_max, acc_max values to all the Joints in j variable
    for index in range(len(j)):
        j[index][T_B] = tb
        j[index][T_T] = T
        j[index][T_F] = tf
        j[index][VEL] = (j[index][QF] - j[index][Q0]) / T
        j[index][ACC] = j[index][VEL] / tb

    for i in range(len(j)):
        print(f"Joint {i+1} velocity modified from {j_copy[i][VEL]:.2f} to {j[i][VEL]:.2f} and acceleration from {j_copy[i][ACC]:.2f} to {j[i][ACC]:.2f}")

    return j

if __name__ == '__main__':

    tprint("Task   -   1")

    j, times = get_times_trajectory(j)
    time_intervals, q, v, a, j = plan_trajectories(j)
    plot_trajectories(time_intervals, q, v, a, j)

    tprint("Task   -   2")
    j = synchronize_trajectories(j)
    time_intervals, q, v, a, j = plan_trajectories(j)
    plot_trajectories(time_intervals, q, v, a, j)
