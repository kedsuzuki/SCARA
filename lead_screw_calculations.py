'''
@file: lead_screw_calculations.py
@updated: 2024/06/17
@brief: Computes max raising and lowering torque for a lead screw assembly
@resource:  https://www.youtube.com/watch?v=BstZUC4tcOA
            https://www.youtube.com/watch?v=oAxYvoVXDrI&list=TLPQMTAwNjIwMjTrMRsCZJhThA&index=6 
            https://reprap.org/wiki/NEMA_17_Stepper_motor
@hardware: 42HB34F103AB Nema 17 42x42x34mm^3
'''

import math
# import csv
# import numpy as np
# from scipy.interpolate import interp1d
# import matplotlib.pyplot as plt

# Torque/Force Parameters **assuming metric trapezoidal lead screw**
pitch           = 2                     # lead screw pitch for T8 [mm]
starts          = 4                     # num of starts of lead screw 
l               = starts*pitch          # lead i.e., axial travel for one rotation [mm]
d_maj           = 8                     # major diameter [mm]
d_root          = 6.7                   # root diameter [mm]
d_m             = d_maj - 2*(d_root/4)  # mean diameter of lead screw [mm] 
alpha           = math.radians(15)      # lead angle of thread (metric trap ~ 30/2) [rad]
f               = 0.19                  # avg coefficient of friction for steel screw and brass nut (0.15-0.23) **if dry, f=0.5
g               = 9.81                  # acceleration due to gravity [m/s^2]
T_nemaMax       = 235                 # holding torque for a NEMA17-13 stepper motor [N-mm]
rpm_nemaMax     = 1_000                 # screw shaft revolution frequency per minute (rpm)
m_plat          = 20                    # mass of SCARA supported by lead screw platform [kg]
F_g             = m_plat*g              # force load on lead screw [N]

# max force that can be lifted or lowered given NEMA max torque
F_R = (2*T_nemaMax/d_m) * ((math.pi*d_m - f*l*(1/math.cos(alpha))) / (l + math.pi*f*d_m*(1/math.cos(alpha)))) 
F_L = (2*T_nemaMax/d_m) * ((math.pi*d_m + f*l*(1/math.cos(alpha))) / (f*math.pi*d_m*(1/math.cos(alpha)) - l)) 
print(f'mass_R: {F_R/g:.3f} kg')
print(f'mass_L: {F_L/g:.3f} kg\n')

# platform sliding speed, V [m/s]
V_plat = rpm_nemaMax*l/60 * math.pow(10,-3)   
print(f'V_plat: {V_plat:.3f} m/s\n')

# torque to raise and lower the load (assuming ACME profile threads)
T_R = (F_g*d_m/2) * ((l + f*math.pi*d_m*(1/math.cos(alpha))) / (math.pi*d_m - f*l*(1/math.cos(alpha)))) * math.pow(10, -3)
T_L = (F_g*d_m/2) * ((f*math.pi*d_m*(1/math.cos(alpha)) - l) / (math.pi*d_m + f*l*(1/math.cos(alpha)))) * math.pow(10, -3)
print(f'T_R: {T_R:.3f} N-m')
print(f'T_L: {T_L:.3f} N-m\n')

# torque checks
if math.pi*f*d_m <= l and T_L <= 0:
    print("No torque needed to lower the load")
if math.pi*f*d_m > l and T_L > 0:
    print("Self-locking")

# nema_torques = []
# nema_currents = []

# with open('motor_data.csv', 'r') as file:
#     reader = csv.reader(file)
#     next(reader)
#     for row in reader:
#         torque, current = float(row[0]), float(row[1])
#         nema_torques.append(torque)
#         nema_currents.append(current)

# interp_func = interp1d(nema_torques, nema_currents, kind='linear', fill_value='extrapolate')

# def get_current_for_torque(torque):
#     return interp_func(torque)

# I_R = get_current_for_torque(T_R)
# I_L = get_current_for_torque(T_L)
# print(f'I_R: {I_R:.3f} A')
# print(f'I_L: {I_L:.3f} A\n')