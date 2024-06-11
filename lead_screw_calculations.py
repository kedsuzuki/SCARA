'''
@file: lead_screw_calculations.py
@updated: 2024/06/09
@brief: Computes max raising and lowering torque for a lead screw assembly
@resource:  https://www.youtube.com/watch?v=BstZUC4tcOA
            https://www.youtube.com/watch?v=oAxYvoVXDrI&list=TLPQMTAwNjIwMjTrMRsCZJhThA&index=6 
'''

import math

# torque/force parameters assuming metric trapezoidal lead screw
pitch           = 2                     # lead screw pitch for T8 [mm]
starts          = 4                     # num of starts of lead screw 
l               = starts*pitch          # lead [mm]
d_maj           = 8                     # major diameter [mm]
d_root          = 6.7                   # root diameter [mm]
d_m             = d_maj - 2*(d_root/4)  # mean diameter of lead screw [mm] 
alpha           = 15                    # lead angle of thread (metric trap ~ 30/2) [deg]
f               = 0.19                  # avg coefficient of friction for steel screw and brass nut (0.15-0.23) **if dry, f=0.5
m_plat          = 20                     # mass of SCARA supported by lead screw platform [kg]
g               = 9.81                  # acceleration due to gravity [m/s^2]
F_g             = m_plat*g              # force load on lead screw [N]
freq            = 200                   # screw shaft revolution frequency per minute (min^-2)
T_nemaMax       = 0.442                 # holding torque for a NEMA17-13 stepper motor (4.5 kg-m) [N-m]

# torque to raise and lower the load (assuming ACME profile threads)
T_R = (F_g*d_m/2) * ((l + f*math.pi*d_m*(1/math.cos(alpha))) / (math.pi*d_m - f*l*(1/math.cos(alpha)))) * math.pow(10, -3)
T_L = (F_g*d_m/2) * ((f*math.pi*d_m*(1/math.cos(alpha)) - l) / (math.pi*d_m + f*l*(1/math.cos(alpha)))) * math.pow(10, -3)
print(f'T_R: {T_R:.3f} N-m')
print(f'T_L: {T_L:.3f} N-m\n')

# max force that can be lifted or lowered
F_R = (2*T_nemaMax/d_m) * ((math.pi*d_m - f*l*(1/math.cos(alpha))) / (l + math.pi*f*d_m*(1/math.cos(alpha)))) * math.pow(10, 3)
F_L = (2*T_nemaMax/d_m) * ((math.pi*d_m + f*l*(1/math.cos(alpha))) / (f*math.pi*d_m*(1/math.cos(alpha)) - l)) * math.pow(10, 3)
print(f'mass_R: {F_R/g:.3f} kg')
print(f'mass_L: {F_L/g:.3f}kg\n')

# torque checks
if math.pi*f*d_m <= l and T_L <= 0:
    print("No torque needed to lower the load")

if math.pi*f*d_m > l and T_L > 0:
    print("Self-locking")

# platform sliding speed, V
V = (math.pi*d_m*freq)/math.cos(alpha) * math.pow(10, -3)