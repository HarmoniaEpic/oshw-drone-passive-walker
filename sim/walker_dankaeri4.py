#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Gen-4 v4: bladder OUT of the load path (cargo on the sprung plate,
pressure-neutral). Hydraulic transfer driven by the HEIGHT DIFFERENCE the
crank tilt creates between the feet (the crank is the siphon pump), plus
optional electric pump pressure. Crank is wound by a RIGID rack-freewheel
coupling: stance stroke q maps to crank angle through R_EFF = STROKE/pi
(full stroke = half turn), with energy-consistent reaction on the foot
spring. Detent sin(2*theta) quantizes the crank at 0/pi.

Usage: walker_dankaeri4.py out.json SLOPE_DEG PUMP_PA [K_C] [K_DET] [M_W]
"""
import json, math, sys
import pybullet as p

Z0, RC, YLEG = 0.84, 0.07, 0.14
H_LEG = Z0 - 0.015
STROKE = 0.05
R_EFF = STROKE/math.pi          # rack->crank ratio: full stroke = pi
M_BASE, M_CRANK = 0.20, 0.10
M_LEG, M_FOOT0 = 0.30, 0.35

SLOPE = math.radians(float(sys.argv[2])) if len(sys.argv) > 2 else 0.0
P_PUMP = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
K_C   = float(sys.argv[4]) if len(sys.argv) > 4 else 4.0    # coupling [N m/rad]
K_DET = float(sys.argv[5]) if len(sys.argv) > 5 else 0.40
M_W   = float(sys.argv[6]) if len(sys.argv) > 6 else 1.20

RHO = 1000.0
L_HOSE, D_HOSE = 0.5, 0.028
A_P = math.pi*D_HOSE**2/4
L_H = RHO*L_HOSE/A_P
K_H = (0.03*L_HOSE/D_HOSE + 3.0)*RHO/(2*A_P*A_P)
K_SPR, C_SPR = 700.0, 90.0
TAU_CAP = 0.5

DT, T_END = 1.0/2400.0, 30.0

cid = p.connect(p.DIRECT)
p.setGravity(9.81*math.sin(SLOPE), 0, -9.81*math.cos(SLOPE))
p.setTimeStep(DT)

g_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[60, 3, 0.05])
ground = p.createMultiBody(0, g_col, basePosition=[50, 0, -0.05])
p.changeDynamics(ground, -1, lateralFriction=0.9, restitution=0.0)

axle_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.02, height=0.18,
    collisionFrameOrientation=p.getQuaternionFromEuler([math.pi/2,0,0]))
crank_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[RC, 0.01, 0.02])
leg_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.07, 0.025, H_LEG/2-0.05],
    collisionFramePosition=[0, 0, -H_LEG/2])
# rocker feet: arc R with FORWARD OFFSET (gen-1 rule: GRF line passes ahead)
R_F, F_OFF = 0.30, 0.05
foot_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=R_F, height=0.26,
    collisionFramePosition=[F_OFF, 0, R_F - 0.015],
    collisionFrameOrientation=p.getQuaternionFromEuler([math.pi/2, 0, 0]))

w = [M_W*0.5, M_W*0.5]
walker = p.createMultiBody(
    baseMass=M_BASE,
    baseCollisionShapeIndex=axle_col,
    basePosition=[0, 0, Z0],
    linkMasses=[M_CRANK, M_LEG, M_FOOT0+w[0], M_LEG, M_FOOT0+w[1]],
    linkCollisionShapeIndices=[crank_col, leg_col, foot_col, leg_col, foot_col],
    linkVisualShapeIndices=[-1]*5,
    linkPositions=[[0,0,0],
                   [ RC, -YLEG, 0], [0, 0, -H_LEG],
                   [-RC,  YLEG, 0], [0, 0, -H_LEG]],
    linkOrientations=[[0,0,0,1]]*5,
    linkInertialFramePositions=[[0,0,0],
                                [0,0,-H_LEG*0.45], [0,0,0],
                                [0,0,-H_LEG*0.45], [0,0,0]],
    linkInertialFrameOrientations=[[0,0,0,1]]*5,
    linkParentIndices=[0, 1, 2, 1, 4],
    linkJointTypes=[p.JOINT_REVOLUTE,
                    p.JOINT_REVOLUTE, p.JOINT_PRISMATIC,
                    p.JOINT_REVOLUTE, p.JOINT_PRISMATIC],
    linkJointAxis=[[0,1,0], [0,1,0], [0,0,1], [0,1,0], [0,0,1]])

CRANK, HIP_L, FOOT_L, HIP_R, FOOT_R = 0, 1, 2, 3, 4
FEET = {0: FOOT_L, 1: FOOT_R}
for j in range(5):
    p.setJointMotorControl2(walker, j, p.VELOCITY_CONTROL, force=0)
for j in (FOOT_L, FOOT_R):
    p.changeDynamics(walker, j, jointLowerLimit=0.0, jointUpperLimit=STROKE,
                     jointLimitForce=3000, lateralFriction=0.9, restitution=0.0)
# hips: UNLIMITED continuous revolute (Ferris-wheel gondola bearings) —
# legs must stay vertical while the crank makes full revolutions
p.changeDynamics(walker, -1, linearDamping=0.02, angularDamping=0.05)
I_C = 0.002
p.changeDynamics(walker, CRANK, localInertiaDiagonal=[I_C, I_C, I_C])
p.setCollisionFilterGroupMask(walker, CRANK, 0, 0)

m_ref = {FOOT_L: M_FOOT0+w[0], FOOT_R: M_FOOT0+w[1]}
I_ref = {l: p.getDynamicsInfo(walker, l)[2] for l in (FOOT_L, FOOT_R)}

def set_water(i):
    l = FEET[i]
    m = M_FOOT0 + w[i]
    s = m/m_ref[l]
    p.changeDynamics(walker, l, mass=m,
                     localInertiaDiagonal=[c*s for c in I_ref[l]])

# start from REST at the detent well: symmetric, feet level, no transients.
# the pump loading the front bladder is the starter (real machine sequence)
TH0 = 0.0

msum, mz = M_BASE, M_BASE*Z0
for l in range(5):
    di = p.getDynamicsInfo(walker, l)
    msum += di[0]; mz += di[0]*p.getLinkState(walker, l)[0][2]
print(f"# m={msum:.2f} kg, h_com={mz/msum:.3f} m")

KY, KDY, KROT = 250.0, 25.0, 1.5
hip_lock = [False, False]      # load-engaged stance hip brake (with hysteresis)
q_h = 0.0
E_pump = E_drive = 0.0
th_hold = 0.0
# rack freewheel refs: tau_i = K_C*(q_i/R_EFF - (th - ref_i)), engaged in contact
ref = [TH0, TH0]
frames = []
fell = False

for i in range(int(T_END/DT)):
    t = i*DT
    pos, orn = p.getBasePositionAndOrientation(walker)
    vel, avel = p.getBaseVelocity(walker)

    p.applyExternalForce(walker, -1, [0, -KY*pos[1]-KDY*vel[1], 0], pos, p.WORLD_FRAME)
    p.applyExternalTorque(walker, -1, [-KROT*avel[0], 0, -KROT*avel[2]], p.WORLD_FRAME)

    th, om = p.getJointState(walker, CRANK)[0:2]

    # feet: spring + rack-coupling reaction; collect contact state
    tau_c = 0.0
    qs = [0.0, 0.0]
    contact = [False, False]
    for k, jf in ((0, FOOT_L), (1, FOOT_R)):
        qf, qd = p.getJointState(walker, jf)[0:2]
        qs[k] = qf
        cps = p.getContactPoints(walker, ground, linkIndexA=jf)
        contact[k] = len(cps) > 0
        Fs = K_SPR*qf + C_SPR*qd
        F_react = 0.0
        if contact[k] and t > 1.0:
            tau_i = K_C*(qf/R_EFF - (th - ref[k])) + 0.3*(qd/R_EFF - om)
            if tau_i > 0:
                tau_i = min(tau_i, TAU_CAP)
                tau_c += tau_i
                F_react = tau_i/R_EFF          # rack resists compression
            else:
                ref[k] = th - qf/R_EFF         # freewheel overrun: slip
        else:
            ref[k] = th - qf/R_EFF             # idle / re-arm
        p.setJointMotorControl2(walker, jf, p.TORQUE_CONTROL,
                                force=-(Fs + F_react))

    # stance hip brake: engage when the foot carries load, release when light.
    # (weight-activated clutch -- the dankaeri doll's 'rigid body during tumble')
    for k, jh, jf in ((0, HIP_L, FOOT_L), (1, HIP_R, FOOT_R)):
        cf = sum(c[9] for c in p.getContactPoints(walker, ground, linkIndexA=jf))
        if hip_lock[k]:
            if cf < 6.0:
                hip_lock[k] = False
        else:
            if cf > 14.0:
                hip_lock[k] = True
        if hip_lock[k]:
            p.setJointMotorControl2(walker, jh, p.VELOCITY_CONTROL,
                                    targetVelocity=0, force=30.0)
        else:
            p.setJointMotorControl2(walker, jh, p.VELOCITY_CONTROL, force=0)

    # hydraulics: crank-tilt height difference is the siphon (+ pump)
    pL = p.getLinkState(walker, FOOT_L)[0]
    pR = p.getLinkState(walker, FOOT_R)[0]
    hL = -pL[0]*math.sin(SLOPE) + pL[2]*math.cos(SLOPE)
    hR = -pR[0]*math.sin(SLOPE) + pR[2]*math.cos(SLOPE)
    dP = RHO*9.81*(hL-hR)
    if P_PUMP != 0.0 and t > 1.5:
        ramp = min(1.0, (t-1.5)/1.0)
        dP += P_PUMP*ramp*math.cos(th)   # crank-driven rotary valve commutation
        E_pump += abs(P_PUMP*ramp*math.cos(th)*q_h)*DT
    q_h += (dP - K_H*q_h*abs(q_h))/L_H*DT
    dm = RHO*q_h*DT
    dm = max(-w[1], min(w[0], dm))
    if abs(dm) > 0:
        w[0] -= dm; w[1] += dm
        set_water(0); set_water(1)
    if w[0] <= 1e-6 or w[1] <= 1e-6:
        q_h = 0.0

    # crank: coupling drive + detent + one-way catch
    # NOTE: escapement phase gating (release on forward-rock only) needs a
    # separate wind buffer + release latch; direct gating of tau_c starves
    # the winding (0 flips) -- see project log. Ungated for now.
    tau = tau_c - K_DET*math.sin(2*th) - 0.10*om
    E_drive += tau_c*om*DT
    th_hold = max(th_hold, th)
    if th < th_hold - 0.02:
        tau += 20.0*(th_hold - 0.02 - th) - 0.2*om
    p.setJointMotorControl2(walker, CRANK, p.TORQUE_CONTROL, force=tau)

    p.stepSimulation()

    if pos[2] < 0.55 or pos[2] > 1.1:
        fell = True
        break

    if i % 20 == 0:
        cf = [sum(c[9] for c in p.getContactPoints(walker, ground, linkIndexA=l))
              for l in (FOOT_L, FOOT_R)]
        frames.append({'t': round(t,4),
            'base': [round(v,5) for v in pos]+[round(v,6) for v in orn],
            'crank': round(th,4), 'om': round(om,3),
            'prism': [round(qs[0],4), round(qs[1],4)],
            'wL': round(w[0],4), 'wR': round(w[1],4),
            'fL': round(cf[0],1), 'fR': round(cf[1],1),
            'q': round(q_h*1000,4)})

th_end = p.getJointState(walker, CRANK)[0]
pos, _ = p.getBasePositionAndOrientation(walker)
out = {'slope_deg': math.degrees(SLOPE), 'pump_Pa': P_PUMP, 'K_C': K_C,
       'K_DET': K_DET, 'M_W': M_W,
       'fell': fell, 't_end': frames[-1]['t'] if frames else 0,
       'distance': round(pos[0],3), 'crank_rad': round(th_end,3),
       'n_flips': int(abs(th_end)//math.pi),
       'E_pump_J': round(E_pump,3), 'E_drive_J': round(E_drive,3),
       'dt_record': 1/60., 'frames': frames}
with open(sys.argv[1], 'w') as f:
    json.dump(out, f)
print(f"slope={math.degrees(SLOPE):4.1f} pump={P_PUMP:6.0f}Pa K_C={K_C:.0f} "
      f"K_DET={K_DET}  fell={fell} t={out['t_end']:.2f}s "
      f"dist={out['distance']:+.2f}m crank={th_end:+.2f}rad "
      f"flips={out['n_flips']} E_drv={E_drive:.2f}J E_pmp={E_pump:.2f}J")
