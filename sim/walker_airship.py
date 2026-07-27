#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Airship-hulled kneed passive walker: prolate helium hull with cruciform tail
(pitch/yaw fin damping), a pair of azimuth ducted fans, passive attitude.

"A non-flying airship with legs": helium buoyancy above the CoM provides
PASSIVE attitude restoring (metacentric / upright-pendulum stabilization)
and static weight offload; the two side fans supply only horizontal thrust
(the energy top-up that replaces a slope). No active attitude control.

Usage: walker_balloon.py out.json BUOYANCY_N [KV]
"""
import json, math, sys
import pybullet as p

# ---------------- model parameters ----------------
HIP_Z   = 0.837
L_TH    = 0.40
L_SH    = 0.315
R_FOOT  = 0.12
FOOT_OFF= 0.04
K_STOP  = 0.08
M_TORSO = 1.40          # platform + axle + fan hardware
M_BAL   = 0.30          # airship envelope + fins + mast (lumped at CB)
M_TH, M_SH = 0.22, 0.20
BAL_H   = 1.55          # balloon attach height above hip
DT      = 1.0/1200.0
T_END   = 8.0
V_SET   = 0.38

B_UP = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0   # buoyancy [N]
KV   = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0

cid = p.connect(p.DIRECT)
p.setGravity(0, 0, -9.81)
p.setTimeStep(DT)

g_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[40, 2, 0.05])
ground = p.createMultiBody(0, g_col, basePosition=[35, 0, -0.05])
p.changeDynamics(ground, -1, lateralFriction=1.0, restitution=0.0)

b_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.13, 0.15, 0.035],
                               collisionFramePosition=[0, 0, 0.06])
th_col = p.createCollisionShape(p.GEOM_CAPSULE, radius=0.021, height=0.34,
                                collisionFramePosition=[0, 0, -0.20])
quat_y = p.getQuaternionFromEuler([math.pi/2, 0, 0])
sh_col = p.createCollisionShapeArray(
    shapeTypes=[p.GEOM_CAPSULE, p.GEOM_CYLINDER],
    radii=[0.018, R_FOOT],
    lengths=[0.30, 0.08],
    halfExtents=[[0,0,0],[0,0,0]],
    collisionFramePositions=[[0, 0, -0.17], [FOOT_OFF, 0, -L_SH]],
    collisionFrameOrientations=[[0,0,0,1], quat_y])
bal_col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.05)

walker = p.createMultiBody(
    baseMass=M_TORSO,
    baseCollisionShapeIndex=b_col,
    basePosition=[0, 0, HIP_Z],
    linkMasses=[M_TH, M_SH, M_TH, M_SH, M_BAL],
    linkCollisionShapeIndices=[th_col, sh_col, th_col, sh_col, bal_col],
    linkVisualShapeIndices=[-1]*5,
    linkPositions=[[0,0,0], [0,0,-L_TH], [0,0,0], [0,0,-L_TH], [0,0,BAL_H]],
    linkOrientations=[[0,0,0,1]]*5,
    linkInertialFramePositions=[[0,0,-0.20], [0,0,-0.18]]*2 + [[0,0,0]],
    linkInertialFrameOrientations=[[0,0,0,1]]*5,
    linkParentIndices=[0, 1, 0, 3, 0],
    linkJointTypes=[p.JOINT_REVOLUTE]*4 + [p.JOINT_FIXED],
    linkJointAxis=[[0,1,0]]*4 + [[0,0,1]])

HIP_I, KNEE_I, HIP_O, KNEE_O, BAL = 0, 1, 2, 3, 4
SHANK_I, SHANK_O = 1, 3

for j in (HIP_I, HIP_O):
    p.changeDynamics(walker, j, jointLowerLimit=-1.5, jointUpperLimit=1.5,
                     jointLimitForce=500)
for j in (KNEE_I, KNEE_O):
    p.changeDynamics(walker, j, jointLowerLimit=-2.2, jointUpperLimit=K_STOP,
                     jointLimitForce=500)
for l in (SHANK_I, SHANK_O):
    p.changeDynamics(walker, l, lateralFriction=0.6, restitution=0.0)
p.changeDynamics(walker, -1, linearDamping=0.02, angularDamping=0.05)
p.setCollisionFilterGroupMask(walker, BAL, 0, 0)   # hull: no collisions
# prolate hull rotational inertia (L=2.2 m): pitch/yaw >> roll
p.changeDynamics(walker, BAL, localInertiaDiagonal=[0.02, 0.07, 0.07])

for j in range(4):
    p.setJointMotorControl2(walker, j, p.VELOCITY_CONTROL, force=0)

# ---------------- launch: mid-gait with momentum ----------------
p.resetBasePositionAndOrientation(walker, [0, 0, HIP_Z], [0, 0, 0, 1])
p.resetJointState(walker, HIP_I, -0.15, 0.53)
p.resetJointState(walker, KNEE_I, K_STOP, 0)
p.resetJointState(walker, HIP_O, 0.30, -2.2)
p.resetJointState(walker, KNEE_O, -0.60, -1.0)
p.resetBaseVelocity(walker, [V_SET, 0, 0], [0, 0, 0])

# ---------------- controller ----------------
FX_MAX  = 8.0            # pair total
KY, KDY = 200.0, 20.0    # lateral surrogate for the 4-leg frame
KYAW    = 2.0            # yaw damping surrogate
KD_BAL  = 0.80           # hull translational drag [N s/m]
C_FIN   = 0.50           # tail fin rotational damping [N m s/rad]
FAN_Y   = 0.36           # fan outrigger offset

HIP_EXT  = -0.10
HIP_STRK = -0.08
T_EXT_BK = 0.60
T_TOE    = 0.18
F_STRIKE = 8.0
T_MIN_SW = 0.20

stance, swing = 'I', 'O'
JH = {'I': HIP_I, 'O': HIP_O}; JK = {'I': KNEE_I, 'O': KNEE_O}
LS = {'I': SHANK_I, 'O': SHANK_O}
t_swing = 0.15
strikes = []

def foot_force(leg):
    f = 0.0
    for c in p.getContactPoints(bodyA=walker, bodyB=ground, linkIndexA=LS[leg]):
        f += c[9]
    return f

def set_knee_modes(hip_sw):
    p.setJointMotorControl2(walker, JK[stance], p.POSITION_CONTROL,
                            targetPosition=K_STOP, force=300, maxVelocity=8)
    if t_swing < T_TOE:
        p.setJointMotorControl2(walker, JK[swing], p.VELOCITY_CONTROL,
                                targetVelocity=-5.0, force=1.5)
    elif hip_sw > HIP_EXT and t_swing < T_EXT_BK:
        p.setJointMotorControl2(walker, JK[swing], p.VELOCITY_CONTROL, force=0.02)
    else:
        p.setJointMotorControl2(walker, JK[swing], p.POSITION_CONTROL,
                                targetPosition=K_STOP, force=80, maxVelocity=10)
    p.setJointMotorControl2(walker, JH['I'], p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(walker, JH['O'], p.VELOCITY_CONTROL, force=0)

frames = []
fell = False
max_pitch = 0.0
fan_abs = 0.0
n_steps = int(T_END/DT)
for i in range(n_steps):
    t = i*DT
    pos, orn = p.getBasePositionAndOrientation(walker)
    vel, avel = p.getBaseVelocity(walker)
    ax, ay, az = p.getEulerFromQuaternion(orn)
    max_pitch = max(max_pitch, abs(ay))

    # ---- balloon: buoyancy + air drag at the attach point (passive attitude) ----
    ls = p.getLinkState(walker, BAL, computeLinkVelocity=1)
    bal_pos, bal_vel = ls[0], ls[6]
    p.applyExternalForce(walker, BAL,
                         [-KD_BAL*bal_vel[0], -KD_BAL*bal_vel[1],
                          B_UP - KD_BAL*bal_vel[2]],
                         bal_pos, p.WORLD_FRAME)

    # ---- azimuth fan pair: horizontal thrust only (energy top-up) ----
    fx = max(-FX_MAX, min(FX_MAX, KV*(V_SET - vel[0])))
    fy = -KY*pos[1] - KDY*vel[1]
    rot = p.getMatrixFromQuaternion(orn)
    for s in (1, -1):
        fp = [pos[0] + rot[1]*s*FAN_Y, pos[1] + rot[4]*s*FAN_Y,
              pos[2] + rot[7]*s*FAN_Y]
        p.applyExternalForce(walker, -1, [fx/2, fy/2, 0.0], fp, p.WORLD_FRAME)
    p.applyExternalTorque(walker, -1, [0, 0, -KYAW*avel[2]], p.WORLD_FRAME)
    # tail fins: aerodynamic damping of pitch/yaw oscillation
    p.applyExternalTorque(walker, -1, [0, -C_FIN*avel[1], -C_FIN*avel[2]],
                          p.WORLD_FRAME)
    fan_abs += abs(fx)

    # ---- event-driven gait ----
    t_swing += DT
    f_sw = foot_force(swing)
    hip_sw = p.getJointState(walker, JH[swing])[0]
    if t_swing > T_MIN_SW and f_sw > F_STRIKE and hip_sw < HIP_STRK:
        strikes.append(t)
        stance, swing = swing, stance
        t_swing = 0.0
        hip_sw = p.getJointState(walker, JH[swing])[0]
    set_knee_modes(hip_sw)

    p.stepSimulation()

    if pos[2] < 0.45 or pos[2] > 1.4:   # fell or floated away
        fell = True
        break

    if i % 20 == 0:
        js = [p.getJointState(walker, j)[0] for j in range(4)]
        frames.append({
            't': round(t, 5),
            'base': [round(v, 5) for v in pos] + [round(v, 6) for v in orn],
            'joints': [round(v, 5) for v in js],
            'fI': round(foot_force('I'), 2),
            'fO': round(foot_force('O'), 2),
            'stance': stance})

pos, _ = p.getBasePositionAndOrientation(walker)
W_tot = (M_TORSO + M_BAL + 2*(M_TH+M_SH))*9.81
out = {
    'dt_record': 1.0/60.0, 'buoyancy_N': B_UP, 'weight_N': round(W_tot,2),
    'fell': fell, 't_end': frames[-1]['t'] if frames else 0,
    'distance': round(pos[0], 3),
    'n_strikes': len(strikes),
    'strike_times': [round(s, 3) for s in strikes],
    'max_pitch_deg': round(math.degrees(max_pitch), 1),
    'mean_fan_N': round(fan_abs/max(1, n_steps), 2),
    'frames': frames}
with open(sys.argv[1], 'w') as f:
    json.dump(out, f)

print(f"B={B_UP:4.1f}N ({100*B_UP/W_tot:4.0f}%W)  fell={fell}  t={out['t_end']:.2f}s  "
      f"dist={out['distance']:.2f}m  strikes={len(strikes)}  "
      f"max_pitch={out['max_pitch_deg']}deg  fan={out['mean_fan_N']}N")
