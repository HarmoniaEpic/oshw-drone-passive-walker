#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Drone-stabilized kneed passive walker — PyBullet dynamics + event-driven control.

Architecture: physics/control here (1200 Hz), Blender only visualizes the
recorded trajectory. Sensors: contact normal force (load cell), joint states
(encoders). Actuators: drone FC wrench on the torso (attitude/height/speed PD),
knee latches (position-hold motors, hip joints fully passive).
"""
import json, math, sys
import pybullet as p

# ---------------- model parameters (mirror of the Blender rig) ----------------
HIP_Z   = 0.837
L_TH    = 0.40          # hip -> knee
L_SH    = 0.315         # knee -> foot arc center
R_FOOT  = 0.12
FOOT_OFF= 0.04          # arc center forward offset (anti-buckling)
K_STOP  = 0.08          # recurved knee stop angle
M_TORSO, M_TH, M_SH = 1.2, 0.22, 0.20
DT      = 1.0/1200.0
T_END   = 8.0
V_SET   = 0.38

cid = p.connect(p.DIRECT)
p.setGravity(0, 0, -9.81)
p.setTimeStep(DT)

# ground
g_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[40, 2, 0.05])
ground = p.createMultiBody(0, g_col, basePosition=[35, 0, -0.05])
p.changeDynamics(ground, -1, lateralFriction=1.0, restitution=0.0)

# torso (base frame at hip axle center)
b_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.13, 0.15, 0.035],
                               collisionFramePosition=[0, 0, 0.06])
th_col = p.createCollisionShape(p.GEOM_CAPSULE, radius=0.021, height=0.34,
                                collisionFramePosition=[0, 0, -0.20])
quat_y = p.getQuaternionFromEuler([math.pi/2, 0, 0])   # cylinder axis -> Y
sh_col = p.createCollisionShapeArray(
    shapeTypes=[p.GEOM_CAPSULE, p.GEOM_CYLINDER],
    radii=[0.018, R_FOOT],
    lengths=[0.30, 0.08],
    halfExtents=[[0,0,0],[0,0,0]],
    collisionFramePositions=[[0, 0, -0.17], [FOOT_OFF, 0, -L_SH]],
    collisionFrameOrientations=[[0,0,0,1], quat_y])

walker = p.createMultiBody(
    baseMass=M_TORSO,
    baseCollisionShapeIndex=b_col,
    basePosition=[0, 0, HIP_Z],
    linkMasses=[M_TH, M_SH, M_TH, M_SH],
    linkCollisionShapeIndices=[th_col, sh_col, th_col, sh_col],
    linkVisualShapeIndices=[-1]*4,
    linkPositions=[[0,0,0], [0,0,-L_TH], [0,0,0], [0,0,-L_TH]],
    linkOrientations=[[0,0,0,1]]*4,
    linkInertialFramePositions=[[0,0,-0.20], [0,0,-0.18]]*2,
    linkInertialFrameOrientations=[[0,0,0,1]]*4,
    linkParentIndices=[0, 1, 0, 3],
    linkJointTypes=[p.JOINT_REVOLUTE]*4,
    linkJointAxis=[[0,1,0]]*4)

HIP_I, KNEE_I, HIP_O, KNEE_O = 0, 1, 2, 3
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

# kill default joint motors -> passive joints
for j in range(4):
    p.setJointMotorControl2(walker, j, p.VELOCITY_CONTROL, force=0)

# ---------------- launch: just-after-heel-strike, with momentum ----------------
p.resetBasePositionAndOrientation(walker, [0, 0, HIP_Z], [0, 0, 0, 1])
p.resetJointState(walker, HIP_I, -0.15, 0.53)   # stance: vault rate
p.resetJointState(walker, KNEE_I, K_STOP, 0)
p.resetJointState(walker, HIP_O, 0.30, -2.2)    # swing: mid-swing forward rate
p.resetJointState(walker, KNEE_O, -0.60, -1.0)
p.resetBaseVelocity(walker, [V_SET, 0, 0], [0, 0, 0])

# ---------------- controller ----------------
KP_R, KD_R   = 80.0, 10.0      # attitude PD (drone)
KZ  = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0   # height support gain
KV  = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0      # speed top-up gain
KDZ = KZ/10.0
Z_TGT = HIP_Z + (4.0/KZ if KZ > 0 else 0.0)
FX_MAX       = 8.0
KY, KDY      = 200.0, 20.0

HIP_EXT  = -0.10  # swing hip angle gate: extend knee once thigh is ahead
HIP_STRK = -0.08  # strike accepted only with foot genuinely ahead
T_EXT_BK = 0.60   # time backstop: extend anyway if pendulum falls short
T_TOE    = 0.18   # toe-off knee fold pulse duration
F_STRIKE = 8.0    # heel-strike force threshold [N]
T_MIN_SW = 0.20   # refractory: min swing time before strike accepted

stance, swing = 'I', 'O'
JH = {'I': HIP_I, 'O': HIP_O}; JK = {'I': KNEE_I, 'O': KNEE_O}
LS = {'I': SHANK_I, 'O': SHANK_O}
t_swing = 0.15    # outer already mid-swing at launch
strikes = []

def foot_force(leg):
    f = 0.0
    for c in p.getContactPoints(bodyA=walker, bodyB=ground, linkIndexA=LS[leg]):
        f += c[9]
    return f

def set_knee_modes(hip_sw):
    # stance knee: hard latch at the stop
    p.setJointMotorControl2(walker, JK[stance], p.POSITION_CONTROL,
                            targetPosition=K_STOP, force=300, maxVelocity=8)
    # toe-off fold pulse (minimal actuation, Delft-style): actively flex the
    # knee right after toe-off so the foot clears the parity graze
    if t_swing < T_TOE:
        p.setJointMotorControl2(walker, JK[swing], p.VELOCITY_CONTROL,
                                targetVelocity=-5.0, force=1.5)
    # swing knee: stay folded until the thigh is ahead (encoder gate),
    # then extend to prepare landing; time backstop for short pendulums
    elif hip_sw > HIP_EXT and t_swing < T_EXT_BK:
        p.setJointMotorControl2(walker, JK[swing], p.VELOCITY_CONTROL, force=0.02)
    else:
        p.setJointMotorControl2(walker, JK[swing], p.POSITION_CONTROL,
                                targetPosition=K_STOP, force=80, maxVelocity=10)
    # hips always passive
    p.setJointMotorControl2(walker, JH['I'], p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(walker, JH['O'], p.VELOCITY_CONTROL, force=0)

frames = []
fell = False
n_steps = int(T_END/DT)
for i in range(n_steps):
    t = i*DT
    pos, orn = p.getBasePositionAndOrientation(walker)
    vel, avel = p.getBaseVelocity(walker)

    # ---- drone flight controller (max assist) ----
    ax, ay, az = p.getEulerFromQuaternion(orn)
    tq = [-KP_R*ax - KD_R*avel[0],
          -KP_R*ay - KD_R*avel[1],
          -KP_R*az - KD_R*avel[2]]
    fx = max(-FX_MAX, min(FX_MAX, KV*(V_SET - vel[0])))
    fy = -KY*pos[1] - KDY*vel[1]
    fz = KZ*(Z_TGT - pos[2]) - KDZ*vel[2] if KZ > 0 else 0.0
    fz = max(0.0, min(30.0, fz))          # rotors cannot pull down
    p.applyExternalForce(walker, -1, [fx, fy, fz], pos, p.WORLD_FRAME)
    p.applyExternalTorque(walker, -1, tq, p.WORLD_FRAME)

    # ---- event-driven gait state machine ----
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

    if pos[2] < 0.45:
        fell = True
        break

    if i % 20 == 0:   # 60 Hz recording
        js = [p.getJointState(walker, j)[0] for j in range(4)]
        frames.append({
            't': round(t, 5),
            'base': [round(v, 5) for v in pos] + [round(v, 6) for v in orn],
            'joints': [round(v, 5) for v in js],
            'jvels': [round(p.getJointState(walker, j)[1], 4) for j in range(4)],
            'fI': round(foot_force('I'), 2),
            'fO': round(foot_force('O'), 2),
            'stance': stance})

pos, _ = p.getBasePositionAndOrientation(walker)
out = {
    'dt_record': 1.0/60.0,
    'fell': fell,
    't_end': frames[-1]['t'] if frames else 0,
    'distance': round(pos[0], 3),
    'n_strikes': len(strikes),
    'strike_times': [round(s, 3) for s in strikes],
    'frames': frames}
path = sys.argv[1] if len(sys.argv) > 1 else '/home/johnds/Claude/withBlenderMCP/walk_traj.json'
with open(path, 'w') as f:
    json.dump(out, f)

sp = 0.0
if len(strikes) >= 2:
    sp = out['distance']/out['t_end']
print(f"fell={fell}  t={out['t_end']:.2f}s  dist={out['distance']:.2f}m  "
      f"strikes={len(strikes)}  mean_speed={sp:.2f}m/s")
print("strike times:", out['strike_times'])
