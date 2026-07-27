#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Gen-6 v1: twin-crank parallelogram legs (gear-locked attitude).

Gen-5's root failure was uncontrolled leg attitude (free pendulum hips).
Gen-6 guides each leg with TWO same-radius, same-phase cranks 0.20 m apart
fore-aft; a 3-gear train on each leg's inner face (pin gear - idler - pin
gear, ratio 1:1 same-sense) locks the parallelogram through its dead
center.  Legs therefore TRANSLATE: feet stay flat, stride = 2*RC exactly,
leg mass drops out of the crank balance, and the body stays upright -- the
dankaeri somersault is carried by the tank crank alone.

Model notes:
 - Front crank = Gen-5 crank (tanks, hub pump/valve) moved +0.10 m.
 - Rear Z-crank's CoM sits ON the rear axis at all angles (C2 pin pair +
   centered jog), so it is modeled as a fixed mass on the base at the rear
   axis; its rotation constraint is the virtual gear below.
 - Virtual gear: stiff PD on each hip joint enforcing q_hip = -theta
   (leg attitude == base attitude).  K_G ~ gear-train stiffness; the
   ~0.01 rad compliance plays the role of backlash.
 - Base origin stays at the FRONT axle (minimal diff vs Gen-5); the leg
   body and foot hang 0.10 m BEHIND the front pin (leg center between the
   two bearings).  Base pitch is a legitimate fall criterion again.

Naming: y<0 side = RIGHT half-body (walking dir +x, left hand = +y).

Usage: walker_gen6.py out.json SLOPE_DEG Q_PUMP [M_W] [OM_V] [T_END] [R_T]
       [D_EXTRA] [BRAKES(ignored)] [C_CRK] [X_TOE] [Q_LIM(ignored)]
       [FLIPLOCK(ignored)] [R_TOE] [FOOT_REV] [PHASECTL] [DZ_T] [RATCHET] [RC]
"""
import json, math, sys
import pybullet as p

Z0, RC, YLEG = 0.84, 0.07, 0.14
RC = float(sys.argv[19]) if len(sys.argv) > 19 else RC
H_LEG = Z0 - 0.015
STROKE = 0.05
D_AXLE = 0.20            # fore-aft crank spacing (leg bearing spacing)
X_LEG = -D_AXLE/2        # leg body center behind the front pin
M_BASE = 0.20            # front axle + bearing housing (non-rotating)
M_REAR = 0.35            # rear Z-crank + its 2 pin gears (CoM on rear axis)
M_CRANK = 0.65           # Gen-5 crank 0.45 + 2 front pin gears
M_LEG, M_FOOT0 = 0.38, 0.35   # leg 0.30 + idler gear + bosses
R_T = float(sys.argv[7]) if len(sys.argv) > 7 else 0.355
D_EXTRA = float(sys.argv[8]) if len(sys.argv) > 8 else 0.30
BRAKES = int(sys.argv[9]) if len(sys.argv) > 9 else 0      # ignored (no hip brakes)
Y_TANK = 0.44
DZ_T = float(sys.argv[17]) if len(sys.argv) > 17 else 0.290
X_TOE = float(sys.argv[11]) if len(sys.argv) > 11 else 0.08

SLOPE = math.radians(float(sys.argv[2])) if len(sys.argv) > 2 else 0.0
Q_PUMP = float(sys.argv[3]) if len(sys.argv) > 3 else 0.10   # kg/s
M_W   = float(sys.argv[4]) if len(sys.argv) > 4 else 1.20
OM_V  = float(sys.argv[5]) if len(sys.argv) > 5 else 0.30    # valve-freeze [rad/s]
T_END = float(sys.argv[6]) if len(sys.argv) > 6 else 40.0

K_SPR, C_SPR = 1500.0, 120.0
K_G, C_G = 300.0, 5.0    # virtual gear train (leg-attitude lock) PD
PHASECTL = int(sys.argv[16]) if len(sys.argv) > 16 else 0
RATCHET = int(sys.argv[18]) if len(sys.argv) > 18 else 1  # crank freewheel
SLACK = float(sys.argv[20]) if len(sys.argv) > 20 else 0.15   # catch backswing
HEEL_HX = float(sys.argv[21]) if len(sys.argv) > 21 else 0.10 # heel half-length
C_CRK = float(sys.argv[10]) if len(sys.argv) > 10 else 0.30
R_TOE = float(sys.argv[14]) if len(sys.argv) > 14 else 0.30
FOOT_REV = int(sys.argv[15]) if len(sys.argv) > 15 else 0
SPAWN_TH = -0.60
# Gen-6 kamae: legs contribute zero net crank torque (pair cancellation),
# so the free equilibrium is "full tank plumb", th = -atan2(R_T,DZ_T)
# = -0.885.  Spawning at -0.60 with the tight freewheel engaged parks the
# tank ~0.13 m BEHIND the plumb line: the one-way catch carries it, and
# the forward fall direction of the next step is thereby determined.
LOSS_HEAD = 0.10         # pump plumbing loss [m equivalent]
Q_DRAIN = 0.30           # valve-open gravity drain rate [kg/s]
PITCH_FALL = 0.90        # rad; body must stay upright in Gen-6

DT = 1.0/2400.0

cid = p.connect(p.DIRECT)
p.setGravity(9.81*math.sin(SLOPE), 0, -9.81*math.cos(SLOPE))
p.setTimeStep(DT)

g_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[60, 3, 0.05])
ground = p.createMultiBody(0, g_col, basePosition=[50, 0, -0.05])
p.changeDynamics(ground, -1, lateralFriction=2.0, restitution=0.0)

axle_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.02, height=0.18,
    collisionFrameOrientation=p.getQuaternionFromEuler([math.pi/2,0,0]))
crank_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[RC, 0.01, 0.02])
leg_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.07, 0.025, H_LEG/2-0.05],
    collisionFramePosition=[X_LEG, 0, -H_LEG/2])
quat_y = p.getQuaternionFromEuler([math.pi/2, 0, 0])
if FOOT_REV:
    box_x, arc_x = X_TOE + HEEL_HX, X_TOE
else:
    box_x, arc_x = X_TOE - HEEL_HX, X_TOE
foot_col = p.createCollisionShapeArray(
    shapeTypes=[p.GEOM_BOX, p.GEOM_CYLINDER],
    radii=[0, R_TOE],
    lengths=[0, 0.26],
    halfExtents=[[HEEL_HX, 0.13, 0.015], [0, 0, 0]],
    collisionFramePositions=[[box_x, 0, 0], [arc_x, 0, R_TOE - 0.015]],
    collisionFrameOrientations=[[0, 0, 0, 1], quat_y])
tank_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.08, 0.05, 0.05])

# water: w[0] = tank on y<0 side (front arm at spawn, HIGH: +DZ_T)
#        w[1] = tank on y>0 side (rear arm at spawn, LOW: -DZ_T)  <- full
w = [0.02, M_W - 0.02]
walker = p.createMultiBody(
    baseMass=M_BASE,
    baseCollisionShapeIndex=axle_col,
    basePosition=[0, 0, Z0 + RC*abs(math.sin(SPAWN_TH)) + 0.002],
    linkMasses=[M_CRANK, w[0], w[1], M_LEG, M_FOOT0, M_LEG, M_FOOT0, M_REAR],
    linkCollisionShapeIndices=[crank_col, tank_col, tank_col,
                               leg_col, foot_col, leg_col, foot_col, -1],
    linkVisualShapeIndices=[-1]*8,
    linkPositions=[[0,0,0],
                   [ R_T, -Y_TANK,  DZ_T],
                   [-R_T,  Y_TANK, -DZ_T],
                   [ RC, -YLEG, 0], [X_LEG, 0, -H_LEG],
                   [-RC,  YLEG, 0], [X_LEG, 0, -H_LEG],
                   [-D_AXLE, 0, 0]],
    linkOrientations=[[0,0,0,1]]*8,
    linkInertialFramePositions=[[0,0,0], [0,0,0], [0,0,0],
                                [X_LEG,0,-H_LEG*0.45], [0,0,0],
                                [X_LEG,0,-H_LEG*0.45], [0,0,0], [0,0,0]],
    linkInertialFrameOrientations=[[0,0,0,1]]*8,
    linkParentIndices=[0, 1, 1, 1, 4, 1, 6, 0],
    linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_FIXED, p.JOINT_FIXED,
                    p.JOINT_REVOLUTE, p.JOINT_PRISMATIC,
                    p.JOINT_REVOLUTE, p.JOINT_PRISMATIC, p.JOINT_FIXED],
    linkJointAxis=[[0,1,0], [0,0,1], [0,0,1],
                   [0,1,0], [0,0,1], [0,1,0], [0,0,1], [0,0,1]])

CRANK, TANK_R, TANK_L, HIP_R, FOOT_R, HIP_L, FOOT_L, REAR = range(8)
TANKS = {0: TANK_R, 1: TANK_L}
FEET = {0: FOOT_R, 1: FOOT_L}
HIPS = {0: HIP_R, 1: HIP_L}
for j in range(8):
    p.setJointMotorControl2(walker, j, p.VELOCITY_CONTROL, force=0)
for j in (FOOT_R, FOOT_L):
    p.changeDynamics(walker, j, jointLowerLimit=0.0, jointUpperLimit=STROKE,
                     jointLimitForce=3000, lateralFriction=2.0, restitution=0.0)
p.changeDynamics(walker, -1, linearDamping=0.02, angularDamping=0.05)
p.changeDynamics(walker, CRANK, localInertiaDiagonal=[0.03, 0.03, 0.03])
p.changeDynamics(walker, -1, localInertiaDiagonal=[0.01, 0.01, 0.01])
p.setCollisionFilterGroupMask(walker, CRANK, 0, 0)

m_ref = {TANKS[i]: max(w[i], 1e-3) for i in (0, 1)}
I_ref = {l: p.getDynamicsInfo(walker, l)[2] for l in (TANK_R, TANK_L)}

def set_water(i):
    l = TANKS[i]
    m = max(w[i], 1e-3)
    s = m/m_ref[l]
    p.changeDynamics(walker, l, mass=m,
                     localInertiaDiagonal=[max(c*s, 1e-7) for c in I_ref[l]])
set_water(0); set_water(1)
p.resetJointState(walker, CRANK, targetValue=SPAWN_TH)
p.resetJointState(walker, HIP_R, targetValue=-SPAWN_TH)   # legs vertical
p.resetJointState(walker, HIP_L, targetValue=-SPAWN_TH)

msum, mz = M_BASE, M_BASE*Z0
for l in range(8):
    di = p.getDynamicsInfo(walker, l)
    msum += di[0]; mz += di[0]*p.getLinkState(walker, l)[0][2]
print(f"# gen6 m={msum:.2f} kg, h_com={mz/msum:.3f} m, R_T={R_T}, dz={DZ_T}, "
      f"D_axle={D_AXLE}, RC={RC}")

KY, KDY, KROT = 250.0, 25.0, 1.5
C_ROCK = 0.05
E_pump = 0.0
cf_k = [0.0, 0.0]
mode = 'PUMP'
n_dose = 0
th_hold = SPAWN_TH
phase = 'SETTLE'
t_phase = 0.0
frames = []
fell = False
max_pitch = 0.0
max_gear_err = 0.0
valve_open_t = 0.0

for i in range(int(T_END/DT)):
    t = i*DT
    pos, orn = p.getBasePositionAndOrientation(walker)
    vel, avel = p.getBaseVelocity(walker)

    # 2D-ization: lateral spring + roll/yaw damping (as Gen-4/5)
    p.applyExternalForce(walker, -1, [0, -KY*pos[1]-KDY*vel[1], 0], pos, p.WORLD_FRAME)
    p.applyExternalTorque(walker, -1, [-KROT*avel[0], -C_ROCK*avel[1], -KROT*avel[2]], p.WORLD_FRAME)

    th, om = p.getJointState(walker, CRANK)[0:2]

    # foot springs (landing shock only)
    qs = [0.0, 0.0]
    for k in (0, 1):
        jf = FEET[k]
        qf, qd = p.getJointState(walker, jf)[0:2]
        qs[k] = qf
        p.setJointMotorControl2(walker, jf, p.TORQUE_CONTROL,
                                force=-(K_SPR*qf + C_SPR*qd))

    # virtual gear train: parallelogram attitude lock (leg attitude == base
    # attitude).  In the real mechanism the holding couple is a same-phase
    # pin-force pair whose front/rear crank torques CANCEL through the gear
    # mesh -- net crank torque is zero and the reaction routes to the BASE.
    # Model it as an external couple leg<->base, NOT as a hip-joint torque
    # (a hip torque would dump its reaction into the crank shaft: spurious
    # positive feedback, confirmed explosive).
    tau_gear = 0.0
    for k in (0, 1):
        cf_k[k] = sum(c[9] for c in p.getContactPoints(walker, ground, linkIndexA=FEET[k]))
        qh, oh = p.getJointState(walker, HIPS[k])[0:2]
        att = th + qh          # leg attitude relative to base (want 0)
        attd = om + oh
        max_gear_err = max(max_gear_err, abs(att))
        tau = -K_G*att - C_G*attd
        p.applyExternalTorque(walker, HIPS[k], [0, tau, 0], p.WORLD_FRAME)
        tau_gear -= tau
        p.setJointMotorControl2(walker, HIPS[k], p.TORQUE_CONTROL, force=-0.02*oh)
    p.applyExternalTorque(walker, -1, [0, tau_gear, 0], p.WORLD_FRAME)

    # crank freewheel (one-way catch).  Slack + catch damping absorb the
    # landing rebound; a tight hard catch slams the backswing momentum into
    # the base and topples it backward (confirmed at slack=0.05).
    tau_c = -C_CRK*om
    if RATCHET:
        th_hold = max(th_hold, th)
        if th < th_hold - SLACK:
            tau_c += 25.0*(th_hold - SLACK - th) - 3.0*om
    p.setJointMotorControl2(walker, CRANK, p.TORQUE_CONTROL, force=tau_c)

    # phase-synchronized transfer control (one-step state machine)
    if PHASECTL:
        if phase == 'SETTLE':
            if (t - t_phase > 0.8 and abs(om) < 0.15 and abs(avel[1]) < 0.20
                    and max(cf_k) > 10.0):
                phase = 'CHARGE'; t_phase = t
        elif phase == 'CHARGE':
            h = [0.0, 0.0]; x_t = [0.0, 0.0]
            for k in (0, 1):
                tp = p.getLinkState(walker, TANKS[k])[0]
                x_t[k] = tp[0]
                h[k] = -tp[0]*math.sin(SLOPE) + tp[2]*math.cos(SLOPE)
            fw = 0 if x_t[0] > x_t[1] else 1
            bk = 1 - fw
            uphill = h[fw] > h[bk]
            dm = min((Q_PUMP if uphill else Q_DRAIN)*DT, w[bk])
            if dm > 0:
                w[bk] -= dm; w[fw] += dm
                set_water(0); set_water(1)
                if uphill:
                    E_pump += dm*9.81*((h[fw]-h[bk]) + LOSS_HEAD)
            if abs(om) > 0.5:
                phase = 'FLIP'; t_phase = t
        else:  # FLIP: valve frozen until the crank half-turn lands and calms
            if (t - t_phase > 0.5 and min(cf_k) > 3.0 and abs(om) < 0.3):
                phase = 'SETTLE'; t_phase = t
                th_hold = th
                n_dose += 1
            elif t - t_phase > 8.0:
                phase = 'SETTLE'; t_phase = t
                th_hold = th
        mode = 'PHASE'
    else:
        mode = 'PUMP'
    if mode == 'PUMP':
        if t > 1.0 and abs(om) < OM_V:
            valve_open_t += DT
            h = [0.0, 0.0]
            for k in (0, 1):
                tp = p.getLinkState(walker, TANKS[k])[0]
                h[k] = -tp[0]*math.sin(SLOPE) + tp[2]*math.cos(SLOPE)
            x0 = p.getLinkState(walker, TANKS[0])[0][0]
            x1 = p.getLinkState(walker, TANKS[1])[0][0]
            fw = 0 if x0 > x1 else 1
            bk = 1 - fw
            uphill = h[fw] > h[bk]
            rate = Q_PUMP if uphill else Q_DRAIN
            dm = min(rate*DT, w[bk])
            if dm > 0:
                w[bk] -= dm; w[fw] += dm
                set_water(0); set_water(1)
                if uphill:
                    E_pump += dm*9.81*((h[fw]-h[bk]) + LOSS_HEAD)

    p.stepSimulation()

    eul = p.getEulerFromQuaternion(orn)
    max_pitch = max(max_pitch, abs(eul[1]))
    # Gen-6: the body must stay upright (legs are attitude-locked to it)
    if abs(eul[1]) > PITCH_FALL or pos[2] < 0.50 or pos[2] > 1.15:
        fell = True
        break

    if i % 40 == 0:
        cf = [sum(c[9] for c in p.getContactPoints(walker, ground, linkIndexA=l))
              for l in (FOOT_R, FOOT_L)]
        rec = {'t': round(t, 4),
               'base': [round(v, 5) for v in pos]+[round(v, 6) for v in orn],
               'crank': round(th, 4), 'om': round(om, 3),
               'prism': [round(qs[0], 4), round(qs[1], 4)],
               'wR': round(w[0], 4), 'wL': round(w[1], 4),
               'fR': round(cf[0], 1), 'fL': round(cf[1], 1)}
        for key, lnk in (('legR', HIP_R), ('legL', HIP_L),
                         ('ftR', FOOT_R), ('ftL', FOOT_L),
                         ('tankR', TANK_R), ('tankL', TANK_L)):
            ls = p.getLinkState(walker, lnk)
            rec[key] = ([round(v, 5) for v in ls[4]]
                        + [round(v, 6) for v in ls[5]])
        frames.append(rec)

th_end = p.getJointState(walker, CRANK)[0]
pos, _ = p.getBasePositionAndOrientation(walker)
out = {'slope_deg': math.degrees(SLOPE), 'Q_pump': Q_PUMP, 'M_W': M_W,
       'OM_V': OM_V, 'fell': fell,
       't_end': frames[-1]['t'] if frames else 0,
       'distance': round(pos[0], 3), 'crank_rad': round(th_end, 3),
       'n_flips': int(abs(th_end)//math.pi),
       'max_pitch_deg': round(math.degrees(max_pitch), 1),
       'n_dose': n_dose, 'R_T': R_T, 'D_EXTRA': D_EXTRA,
       'E_pump_J': round(E_pump, 3),
       'gear_err_max_deg': round(math.degrees(max_gear_err), 2),
       'dt_record': 1/60.,
       'frames': frames}
with open(sys.argv[1], 'w') as f:
    json.dump(out, f)
print(f"slope={math.degrees(SLOPE):4.1f} Q={Q_PUMP:.3f}kg/s M_W={M_W}  "
      f"fell={fell} t={out['t_end']:.2f}s dist={out['distance']:+.3f}m "
      f"crank={th_end:+.2f}rad flips={out['n_flips']} "
      f"pitch_max={out['max_pitch_deg']}deg gear_err={out['gear_err_max_deg']}deg "
      f"E_pump={E_pump:.2f}J")
