#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Gen-5 v1: crank-integrated water transfer (dankaeri wheel).

The two water tanks ride the CRANK itself (radius R_T, neck offset delta
ahead of each arm, C2-symmetric).  The crank joint is fully PASSIVE -- no
torque control, no detent, no rack/bellows.  The only actuation is mass
transfer between the tanks by the hub pump; the hub valve is modeled as
"transfer frozen while the crank is turning fast" (|om| > OM_V), i.e. the
doll's mercury-lag principle.

Cycle: dwell (full tank low) -> pump lifts water to the high/front tank ->
net crank torque flips sign -> rear pin rises, rear foot lifts clear (no
suriashi) -> gravity rolls the crank +pi, swing leg lands 2*RC ahead ->
tanks have swapped roles (C2) -> next dwell.

Naming: y<0 side = RIGHT half-body (walking dir +x, left hand = +y).

Usage: walker_gen5.py out.json SLOPE_DEG Q_PUMP [M_W] [OM_V] [T_END]
       Q_PUMP in kg/s (0.15 = Gen-4 pump class)
"""
import json, math, sys
import pybullet as p

Z0, RC, YLEG = 0.84, 0.07, 0.14
H_LEG = Z0 - 0.015
STROKE = 0.05
M_BASE = 0.20            # axle + bearing housing (non-rotating)
M_CRANK = 0.45           # arms + bars + necks + tank shells + hub pump/valve
M_LEG, M_FOOT0 = 0.30, 0.35
R_T = float(sys.argv[7]) if len(sys.argv) > 7 else 0.355  # tank x-offset (model)
D_EXTRA = float(sys.argv[8]) if len(sys.argv) > 8 else 0.30   # dose target dW [kg]
BRAKES = int(sys.argv[9]) if len(sys.argv) > 9 else 1     # hip brakes on/off
Y_TANK = 0.44
DZ_T = 0.290               # tank z-offset: mass line 39 deg ahead of pin line
X_TOE = float(sys.argv[11]) if len(sys.argv) > 11 else 0.08
                           # whole rocker shifted FORWARD on the leg: puts the
                           # stance tangent under the axle (= under the water
                           # hanging at settle). Kamae balance requirement.

SLOPE = math.radians(float(sys.argv[2])) if len(sys.argv) > 2 else 0.0
Q_PUMP = float(sys.argv[3]) if len(sys.argv) > 3 else 0.10   # kg/s
M_W   = float(sys.argv[4]) if len(sys.argv) > 4 else 1.20
OM_V  = float(sys.argv[5]) if len(sys.argv) > 5 else 0.30    # valve-freeze [rad/s]
T_END = float(sys.argv[6]) if len(sys.argv) > 6 else 40.0

K_SPR, C_SPR = 1500.0, 120.0   # stiffened: Gen-5 single-leg kamae loads
C_HIP = 0.02             # gondola bearing drag
Q_LIM = float(sys.argv[12]) if len(sys.argv) > 12 else 0.0
                         # hip cord limit [rad]: leg-vs-crank one-way stop
FLIPLOCK = int(sys.argv[13]) if len(sys.argv) > 13 else 1
K_LIM, C_LIM = 30.0, 1.0 # cord stiffness/damping (doll's taut string)
C_CRK = float(sys.argv[10]) if len(sys.argv) > 10 else 0.30
                         # crank drag incl. in-tank slosh dissipation
SPAWN_TH = -0.60         # spawn at the L-full settled angle (kamae)
LOSS_HEAD = 0.10         # pump plumbing loss [m equivalent]
Q_DRAIN = 0.30           # valve-open gravity drain rate [kg/s]

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
    collisionFramePosition=[0, 0, -H_LEG/2])
# half-kamaboko foot (Gen-4 proven): flat heel + tangent toe arc.
# FOOT_REV=1 mirrors it: big arc at the HEEL (backward catcher, R>h_com)
# + flat edge at the TOE (forward ratchet) -- reversed rectifier.
R_TOE = float(sys.argv[14]) if len(sys.argv) > 14 else 0.30
FOOT_REV = int(sys.argv[15]) if len(sys.argv) > 15 else 0
quat_y = p.getQuaternionFromEuler([math.pi/2, 0, 0])
if FOOT_REV:
    box_x, arc_x = X_TOE + 0.10, X_TOE
else:
    box_x, arc_x = X_TOE - 0.10, X_TOE
foot_col = p.createCollisionShapeArray(
    shapeTypes=[p.GEOM_BOX, p.GEOM_CYLINDER],
    radii=[0, R_TOE],
    lengths=[0, 0.26],
    halfExtents=[[0.10, 0.13, 0.015], [0, 0, 0]],
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
    linkMasses=[M_CRANK, w[0], w[1], M_LEG, M_FOOT0, M_LEG, M_FOOT0],
    linkCollisionShapeIndices=[crank_col, tank_col, tank_col,
                               leg_col, foot_col, leg_col, foot_col],
    linkVisualShapeIndices=[-1]*7,
    linkPositions=[[0,0,0],
                   [ R_T, -Y_TANK,  DZ_T],
                   [-R_T,  Y_TANK, -DZ_T],
                   [ RC, -YLEG, 0], [0, 0, -H_LEG],
                   [-RC,  YLEG, 0], [0, 0, -H_LEG]],
    linkOrientations=[[0,0,0,1]]*7,
    linkInertialFramePositions=[[0,0,0], [0,0,0], [0,0,0],
                                [0,0,-H_LEG*0.45], [0,0,0],
                                [0,0,-H_LEG*0.45], [0,0,0]],
    linkInertialFrameOrientations=[[0,0,0,1]]*7,
    linkParentIndices=[0, 1, 1, 1, 4, 1, 6],
    linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_FIXED, p.JOINT_FIXED,
                    p.JOINT_REVOLUTE, p.JOINT_PRISMATIC,
                    p.JOINT_REVOLUTE, p.JOINT_PRISMATIC],
    linkJointAxis=[[0,1,0], [0,0,1], [0,0,1],
                   [0,1,0], [0,0,1], [0,1,0], [0,0,1]])

CRANK, TANK_R, TANK_L, HIP_R, FOOT_R, HIP_L, FOOT_L = 0, 1, 2, 3, 4, 5, 6
TANKS = {0: TANK_R, 1: TANK_L}
FEET = {0: FOOT_R, 1: FOOT_L}
HIPS = {0: HIP_R, 1: HIP_L}
for j in range(7):
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
p.resetJointState(walker, HIP_R, targetValue=-SPAWN_TH)   # legs hang vertical
p.resetJointState(walker, HIP_L, targetValue=-SPAWN_TH)

msum, mz = M_BASE, M_BASE*Z0
for l in range(7):
    di = p.getDynamicsInfo(walker, l)
    msum += di[0]; mz += di[0]*p.getLinkState(walker, l)[0][2]
print(f"# m={msum:.2f} kg, h_com={mz/msum:.3f} m, R_T={R_T}, dz={DZ_T}")

KY, KDY, KROT = 250.0, 25.0, 1.5
C_ROCK = 0.05   # base-pitch drag: keep tiny -- the somersault IS base pitch
E_pump = 0.0
hip_lock = [False, False]
cf_k = [0.0, 0.0]
mode = 'PUMP'
dose_left = 0.0
t_roll = -9.0
n_dose = 0
flip_lock = False
th_hold = SPAWN_TH
frames = []
fell = False
max_pitch = 0.0
valve_open_t = 0.0

for i in range(int(T_END/DT)):
    t = i*DT
    pos, orn = p.getBasePositionAndOrientation(walker)
    vel, avel = p.getBaseVelocity(walker)

    # 2D-ization: lateral spring + roll/yaw damping (as Gen-4)
    p.applyExternalForce(walker, -1, [0, -KY*pos[1]-KDY*vel[1], 0], pos, p.WORLD_FRAME)
    p.applyExternalTorque(walker, -1, [-KROT*avel[0], -C_ROCK*avel[1], -KROT*avel[2]], p.WORLD_FRAME)

    th, om = p.getJointState(walker, CRANK)[0:2]

    # foot springs (landing shock only -- not a geometry absorber)
    qs = [0.0, 0.0]
    for k in (0, 1):
        jf = FEET[k]
        qf, qd = p.getJointState(walker, jf)[0:2]
        qs[k] = qf
        p.setJointMotorControl2(walker, jf, p.TORQUE_CONTROL,
                                force=-(K_SPR*qf + C_SPR*qd))

    # hips: load-engaged stance brake, released for the FORWARD roll only
    # (dankaeri order: rigid frame while dwelling/back-settling, free
    #  gondolas while the crank rolls forward over the stance leg)
    for k in (0, 1):
        cf_k[k] = sum(c[9] for c in p.getContactPoints(walker, ground, linkIndexA=FEET[k]))
        if hip_lock[k]:
            if cf_k[k] < 4.0:
                hip_lock[k] = False
        else:
            if cf_k[k] > 10.0:
                hip_lock[k] = True
    # dankaeri rigidity: "rigid while somersaulting" -- when one foot is
    # airborne AND the assembly is pitching fast, the hip cords go taut and
    # the whole machine vaults as one rigid body, carrying the raised swing
    # leg over like a spoke to land ahead.  Free again once grounded/slow.
    if flip_lock:
        if min(cf_k) > 5.0 or abs(avel[1]) < 0.2:
            flip_lock = False
    else:
        if FLIPLOCK and min(cf_k) < 1.0 and abs(avel[1]) > 0.5:
            flip_lock = True
    for k in (0, 1):
        if flip_lock or (BRAKES and mode != 'ROLL' and hip_lock[k]):
            p.setJointMotorControl2(walker, HIPS[k], p.VELOCITY_CONTROL,
                                    targetVelocity=0, force=50.0)
        else:
            qh, oh = p.getJointState(walker, HIPS[k])[0:2]
            tau_h = -C_HIP*oh
            # one-way hip cord (dankaeri string): free swing until the
            # assembly leans Q_LIM past the hanging leg, then taut
            if qh < -Q_LIM:
                tau_h += K_LIM*(-Q_LIM - qh) - C_LIM*oh
            p.setJointMotorControl2(walker, HIPS[k], p.TORQUE_CONTROL, force=tau_h)
    # crank freewheel: one-way position catch (Gen-4 proven pattern).
    # Allows small backswing, blocks real unwinding of the walk phase.
    th_hold = max(th_hold, th)
    tau_c = -C_CRK*om
    if th < th_hold - 0.35:
        tau_c += 25.0*(th_hold - 0.35 - th) - 0.5*om
    p.setJointMotorControl2(walker, CRANK, p.TORQUE_CONTROL, force=tau_c)

    # hub pump + valve.  BRAKES=0 (passive gondola machine): no mode logic,
    # valve is simply frozen while the crank turns fast (|om|>OM_V).
    # BRAKES=1 keeps the dose/release state machine for the rigid-frame mode.
    if not BRAKES:
        mode = 'PUMP'
    if mode == 'PUMP':
        if t > 1.0 and abs(om) < OM_V:
            valve_open_t += DT
            h = [0.0, 0.0]
            for k in (0, 1):
                tp = p.getLinkState(walker, TANKS[k])[0]
                h[k] = -tp[0]*math.sin(SLOPE) + tp[2]*math.cos(SLOPE)
            # direction rule: ALWAYS move water toward the FRONT tank
            # (walking direction), regardless of height.  Uphill costs pump
            # work; downhill is a free valve-open drain.
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
            if BRAKES and (w[hi] - w[lo]) >= D_EXTRA and t > 2.0:
                mode = 'ROLL'
                t_roll = t
                th_roll0 = th
                n_dose += 1
        if BRAKES and mode == 'PUMP' and om > 0.12 and t > 2.0:
            mode = 'ROLL'          # crank creep = physical liftoff: release
            t_roll = t
            th_roll0 = th
            n_dose += 1
    else:  # ROLL: hips free, valve frozen; re-latch only after a real half-turn
        if (t - t_roll > 0.5 and th - th_roll0 > 1.0
                and cf_k[0] > 10.0 and cf_k[1] > 10.0 and abs(om) < 0.30):
            mode = 'PUMP'
        elif t - t_roll > 6.0:
            mode = 'PUMP'

    p.stepSimulation()

    # world-frame health checks (Gen-4 lesson: verify in world coords)
    eul = p.getEulerFromQuaternion(orn)
    max_pitch = max(max_pitch, abs(eul[1]))
    # base pitch is NOT a fall: the somersault happens about the hip
    # gondolas, so the whole assembly (incl. base) flips each step.
    if pos[2] < 0.30 or pos[2] > 1.15:
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
       'dt_record': 60.0/2400.0*40/40 and 1/60.,
       'frames': frames}
with open(sys.argv[1], 'w') as f:
    json.dump(out, f)
print(f"slope={math.degrees(SLOPE):4.1f} Q={Q_PUMP:.3f}kg/s M_W={M_W}  "
      f"fell={fell} t={out['t_end']:.2f}s dist={out['distance']:+.3f}m "
      f"crank={th_end:+.2f}rad flips={out['n_flips']} "
      f"pitch_max={out['max_pitch_deg']}deg E_pump={E_pump:.2f}J")
