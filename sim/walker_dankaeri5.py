#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Gen-4 v5: clock architecture completed -- WIND BUFFER + PHASE-GATED RELEASE.

Winding (continuous): a constant-force winder on each foot spring resists
stroke motion in BOTH directions (regenerative damper); the extracted work
charges a mainspring barrel E_buf. Rocking oscillation itself is harvested.

Release (discrete): the crank is held latched at the nearest detent (0/pi).
When the barrel holds enough energy AND the machine is rocking FORWARD
(base vx > V_TRIG -- the dankaeri timing), the latch opens and the barrel
torque drives the crank through exactly half a turn, then re-latches.

Everything else as v4: rocker feet (R=0.30, +0.05 forward offset), free
gondola hips + load-engaged stance hip brake, bladder out of load path,
crank-tilt siphon hydraulics with crank-angle rotary-valve pump commutation.

Usage: walker_dankaeri5.py out.json SLOPE_DEG PUMP_PA [F_WIND] [E_MIN] [M_W]
"""
import json, math, sys
import pybullet as p

Z0, RC, YLEG = 0.84, 0.07, 0.14
H_LEG = Z0 - 0.015
STROKE = 0.05
M_BASE, M_CRANK = 0.20, 0.10
M_LEG, M_FOOT0 = 0.30, 0.35

SLOPE = math.radians(float(sys.argv[2])) if len(sys.argv) > 2 else 0.0
P_PUMP = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
F_WIND = float(sys.argv[4]) if len(sys.argv) > 4 else 12.0    # winder force [N]
E_MIN  = float(sys.argv[5]) if len(sys.argv) > 5 else 0.4    # release charge [J]
M_W    = float(sys.argv[6]) if len(sys.argv) > 6 else 1.20

R_EFF   = STROKE/math.pi   # rack ratio: full stroke = half turn
K_C     = 4.0
TAU_CAP = 1.5
V_TRIG  = 0.008     # forward-rock trigger on base vx [m/s]
K_DET   = 0.25     # physical detent (mild; the latch does the holding)
K_HOLD  = 20.0
# hydraulic drive bellows (windkessel actuator on the rack)
P_BEL  = float(sys.argv[7]) if len(sys.argv) > 7 else 12000.0   # [Pa]
V_MAXB = float(sys.argv[8]) if len(sys.argv) > 8 else 0.28e-3   # [m^3]
A_BEL, Q_CHG = 0.008, 0.15e-3

RHO = 1000.0
L_HOSE, D_HOSE = 0.5, 0.028
A_P = math.pi*D_HOSE**2/4
L_H = RHO*L_HOSE/A_P
K_H = (0.03*L_HOSE/D_HOSE + 3.0)*RHO/(2*A_P*A_P)
K_SPR, C_SPR = 700.0, 90.0

DT, T_END = 1.0/2400.0, 40.0

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
# half-kamaboko foot: flat heel (one-way catch) + tangent toe arc (forward roll)
R_TOE = 0.30
quat_y = p.getQuaternionFromEuler([math.pi/2, 0, 0])
foot_col = p.createCollisionShapeArray(
    shapeTypes=[p.GEOM_BOX, p.GEOM_CYLINDER],
    radii=[0, R_TOE],
    lengths=[0, 0.26],
    halfExtents=[[0.10, 0.13, 0.015], [0, 0, 0]],
    collisionFramePositions=[[-0.10, 0, 0], [0, 0, R_TOE - 0.015]],
    collisionFrameOrientations=[[0, 0, 0, 1], quat_y])

w = [M_W*0.5, M_W*0.5]
blad_col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.05)
Z_BLAD = -(Z0 - 0.22)      # bladder rides the SPRUNG leg, low (upper plate top)
walker = p.createMultiBody(
    baseMass=M_BASE,
    baseCollisionShapeIndex=axle_col,
    basePosition=[0, 0, Z0],
    linkMasses=[M_CRANK, M_LEG, M_FOOT0, w[0], M_LEG, M_FOOT0, w[1]],
    linkCollisionShapeIndices=[crank_col, leg_col, foot_col, blad_col,
                               leg_col, foot_col, blad_col],
    linkVisualShapeIndices=[-1]*7,
    linkPositions=[[0,0,0],
                   [ RC, -YLEG, 0], [0, 0, -H_LEG], [0, -0.09, Z_BLAD],
                   [-RC,  YLEG, 0], [0, 0, -H_LEG], [0,  0.09, Z_BLAD]],
    linkOrientations=[[0,0,0,1]]*7,
    linkInertialFramePositions=[[0,0,0],
                                [0,0,-H_LEG*0.45], [0,0,0], [0,0,0],
                                [0,0,-H_LEG*0.45], [0,0,0], [0,0,0]],
    linkInertialFrameOrientations=[[0,0,0,1]]*7,
    linkParentIndices=[0, 1, 2, 2, 1, 5, 5],
    linkJointTypes=[p.JOINT_REVOLUTE,
                    p.JOINT_REVOLUTE, p.JOINT_PRISMATIC, p.JOINT_FIXED,
                    p.JOINT_REVOLUTE, p.JOINT_PRISMATIC, p.JOINT_FIXED],
    linkJointAxis=[[0,1,0], [0,1,0], [0,0,1], [0,0,1],
                   [0,1,0], [0,0,1], [0,0,1]])

CRANK, HIP_L, FOOT_L, BLAD_L, HIP_R, FOOT_R, BLAD_R = 0, 1, 2, 3, 4, 5, 6
FEET = {0: FOOT_L, 1: FOOT_R}
BLADS = {0: BLAD_L, 1: BLAD_R}
for j in range(7):
    p.setJointMotorControl2(walker, j, p.VELOCITY_CONTROL, force=0)
for l in (BLAD_L, BLAD_R):
    p.setCollisionFilterGroupMask(walker, l, 0, 0)
for j in (FOOT_L, FOOT_R):
    p.changeDynamics(walker, j, jointLowerLimit=0.0, jointUpperLimit=STROKE,
                     jointLimitForce=3000, lateralFriction=0.9, restitution=0.0)
# hips: unlimited gondola bearings
p.changeDynamics(walker, -1, linearDamping=0.02, angularDamping=0.05)
I_C = 0.002
p.changeDynamics(walker, CRANK, localInertiaDiagonal=[I_C, I_C, I_C])
p.changeDynamics(walker, -1, localInertiaDiagonal=[0.01, 0.01, 0.01])
p.setCollisionFilterGroupMask(walker, CRANK, 0, 0)

m_ref = {BLAD_L: max(w[0],1e-3), BLAD_R: max(w[1],1e-3)}
I_ref = {l: p.getDynamicsInfo(walker, l)[2] for l in (BLAD_L, BLAD_R)}

def set_water(i):
    l = BLADS[i]
    m = max(w[i], 1e-3)
    s = m/m_ref[l]
    p.changeDynamics(walker, l, mass=m,
                     localInertiaDiagonal=[max(c*s,1e-7) for c in I_ref[l]])

msum, mz = M_BASE, M_BASE*Z0
for l in range(7):
    di = p.getDynamicsInfo(walker, l)
    msum += di[0]; mz += di[0]*p.getLinkState(walker, l)[0][2]
print(f"# m={msum:.2f} kg, h_com={mz/msum:.3f} m, toe R={R_TOE}")

KY, KDY, KROT = 250.0, 25.0, 1.5
C_ROCK = 0.25    # light: rocking must stay alive (it is the pendulum)
hip_lock = [False, False]
q_h = 0.0
E_wind = E_rel = E_pump = 0.0
ref = [0.0, 0.0]
latched = True
th_det = 0.0
th_rel0 = 0.0
t_rel = -9.0
t_stuck = 0.0
V_bel = 0.0
osc_on = False
osc_dir = -1.0      # bang-bang slosh direction (multiplies like cos-drive)
n_rel = 0
th_hold = 0.0
frames = []
fell = False

for i in range(int(T_END/DT)):
    t = i*DT
    pos, orn = p.getBasePositionAndOrientation(walker)
    vel, avel = p.getBaseVelocity(walker)

    p.applyExternalForce(walker, -1, [0, -KY*pos[1]-KDY*vel[1], 0], pos, p.WORLD_FRAME)
    p.applyExternalTorque(walker, -1, [-KROT*avel[0], -C_ROCK*avel[1], -KROT*avel[2]], p.WORLD_FRAME)

    th, om = p.getJointState(walker, CRANK)[0:2]

    # feet: spring + direct rack-freewheel coupling (engaged while released)
    qs = [0.0, 0.0]
    tau_c = 0.0
    for k, jf in ((0, FOOT_L), (1, FOOT_R)):
        qf, qd = p.getJointState(walker, jf)[0:2]
        qs[k] = qf
        cps = p.getContactPoints(walker, ground, linkIndexA=jf)
        Fs = K_SPR*qf + C_SPR*qd
        F_react = 0.0
        if (not latched) and len(cps) > 0 and t > 1.0:
            tau_i = K_C*(qf/R_EFF - (th - ref[k])) + 0.3*(qd/R_EFF - om)
            if tau_i > 0:
                tau_i = min(tau_i, TAU_CAP)
                tau_c += tau_i
                F_react = tau_i/R_EFF
            else:
                ref[k] = th - qf/R_EFF
        else:
            ref[k] = th - qf/R_EFF
        p.setJointMotorControl2(walker, jf, p.TORQUE_CONTROL,
                                force=-(Fs + F_react))

    # stance hip brake (load-engaged, hysteresis)
    for k, jh, jf in ((0, HIP_L, FOOT_L), (1, HIP_R, FOOT_R)):
        cf = sum(c[9] for c in p.getContactPoints(walker, ground, linkIndexA=jf))
        if hip_lock[k]:
            if cf < 6.0:
                hip_lock[k] = False
        else:
            if cf > 14.0:
                hip_lock[k] = True
        if hip_lock[k] and latched:
            # stance rigidity only while holding; during the flip the crank
            # must swing freely on the gondola bearings (dankaeri order)
            p.setJointMotorControl2(walker, jh, p.VELOCITY_CONTROL,
                                    targetVelocity=0, force=30.0)
        else:
            p.setJointMotorControl2(walker, jh, p.VELOCITY_CONTROL, force=0)

    # hydraulics: crank-tilt siphon + rotary-valve pump commutation
    pL = p.getLinkState(walker, BLAD_L)[0]
    pR = p.getLinkState(walker, BLAD_R)[0]
    hL = -pL[0]*math.sin(SLOPE) + pL[2]*math.cos(SLOPE)
    hR = -pR[0]*math.sin(SLOPE) + pR[2]*math.cos(SLOPE)
    dP = RHO*9.81*(hL-hR)
    # stuck detector (any latch state) -> bang-bang water-slosh excitation.
    # each full transfer = 17mm stroke = ~1.07 rad of crank winding through
    # the freewheel; the geometric diode rectifies all of it forward.
    moving = (abs(vel[0]) > 0.01) or (abs(om) > 0.5)
    if t > 2.0 and not moving:
        t_stuck += DT
    else:
        t_stuck = 0.0
        if moving:
            osc_on = False
    if t_stuck > 1.5 and not osc_on:
        osc_on = True
        osc_dir = -1.0 if w[0] >= w[1] else 1.0   # pull water off the full side
    if (not latched) and (th - th_rel0) > 0.25:
        osc_on = False                            # flip underway: commit
    if P_PUMP != 0.0 and t > 0.5:
        ramp = min(1.0, (t-0.5)/0.5)
        if osc_on:
            if w[0] >= 0.92*M_W:
                osc_dir = -1.0
            elif w[1] >= 0.92*M_W:
                osc_dir = 1.0
            drive = osc_dir
        else:
            drive = math.cos(th)
        dP += P_PUMP*ramp*drive
        E_pump += abs(P_PUMP*ramp*drive*q_h)*DT
    q_h += (dP - K_H*q_h*abs(q_h))/L_H*DT
    dm = RHO*q_h*DT
    dm = max(-w[1], min(w[0], dm))
    if abs(dm) > 0:
        w[0] -= dm; w[1] += dm
        set_water(0); set_water(1)
    if w[0] <= 1e-6 or w[1] <= 1e-6:
        q_h = 0.0

    # crank: latch / release state machine + mild physical detent + ratchet
    # release = dankaeri condition: water fully shifted AND moving forward
    tau = tau_c - K_DET*math.sin(2*th) - 0.10*om
    E_rel += tau_c*max(om, 0.0)*DT
    if latched:
        tau += max(-2.0, min(2.0, -K_HOLD*(th - th_det) - 1.5*om))
        # latch = closed rotary valve: the pump charges the drive bellows
        if V_bel < V_MAXB and t > 1.0:
            V_bel = min(V_MAXB, V_bel + Q_CHG*DT)
            E_pump += P_BEL*Q_CHG*DT
        # release: dankaeri condition (water shifted) AND bellows charged
        f_front = 0 if math.cos(th) > 0 else 1
        if (t > 2.0 and t - t_rel > 1.0 and w[f_front] >= 0.7*M_W
                and V_bel >= 0.8*V_MAXB):
            latched = False
            th_rel0 = th
            t_rel = t
            n_rel += 1
    else:
        # bellows discharges through the rack: torque until volume spent
        if V_bel > 0.0:
            tau += P_BEL*A_BEL*R_EFF
            dV = A_BEL*R_EFF*max(om, 0.0)*DT
            V_bel = max(0.0, V_bel - dV)
            E_rel += P_BEL*A_BEL*R_EFF*max(om, 0.0)*DT
        if th >= th_rel0 + math.pi - 0.05:
            latched = True
            th_det = round(th/math.pi)*math.pi
            V_bel = 0.0                     # vent surplus (escapement dump)
        elif t - t_rel > 4.0:               # stalled flip: re-latch, retry later
            latched = True
            th_det = round(th/math.pi)*math.pi
            V_bel = 0.0
    th_hold = max(th_hold, th)
    if th < th_hold - 0.02:
        tau += 20.0*(th_hold - 0.02 - th) - 0.2*om
    p.setJointMotorControl2(walker, CRANK, p.TORQUE_CONTROL, force=tau)
    # reaction re-routing: the joint dumps -tau into the free axle body, but
    # the real load paths are rack->stance foot (drive) and detent lever->leg.
    # cancel the base reaction and ground it through the loaded foot instead.
    k_st = 0 if qs[0] >= qs[1] else 1
    p.applyExternalTorque(walker, -1, [0, tau, 0], p.WORLD_FRAME)
    p.applyExternalTorque(walker, FEET[k_st], [0, -tau, 0], p.WORLD_FRAME)

    p.stepSimulation()

    if pos[2] < 0.55 or pos[2] > 1.1:
        fell = True
        break

    if i % 20 == 0:
        cf = [sum(c[9] for c in p.getContactPoints(walker, ground, linkIndexA=l))
              for l in (FOOT_L, FOOT_R)]
        frames.append({'t': round(t,4),
            'base': [round(v,5) for v in pos]+[round(v,6) for v in orn],
            'crank': round(th,4), 'om': round(om,3), 'E': round(E_rel,3),
            'lat': int(latched),
            'prism': [round(qs[0],4), round(qs[1],4)],
            'wL': round(w[0],4), 'wR': round(w[1],4), 'V': round(V_bel*1e6),
            'fL': round(cf[0],1), 'fR': round(cf[1],1)})

th_end = p.getJointState(walker, CRANK)[0]
pos, _ = p.getBasePositionAndOrientation(walker)
out = {'slope_deg': math.degrees(SLOPE), 'pump_Pa': P_PUMP,
       'F_WIND': F_WIND, 'E_MIN': E_MIN, 'M_W': M_W,
       'fell': fell, 't_end': frames[-1]['t'] if frames else 0,
       'distance': round(pos[0],3), 'crank_rad': round(th_end,3),
       'n_flips': int(abs(th_end)//math.pi), 'n_releases': n_rel,
       'E_wind_J': round(E_wind,3), 'E_rel_J': round(E_rel,3),
       'E_pump_J': round(E_pump,3),
       'dt_record': 1/60., 'frames': frames}
with open(sys.argv[1], 'w') as f:
    json.dump(out, f)
print(f"slope={math.degrees(SLOPE):4.1f} pump={P_PUMP:6.0f}Pa P_BEL={P_BEL:.0f} "
      f"V_MAXB={V_MAXB*1e6:.0f}ml  fell={fell} t={out['t_end']:.2f}s "
      f"dist={out['distance']:+.2f}m crank={th_end:+.2f}rad flips={out['n_flips']} "
      f"rel={n_rel} E_wind={E_wind:.2f}J E_rel={E_rel:.2f}J E_pmp={E_pump:.2f}J")
