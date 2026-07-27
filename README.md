# Drone-Stabilized Passive Dynamic Walker（ドローン安定化受動歩行機）

**An open-source hardware concept: a McGeer-type passive dynamic walker whose
inverted-pendulum instability is converted into stable upright-pendulum
oscillation by an overhead stabilizing wrench — active thrust (Gen‑1) or
partial buoyancy (Gen‑2/3) — while the walking energy itself comes from
passive pendulum dynamics.**

> "A non-flying airship with legs."

---

## English summary

Passive dynamic walkers (McGeer 1990) are energy-efficient but unstable:
the stance phase is a divergent inverted pendulum. This project demonstrates,
in contact-accurate simulation (PyBullet, 1200 Hz event-driven control),
three generations of a machine that keeps the passive limit cycle intact and
stabilizes **attitude only** from above the virtual pivot point:

| Gen | Stabilizer | Propulsion | Key result |
|-----|-----------|------------|-----------|
| 1 | Quadrotor attitude PD (**zero altitude support**, KZ=0) | Horizontal thrust (KV≥4–5) | 10 strikes / 8 s / 0.32 m/s sustained walking |
| 2 | Passive buoyancy (spherical He envelope on mast) | Azimuth ducted fan pair | Operating window **37–94 %W**; gait collapses by ground-force loss (~98 %W) *before* float-off (102 %W) |
| 3 | Passive buoyancy (airship hull + cruciform tail damping) | Azimuth ducted fan pair | 52 %W recommended; tail adds pitch/yaw damping & weathervane potential |
| 4 | **Mass geometry only** (water-ballasted feet, h_com < R, one-way "half-kamaboko" rocker soles) | Water transfer between feet + pump-charged drive bellows through a clock-escapement crank | 6 steps / +2.17 m / 40 s on flat ground, zero falls, body pitch within ±0.4°; crank quantized to half-turns |
| 5 | Mass geometry + "kamae" stance (crank-mounted tanks settle into a one-foot-raised ready pose) | **Water transfer only** — tanks ride the crank, hollow arms are the conduit, hub pump/valve; the whole body somersaults over the stance leg (dankaeri-faithful) | Fall-free 90 s / +0.43 m conservative gait (2 flips then parks); fast mode +1.93 m / 17.8 s (0.11 m/s); root limitation identified: uncontrolled leg attitude |
| 6 | Mass geometry + **gear-locked parallelogram legs** (each leg guided by two same-phase cranks 0.20 m apart; a 3-gear train on the leg locks the linkage through its dead center) | Water transfer only — the tank crank alone somersaults, the body stays upright; phase-synchronized transfer (SETTLE→CHARGE→FLIP) | **Zero falls over the whole pump range** (Q 0.15–0.50); best 18 steps / +2.42 m / 90 s, exact kinematic stride 2·RC = 0.134 m/step, body pitch ≤ 7° |

Distinct from BALLU (UCLA, buoyant body anchored by feet; patent application
US2018/0370040A1 **abandoned 2021**): here buoyancy is deliberately partial
(≈52 % of weight) so the legs stay loaded and heel-strike pendulum dynamics
— the energy mechanism — survive.

Everything here is simulation-validated design data: CAD, engineering
drawings, physics/control source, gait trajectories, and videos. No physical
prototype has been built yet — treat the BOM as a reference design.

## コンセプト（日本語）

受動歩行の振り子効率をドローンに持ち込み、倒立振り子の発散モードを
「上の支点から吊られた正立振り子」の振動モードに転換する。核心は3点：

1. **姿勢のみの上方安定化** — 高度支持ゼロ（KZ=0）でも歩行が成立する。
   安定化は帯域（Gen‑1）または浮心配置の幾何（Gen‑2/3）で払い、エネルギーでは払わない。
2. **部分浮力の適正窓** — 浮力37〜94%W。下限は姿勢発散、上限は浮上（102%W〜）より
   先に来る**接地力喪失による歩容崩壊**（98%W〜）。推奨52%Wで脚に体重の約半分を残し
   摩擦余裕を確保する。
3. **水平推力＝坂の代替** — ヒールストライク損失の補填はファンの水平推力
   （機械仕事0.1W級）のみ。同機体の単独飛行（誘導パワー約186W）に対し3〜4桁小さい。

## Repository layout

```
hardware/
  cad/        FreeCAD sources (.FCStd) + neutral STEP exports, Gen 1–6
  drawings/   A4 engineering drawings (PDF): assembly + parts, 21 sheets
sim/          Physics & control source of truth (PyBullet) + recorded gaits
  walker_pybullet.py   Gen-1  (usage: python walker_pybullet.py out.json <KZ> <KV>)
  walker_balloon.py    Gen-2  (usage: python walker_balloon.py out.json <B_N> [KV])
  walker_airship.py    Gen-3  (usage: python walker_airship.py out.json <B_N> [KV])
  walker_dankaeri5.py  Gen-4  (usage: python walker_dankaeri5.py out.json SLOPE_DEG
                       PUMP_PA [F_WIND] [E_MIN] [M_W] [P_BEL] [V_MAXB];
                       walker_dankaeri4.py is the earlier Gen-4 iteration)
  walker_gen5.py       Gen-5  (usage: python walker_gen5.py out.json SLOPE_DEG
                       Q_PUMP [M_W] [OM_V] [T_END] [R_T] [D_EXTRA] [BRAKES]
                       [C_CRK] [X_TOE] [Q_LIM] [FLIPLOCK] [R_TOE] [FOOT_REV])
  walker_gen6.py       Gen-6  (usage: python walker_gen6.py out.json SLOPE_DEG
                       Q_PUMP [M_W] [OM_V] [T_END] [R_T] [D_EXTRA] [-] [C_CRK]
                       [X_TOE] [-] [-] [R_TOE] [FOOT_REV] [PHASECTL] [DZ_T]
                       [RATCHET] [RC] [SLACK] [HEEL_HX]; reference config:
                       0 0.30 1.2 2.0 90 0.355 0.30 0 0.30 0.10 0 0 0.55 0 1
                       0.290 1 0.07 0.15 0.14)
  walk_traj_*.json     gait logs (base pose, joint angles, contact forces)
docs/         Concept note, per-generation design documents, BOM
media/        Walking videos (one per generation)
viz/          Blender visualization rigs (kinematic playback of sim output)
LICENSES/     Full license texts
```

## Reproducing the results

```
python -m venv pybullet-venv
pybullet-venv/bin/pip install pybullet numpy
pybullet-venv/bin/python sim/walker_pybullet.py out.json 0 5     # Gen-1: KZ=0, KV=5
pybullet-venv/bin/python sim/walker_balloon.py  out.json 13      # Gen-2: B=13 N
pybullet-venv/bin/python sim/walker_airship.py  out.json 13      # Gen-3: B=13 N
```

Each run prints strike count, distance, max pitch and mean fan force, and
writes a 60 Hz trajectory JSON. The Blender rigs in `viz/` replay these
trajectories as kinematic keyframes (Blender is visualization only; the
physics source of truth is `sim/`).

## Licensing（3層構成）

| Layer | Files | License | SPDX |
|-------|-------|---------|------|
| Hardware design | `hardware/**` | CERN Open Hardware Licence v2 — Strongly Reciprocal | `CERN-OHL-S-2.0` |
| Source code | `sim/**` | MIT | `MIT` |
| Documentation & media | `docs/**`, `media/**`, `README.md` | Creative Commons BY-SA 4.0 | `CC-BY-SA-4.0` |

Full texts in [`LICENSES/`](LICENSES/). When adding files, put an SPDX
header in each source file, e.g. `# SPDX-License-Identifier: MIT`.

## Prior art & defensive publication

This design is published as open-source hardware; **no patent will be
sought**, and this repository (archived with a DOI on Zenodo — see
`docs/DEFENSIVE_PUBLICATION.md`) serves as defensive prior art for the
concepts listed above.

Key prior art acknowledged:

- T. McGeer, "Passive Dynamic Walking," *IJRR* 9(2), 1990.
- S. Collins, M. Wisse, A. Ruina, "A Three-Dimensional Passive-Dynamic
  Walking Robot with Two Legs and Knees," *IJRR* 20(7), 2001.
- H.-M. Maus et al., "Upright human gait did not provide a major mechanical
  challenge for our ancestors," *Nature Communications* 1:70, 2010
  (virtual pivot point).
- UCLA RoMeLa, BALLU / BALLU2 (buoyancy-assisted biped);
  US2018/0370040A1 (abandoned 2021), WO2017/087987A1.
- Traditional karakuri "dankaeri" tumbling doll (liquid-shift somersault
  automaton) — mechanism reference for Gen-4/5/6.
- Classic parallel-crank (side-rod) motion — locomotive coupling rods and
  wind-up walking toys — and gear-synchronized parallelogram guidance:
  mechanism references for the Gen-6 leg-attitude lock.

## Status / OSHW

- [ ] Zenodo DOI: *to be assigned on first release*
- [ ] OSHWA self-certification (JP): *pending — apply after first tagged release*
- Simulation-validated; no physical build yet. Real-scale note: B=13 N of
  helium lift requires ≈1.3 m³ (sphere Ø≈1.4 m); the modeled envelopes are
  intentionally idealized.
