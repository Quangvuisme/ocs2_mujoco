#!/usr/bin/env python3
"""Writes robots/g1/scene_parkour.xml: a harder obstacle course for the Unitree G1 along +x.

The walkable surfaces are boxes standing on a pit floor 0.5 m below the start level, so gaps between
boxes are real pits. Sections (default order, see SECTIONS):

    steps     3 steps up and 3 down, rise 0.12 m, tread 0.45 m, 0.8 m platform
    gaps      pits of 0.10, 0.15 and 0.20 m between 0.7 m landings
    stones    stepping stones: rows of a left and a right stone (0.32 x 0.22 m), 0.10 m pits between rows,
              heights +-0.04 m, rows shifted sideways by +-0.04 m
    ramp      ramp up (12 deg), 0.8 m top, ramp down
    hurdles   two bars to step over (0.10 m high, 0.05 m deep)
    bigstep   one 0.15 m step up, 1.0 m platform, 0.15 m step down

All terrain geoms are in geom group 0 (the elevation mapper ray casts groups 0/1, the robot is in 2/3).

Usage:
    generate_parkour.py                               full course -> robots/g1/scene_parkour.xml
    generate_parkour.py --sections gaps,stones -o x.xml   only these sections (for testing)
    generate_parkour.py --list                        print the sections and the goal x of the course
    generate_parkour.py --level hard -o robots/g1/scene_parkour_hard.xml
                                                      harder version (0.15 m stairs, 0.20-0.30 m gaps, 18 deg
                                                      ramp, 0.15 m hurdles, 0.20 m step, rougher stones)
Deterministic, no dependencies.
"""
import argparse
import math
import os
import random

PIT = -0.5       # pit floor [m]
WIDTH = 0.8      # half width of the course [m]
START_END = 1.2  # x where the first section starts [m]


class Course:
    def __init__(self):
        self.geoms = []
        self.x = START_END
        self.rng = random.Random(11)

    def box(self, name, x0, x1, top, y0=-WIDTH, y1=WIDTH, rgba="0.55 0.55 0.60 1", euler=None):
        """Box from the pit floor up to `top`, covering [x0, x1] x [y0, y1]."""
        hx, hy, hz = (x1 - x0) / 2, (y1 - y0) / 2, (top - PIT) / 2
        rot = f' euler="{euler[0]:.4f} {euler[1]:.4f} {euler[2]:.4f}"' if euler else ""
        self.geoms.append(
            f'    <geom name="{name}" type="box" pos="{(x0 + x1) / 2:.4f} {(y0 + y1) / 2:.4f} {PIT + hz:.4f}" '
            f'size="{hx:.4f} {hy:.4f} {hz:.4f}"{rot} rgba="{rgba}" material="terrain" friction="1.0 0.01 0.001"/>')

    def flat(self, name, length, top=0.0, rgba="0.45 0.50 0.55 1"):
        self.box(name, self.x, self.x + length, top, rgba=rgba)
        self.x += length

    # ---- sections -------------------------------------------------------------------------------
    def steps(self, rise=0.12, tread=0.45, n=3, platform=0.8):
        for k in range(n):
            shade = 0.55 + 0.06 * (k % 2)
            self.flat(f"step_up_{k}", tread, rise * (k + 1), f"{shade:.2f} {shade:.2f} {shade + 0.05:.2f} 1")
        self.flat("step_platform", platform, rise * n, "0.50 0.55 0.62 1")
        for k in range(n):
            shade = 0.55 + 0.06 * (k % 2)
            self.flat(f"step_down_{k}", tread, rise * (n - 1 - k), f"{shade:.2f} {shade:.2f} {shade + 0.05:.2f} 1")

    def gaps(self, widths=(0.10, 0.15, 0.20), land=0.7):
        for k, w in enumerate(widths):
            self.x += w  # pit
            self.flat(f"gap_landing_{k}", land, 0.0, "0.40 0.55 0.45 1")

    def stones(self, rows=6, length=0.32, width=0.22, pit=0.10, dz=0.04, dy=0.04):
        self.x += pit
        for r in range(rows):
            shift = self.rng.uniform(-dy, dy)
            for side, yc in (("l", 0.13), ("r", -0.13)):
                top = self.rng.uniform(-dz, dz)
                g = 0.5 + 3.0 * top
                y = yc + shift
                self.box(f"stone_{r}_{side}", self.x, self.x + length, top, y - width / 2, y + width / 2,
                         rgba=f"{g:.2f} {0.85 * g:.2f} {0.65 * g:.2f} 1")
            self.x += length + pit
        self.flat("stones_exit", 0.6, 0.0)

    def ramp(self, angle_deg=12.0, length=1.2, top_length=0.8, thick=0.3):
        a = math.radians(angle_deg)
        h = length * math.tan(a)
        for name, up in (("ramp_up", True), ("ramp_down", False)):
            # slab whose top surface runs from (x, 0) to (x + length, h) (up) or (x, h) to (x + length, 0) (down)
            nx = -math.sin(a) if up else math.sin(a)  # surface normal (nx, 0, cos a)
            cx = self.x + length / 2 - thick / 2 * nx
            cz = h / 2 - thick / 2 * math.cos(a)
            pitch = -a if up else a
            self.geoms.append(
                f'    <geom name="{name}" type="box" pos="{cx:.4f} 0 {cz:.4f}" '
                f'size="{length / 2 / math.cos(a):.4f} {WIDTH:.4f} {thick / 2:.4f}" euler="0 {pitch:.4f} 0" '
                f'rgba="0.60 0.50 0.40 1" material="terrain" friction="1.0 0.01 0.001"/>')
            self.box(f"{name}_fill", self.x, self.x + length, 0.0, rgba="0.45 0.40 0.35 1")
            self.x += length
            if up:
                self.flat("ramp_top", top_length, h, "0.55 0.48 0.40 1")

    def hurdles(self, height=0.10, depth=0.05, n=2, spacing=1.0):
        start = self.x
        self.flat("hurdle_floor", spacing * (n + 0.5), 0.0)
        for k in range(n):
            xc = start + spacing * (k + 0.75)
            self.box(f"hurdle_{k}", xc - depth / 2, xc + depth / 2, height, rgba="0.80 0.45 0.20 1")

    def bigstep(self, rise=0.15, platform=1.0):
        self.flat("bigstep_up", platform, rise, "0.50 0.55 0.62 1")


SECTIONS = ["steps", "gaps", "stones", "ramp", "hurdles", "bigstep"]

# section parameters per difficulty level ("normal" = the defaults of the methods above)
LEVELS = {
    "normal": {},
    "hard": {
        "steps": dict(rise=0.15),
        "gaps": dict(widths=(0.20, 0.25, 0.30)),
        "stones": dict(rows=8, pit=0.14, dz=0.06, dy=0.06),
        "ramp": dict(angle_deg=18.0),
        "hurdles": dict(height=0.15),
        "bigstep": dict(rise=0.20),
    },
}


def build(sections, level="normal", overrides=None):
    c = Course()
    c.box("start", -1.5, START_END, 0.0, rgba="0.40 0.45 0.50 1")
    for k, s in enumerate(sections):
        params = dict(LEVELS[level].get(s, {}))
        params.update((overrides or {}).get(s, {}))
        getattr(c, s)(**params)
        c.flat(f"rest_{k}", 0.8)
    goal = c.x - 0.6
    c.flat("finish", 2.0)
    return c, goal


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser()
    p.add_argument("--sections", default=",".join(SECTIONS))
    p.add_argument("-o", "--output", default=os.path.join(here, "..", "robots", "g1", "scene_parkour.xml"))
    p.add_argument("--level", default="normal", choices=sorted(LEVELS))
    p.add_argument("--param", action="append", default=[],
                   help="override a section parameter, e.g. --param stones.length=0.4 --param gaps.widths=0.1,0.2")
    p.add_argument("--list", action="store_true")
    args = p.parse_args()
    sections = [s for s in args.sections.split(",") if s]
    overrides = {}
    for item in args.param:
        key, value = item.split("=", 1)
        section, name = key.split(".", 1)
        values = tuple(float(v) for v in value.split(","))
        overrides.setdefault(section, {})[name] = values if len(values) > 1 else (int(values[0]) if name in ("n", "rows") else values[0])
    c, goal = build(sections, args.level, overrides)
    if args.list:
        print(f"sections: {', '.join(sections)}  goal x: {goal:.2f}")
        return
    model_dir = os.path.relpath(os.path.join(here, "..", "robots", "g1"), os.path.dirname(os.path.abspath(args.output)))
    include = os.path.join(model_dir, "g1_12dof.xml") if model_dir != "." else "g1_12dof.xml"
    xml = f"""<!-- Generated by scripts/generate_parkour.py --level {args.level} ({', '.join(sections)}) - edit the script, not this file.
     Goal x = {goal:.2f} m -->
<mujoco model="g1_12dof parkour course">
  <include file="{include}"/>

  <statistic center="4 0 0.4" extent="4.0"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="-130" elevation="-20" offwidth="1920" offheight="1080"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.12 0.16 0.22" rgb2="0.08 0.11 0.16"
      markrgb="0.5 0.5 0.5" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.1"/>
    <material name="terrain" specular="0.1" shininess="0.1"/>
  </asset>

  <worldbody>
    <light pos="4 0 3" dir="0 0 -1" directional="true" castshadow="false" diffuse="0.4 0.4 0.4"/>
    <geom name="pit_floor" pos="0 0 {PIT}" size="0 0 0.05" type="plane" material="groundplane" friction="1.0 0.01 0.001"/>
{os.linesep.join(c.geoms)}
  </worldbody>
</mujoco>
"""
    with open(args.output, "w") as f:
        f.write(xml)
    print(f"wrote {args.output} ({len(c.geoms)} geoms, goal x = {goal:.2f})")


if __name__ == "__main__":
    main()
