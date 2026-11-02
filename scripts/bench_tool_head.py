#!/usr/bin/env python3
"""Measure the contact tool's sprung head as Gazebo simulates it.

Gazebo's solver does not give a prismatic joint the stiffness its SDF sets: a
light head on a wall settles deeper than ``spring_stiffness`` says, by an amount
that depends on the head's mass and damping. This bench finds what a head
really does. Each variant is a carriage on a vertical slide resting on the
ground through a copy of the tool's head (two balls, a sprung prismatic joint,
the force/torque sensor at its base), under a gravity that makes the
carriage's weight the contact force.

    bench_tool_head.py [--force 3.0] [--carriage 2.8] HEAD_KG:DAMPING:STIFFNESS ...
    bench_tool_head.py 0.02:6:80 0.1:50:150

Prints, per variant, the settled stroke and the stiffness it implies, and the
overshoot after the carriage drops 4.5 cm onto the head. Needs Gazebo Harmonic
(``gz sim``); runs headless on its own partition, about 15 s.
"""

import argparse
import os
import re
import subprocess
import tempfile
import time

DROP_M = 0.045
SLIDE = "slide"
SPRING = "spring"


def variant_sdf(i, carriage_kg, head_kg, damping, stiffness, travel=0.3):
    return f"""
<model name='m{i}'><pose>{i} 0 0 0 0 0</pose>
 <link name='base'><pose>0 0 1 0 0 0</pose>
  <inertial><mass>100</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial></link>
 <joint name='fix' type='fixed'><parent>world</parent><child>base</child></joint>
 <link name='cart'><pose>0 0 {0.315 + DROP_M} 0 0 0</pose>
  <inertial><mass>{carriage_kg}</mass><inertia><ixx>.01</ixx><iyy>.01</iyy><izz>.01</izz></inertia></inertial></link>
 <joint name='{SLIDE}' type='prismatic'><parent>base</parent><child>cart</child>
  <axis><xyz expressed_in='__model__'>0 0 1</xyz></axis></joint>
 <link name='tool'><pose relative_to='cart'>0 0 0 0 0 0</pose>
  <inertial><mass>0.01</mass><inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia></inertial></link>
 <joint name='j_tool' type='fixed'><parent>cart</parent><child>tool</child>
  <sensor name='ft' type='force_torque'><always_on>1</always_on><update_rate>100</update_rate>
   <force_torque><frame>child</frame><measure_direction>child_to_parent</measure_direction></force_torque>
  </sensor></joint>
 <link name='head'><pose relative_to='tool'>0 0 -0.3 0 0 0</pose>
  <inertial><mass>{head_kg}</mass>
   <inertia><ixx>{3.5e-3 * head_kg}</ixx><iyy>{3.5e-4 * head_kg}</iyy><izz>{3.5e-3 * head_kg}</izz></inertia></inertial>
  <collision name='l'><pose>0 .1 0 0 0 0</pose><geometry><sphere><radius>.015</radius></sphere></geometry></collision>
  <collision name='r'><pose>0 -.1 0 0 0 0</pose><geometry><sphere><radius>.015</radius></sphere></geometry></collision>
 </link>
 <joint name='{SPRING}' type='prismatic'><parent>tool</parent><child>head</child>
  <axis><xyz expressed_in='__model__'>0 0 -1</xyz>
   <limit><lower>-{travel}</lower><upper>{travel}</upper></limit>
   <dynamics><damping>{damping}</damping><spring_stiffness>{stiffness}</spring_stiffness>
    <spring_reference>0</spring_reference></dynamics></axis></joint>
 <plugin filename='gz-sim-joint-state-publisher-system' name='gz::sim::systems::JointStatePublisher'/>
</model>"""


def world_sdf(variants, carriage_kg, force_n):
    models = "".join(variant_sdf(i, carriage_kg, *v) for i, v in enumerate(variants))
    return f"""<?xml version='1.0'?>
<sdf version='1.9'><world name='bench'>
 <physics type='ode'><max_step_size>0.004</max_step_size><real_time_factor>0</real_time_factor></physics>
 <plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>
 <plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>
 <plugin filename='gz-sim-forcetorque-system' name='gz::sim::systems::ForceTorque'/>
 <gravity>0 0 {-force_n / carriage_kg}</gravity>
 <model name='ground'><static>1</static><link name='l'><collision name='c'>
  <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision></link></model>
 {models}
</world></sdf>"""


def joint_positions(text, joint):
    """Positions of ``joint`` in a stream of `gz topic -e` Model messages."""
    out = []
    for block in text.split("header {")[1:]:
        m = re.search(r'name: "' + joint + r'"(.*?)(?:joint \{|$)', block, re.S)
        p = re.search(r"position: ([-e0-9.]+)", m.group(1)) if m else None
        out.append(float(p.group(1)) if p else 0.0)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("variants", nargs="+", help="HEAD_KG:DAMPING:STIFFNESS")
    parser.add_argument("--force", type=float, default=3.0, help="contact force, N")
    parser.add_argument("--carriage", type=float, default=2.8, help="moving mass, kg (drone and arm)")
    args = parser.parse_args()
    variants = [tuple(float(x) for x in v.split(":")) for v in args.variants]

    env = dict(os.environ, GZ_PARTITION=os.environ.get("GZ_PARTITION", "uavsim") + "_bench")
    with tempfile.TemporaryDirectory() as tmp:
        world = os.path.join(tmp, "bench.sdf")
        with open(world, "w") as f:
            f.write(world_sdf(variants, args.carriage, args.force))
        logs = [open(os.path.join(tmp, f"m{i}.txt"), "w") for i in range(len(variants))]
        echoes = [subprocess.Popen(["gz", "topic", "-e", "-t", f"/world/bench/model/m{i}/joint_state"],
                                   stdout=log, stderr=subprocess.DEVNULL, env=env)
                  for i, log in enumerate(logs)]
        time.sleep(1.5)
        # 10 s of simulated time, as fast as the machine runs it.
        subprocess.run(["gz", "sim", "-s", "-r", "--iterations", "2500", world],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env, timeout=120)
        time.sleep(0.5)
        for p in echoes:
            p.terminate()
            p.wait()
        for log in logs:
            log.close()
        print(f"{args.force:.1f} N on a {args.carriage:.1f} kg carriage, dropped {DROP_M * 100:.1f} cm")
        print("head kg  damping  stiffness set | settled stroke  stiffness measured  overshoot")
        for i, (head, damping, stiffness) in enumerate(variants):
            with open(os.path.join(tmp, f"m{i}.txt")) as f:
                stroke = [-s for s in joint_positions(f.read(), SPRING)]
            if len(stroke) < 200:
                print(f"{head:7.3f}  {damping:7.1f}  {stiffness:9.0f}     | no joint states")
                continue
            settled = sorted(stroke[-100:])[50]
            peak = max(stroke)
            print(f"{head:7.3f}  {damping:7.1f}  {stiffness:9.0f}     | {settled * 100:9.1f} cm  "
                  f"{args.force / settled:12.0f} N/m  {(peak - settled) / settled * 100:9.0f} %")


if __name__ == "__main__":
    main()
