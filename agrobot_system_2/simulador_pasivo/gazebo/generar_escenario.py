#!/usr/bin/env python3
"""Genera escenarios Gazebo deterministas y con dificultad configurable."""
from __future__ import annotations
import argparse
import copy
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE = HERE / "huerta_ruda.sdf"
MODES = ("surco", "bajo", "baches", "pendiente", "barro", "noche", "mixto")

def tag(parent, name, text=None, attrs=None):
    el = ET.SubElement(parent, name, attrs or {})
    if text is not None:
        el.text = str(text)
    return el

def simple_obstacle(world, name, kind, x, y, sx, sy, sz, color, yaw=0.0):
    model = tag(world, "model", attrs={"name": name})
    tag(model, "static", "true")
    tag(model, "pose", f"{x:.3f} {y:.3f} {sz/2:.3f} 0 0 {yaw:.4f}")
    link = tag(model, "link", attrs={"name": "body"})
    for label in ("collision", "visual"):
        node = tag(link, label, attrs={"name": label})
        geom = tag(node, "geometry")
        tag(geom, "box")
        size = tag(geom.find("box"), "size", f"{sx:.3f} {sy:.3f} {sz:.3f}")
        if label == "visual":
            mat = tag(node, "material")
            tag(mat, "ambient", color)
            tag(mat, "diffuse", color)
    return model

def crop_cluster(world, name, x, y, rng, side):
    """Low, broad crop rosettes that are visible to the camera and collidable."""
    model = tag(world, "model", attrs={"name": name})
    tag(model, "static", "true")
    tag(model, "pose", f"{x:.3f} {y:.3f} 0 0 0 {rng.uniform(-.18,.18):.4f}")
    link = tag(model, "link", attrs={"name": "plant"})
    stem = tag(link, "visual", attrs={"name": "stem"})
    tag(stem, "pose", "0 0 0.105 0 0 0")
    geom = tag(stem, "geometry")
    tag(geom, "cylinder")
    tag(geom.find("cylinder"), "radius", "0.018")
    tag(geom.find("cylinder"), "length", "0.21")
    mat = tag(stem, "material")
    tag(mat, "ambient", "0.22 0.34 0.08 1")
    tag(mat, "diffuse", "0.22 0.34 0.08 1")
    # Rosette leaves span about 20–30 cm, matching the low crops in the photos.
    for i in range(5):
        angle = i * (2 * math.pi / 5) + rng.uniform(-.15, .15)
        length = rng.uniform(.22, .30)
        width = rng.uniform(.15, .21)
        z = rng.uniform(.13, .23)
        dx, dy = math.cos(angle) * .07, math.sin(angle) * .07
        yaw = angle
        color = ("0.16 0.39 0.08 1" if (i + side) % 2 else "0.25 0.50 0.12 1")
        coll = tag(link, "collision", attrs={"name": f"leaf_collision_{i}"})
        tag(coll, "pose", f"{dx:.3f} {dy:.3f} {z:.3f} 0 0 {yaw:.4f}")
        cg = tag(coll, "geometry")
        tag(cg, "box")
        tag(cg.find("box"), "size", f"{length:.3f} {width:.3f} 0.045")
        vis = tag(link, "visual", attrs={"name": f"leaf_visual_{i}"})
        tag(vis, "pose", f"{dx:.3f} {dy:.3f} {z:.3f} 0 0 {yaw:.4f}")
        vg = tag(vis, "geometry")
        leaf = tag(vg, "ellipsoid")
        # Rounded leaf silhouette reads more like the photographed rosettes
        # than the earlier flat rectangular cards. Collisions stay box-shaped
        # and deliberately conservative for robust navigation tests.
        tag(leaf, "radii", f"{length/2:.3f} {width/2:.3f} 0.012")
        m = tag(vis, "material")
        tag(m, "ambient", color)
        tag(m, "diffuse", color)
    return model

def soil_furrow(world, name, x, side, rng):
    """Tramo largo y suave de caballón, con colisión física.

    La cara interior queda en y=±0.25 m: deja el pasillo nominal de 50 cm.
    Cinco tramos solapados por lado evitan el aspecto de piedras sueltas y
    reducen el número de entidades que debe renderizar Gazebo.
    """
    model = tag(world, "model", attrs={"name": name})
    tag(model, "static", "true")
    ry = rng.uniform(.075, .085)
    rx = rng.uniform(1.65, 1.75)
    rz = rng.uniform(.045, .065)
    # Keep long berm segments parallel to the path. Their random longitudinal
    # overlap and height make the earth uneven without narrowing the lane.
    yaw = 0.0
    center_y = side * (.25 + ry + .006)
    center_z = rz * rng.uniform(.70, .82)
    tag(model, "pose", f"{x:.3f} {center_y:.3f} {center_z:.3f} 0 0 {yaw:.4f}")
    link = tag(model, "link", attrs={"name": "tierra"})
    for label in ("collision", "visual"):
        node = tag(link, label, attrs={"name": label})
        geom = tag(node, "geometry")
        ellipsoid = tag(geom, "ellipsoid")
        tag(ellipsoid, "radii", f"{rx:.3f} {ry:.3f} {rz:.3f}")
        if label == "visual":
            mat = tag(node, "material")
            tag(mat, "ambient", "0.25 0.19 0.12 1")
            tag(mat, "diffuse", "0.34 0.27 0.18 1")
    return model

def set_path(root, path, value):
    el = root.find(path)
    if el is not None:
        el.text = str(value)

def generate(mode: str, seed: int, output: Path):
    rng = random.Random(seed)
    tree = ET.parse(BASE)
    root = tree.getroot()
    world = root.find("world")
    world.set("name", "huerta_ruda")
    # Fresh, repeatable layout; crop rows frame a 50 cm nominal travel corridor.
    for m in list(world.findall("model")):
        if m.get("name", "").startswith(("col_", "lechuga_", "roca_", "rama_", "cultivo_", "bache_", "piedra_", "rama_baja_", "surco_")):
            world.remove(m)
    friction = {"surco": .82, "bajo": .75, "baches": .68, "pendiente": .72,
                "barro": .38, "noche": .82, "mixto": .52}[mode]
    set_path(root, ".//model[@name='suelo']/link/collision/surface/friction/ode/mu", f"{friction:.2f}")
    set_path(root, ".//model[@name='suelo']/link/collision/surface/friction/ode/mu2", f"{friction*.78:.2f}")
    ground = world.find("model[@name='suelo']")
    if ground is not None:
        pose = ground.find("pose")
        if pose is None:
            pose = ET.Element("pose")
            ground.insert(0, pose)
        pose.text = "0 0 0 0 0.055 0" if mode == "pendiente" else "0 0 0 0 0 0"
    # Two continuous, gently irregular berms reproduce the raised soil edges
    # visible in the garden photos. Their inner faces preserve >=50 cm clearance.
    furrow_x = [(-2.8 + 2.7*k) for k in range(5)]
    for i, x in enumerate(furrow_x):
        for side in (-1, 1):
            soil_furrow(world, f"surco_{'izq' if side < 0 else 'der'}_{i:02d}",
                        x + rng.uniform(-.06, .06), side, rng)
    # Camera exposure challenge through scene light; retain a low floor so image remains usable.
    light = world.find("light[@name='sun']")
    if light is not None:
        value = .22 if mode == "noche" else .65 if mode == "mixto" else .86
        set_path(light, "diffuse", f"{value} {value*.96:.3f} {value*.86:.3f} 1")
    # Camera-visible rosettes flank a 50 cm dirt corridor like the supplied
    # garden photos. Earlier tiny single-box crops starved visual guidance.
    for i, x in enumerate([-.75 + .48*k for k in range(14)]):
        for side in (-1, 1):
            crop_cluster(world, f"cultivo_{i:02d}_{'izq' if side<0 else 'der'}",
                         x+rng.uniform(-.045,.045), side*(.43+rng.uniform(-.015,.015)),
                         rng, side)
    # Hard low objects challenge monocular camera while sitting below the LiDAR plane.
    if mode in ("bajo", "mixto"):
        for i in range(7):
            x = rng.uniform(-.65, 4.5); y = rng.choice((-1, 1))*rng.uniform(.20,.31)
            simple_obstacle(world, f"rama_baja_{i}", "branch", x, y, rng.uniform(.25,.48),
                            rng.uniform(.025,.055), rng.uniform(.035,.07),
                            "0.31 0.19 0.09 1", rng.uniform(-.6,.6))
        for i in range(4):
            x = rng.uniform(-.4,4.6); y = rng.uniform(-.23,.23)
            simple_obstacle(world, f"piedra_baja_{i}", "rock", x,y,
                            rng.uniform(.10,.19),rng.uniform(.10,.18),rng.uniform(.04,.09),
                            "0.40 0.38 0.34 1",rng.uniform(-.5,.5))
    # Small raised soil clods at wheel-track locations; not tall enough to block chassis.
    if mode in ("baches", "mixto"):
        for i in range(14):
            x = rng.uniform(-1.0,4.8); y = rng.choice((-1,1))*rng.uniform(.11,.20)
            h = rng.uniform(.012,.042)
            simple_obstacle(world, f"bache_{i:02d}", "soil", x,y,rng.uniform(.08,.18),
                            rng.uniform(.08,.16),h,"0.31 0.25 0.17 1",rng.uniform(-.3,.3))
    # Seed and profile are embedded for auditability.
    world.insert(0, ET.Comment(f" escenario={mode}; semilla={seed}; friccion_base={friction:.2f} "))
    ET.indent(tree, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return friction

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--modo",choices=MODES,default="mixto")
    p.add_argument("--semilla",type=int,default=20261012)
    p.add_argument("--salida",type=Path,default=HERE/"generados"/"huerta_mixta.sdf")
    a=p.parse_args()
    mu=generate(a.modo,a.semilla,a.salida)
    print(f"generado={a.salida} modo={a.modo} semilla={a.semilla} mu={mu:.2f}")
if __name__=="__main__": main()
