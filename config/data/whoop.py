"""Bake a compact whoop display model with four guards. Units are metres."""
import json
import math
from pathlib import Path
import struct

TAU = math.tau
document = dict(asset=dict(version="2.0", generator="miniflight/config/data/whoop.py"),
                scene=0, scenes=[dict(nodes=[])], nodes=[], meshes=[], materials=[],
                buffers=[], bufferViews=[], accessors=[])
binary = bytearray()


def material(name, color, metal=0, roughness=.5):
    document["materials"].append(dict(name=name, pbrMetallicRoughness=dict(
        baseColorFactor=[*color, 1], metallicFactor=metal, roughnessFactor=roughness)))
    return len(document["materials"])-1


def accessor(values, kind):
    components = 3 if kind == "VEC3" else 1
    start = len(binary)
    flat = [v for row in values for v in row] if components == 3 else values
    binary.extend(struct.pack("<" + ("f" if components == 3 else "H")*len(flat), *flat))
    document["bufferViews"].append(dict(buffer=0, byteOffset=start, byteLength=len(binary)-start))
    record = dict(bufferView=len(document["bufferViews"])-1, componentType=5126 if components == 3 else 5123,
                  count=len(values), type=kind)
    if components == 3:
        record.update(min=list(map(min, zip(*values))), max=list(map(max, zip(*values))))
    document["accessors"].append(record)
    binary.extend(b"\0" * (-len(binary) % 4))
    return len(document["accessors"])-1


def mesh(vertices, normals, indices, surface):
    primitive = dict(attributes=dict(POSITION=accessor(vertices, "VEC3"), NORMAL=accessor(normals, "VEC3")),
                     indices=accessor(indices, "SCALAR"), material=surface)
    document["meshes"].append(dict(primitives=[primitive]))
    return len(document["meshes"])-1


def node(name, geometry, xyz=(0, 0, 0), rotation=None):
    entry = dict(name=name, mesh=geometry, translation=list(xyz))
    if rotation is not None:
        entry["rotation"] = rotation
    document["scenes"][0]["nodes"].append(len(document["nodes"]))
    document["nodes"].append(entry)


def lathe(profile, surface, segments=64):
    vertices, normals, indices = [], [], []
    for (r0, y0), (r1, y1) in zip(profile, profile[1:] + profile[:1]):
        length = math.hypot(r1-r0, y1-y0)
        base = len(vertices)
        for i in range(segments+1):
            a = TAU*i/segments
            c, s = math.cos(a), math.sin(a)
            vertices.extend(((r0*c, y0, r0*s), (r1*c, y1, r1*s)))
            normal = ((y1-y0)*c/length, (r0-r1)/length, (y1-y0)*s/length)
            normals.extend((normal, normal))
        for i in range(segments):
            a = base + 2*i
            indices.extend((a, a+1, a+3, a, a+3, a+2))
    return mesh(vertices, normals, indices, surface)


def solid(points, bottom, top, surface):
    vertices, normals, indices = [], [], []

    def triangle(a, b, c):
        u, v = [y-x for x, y in zip(a, b)], [y-x for x, y in zip(a, c)]
        n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        length = math.sqrt(sum(x*x for x in n))
        indices.extend(range(len(vertices), len(vertices)+3))
        vertices.extend((a, b, c))
        normals.extend([tuple(x/length for x in n)]*3)

    for i, (x, z) in enumerate(points):
        nx, nz = points[(i+1) % len(points)]
        a, b, c, d = (x, bottom, z), (x, top, z), (nx, top, nz), (nx, bottom, nz)
        triangle(a, b, c)
        triangle(a, c, d)
        if 0 < i < len(points)-1:
            px, pz = points[0]
            triangle((px, top, pz), c, b)
            triangle((px, bottom, pz), a, d)
    return mesh(vertices, normals, indices, surface)


def plate(width, depth, height, surface, bevel=.001):
    x, z, b = width/2, depth/2, bevel
    return solid([(-x+b, -z), (x-b, -z), (x, -z+b), (x, z-b),
                  (x-b, z), (-x+b, z), (-x, z-b), (-x, -z+b)], -height/2, height/2, surface)


def cylinder(radius, height, surface):
    return lathe([(radius, -height/2), (radius, height/2), (0, height/2), (0, -height/2)], surface, 32)


frame = material("Carbon frame", (.006, .008, .011), .1, .65)
guard = material("Smoke guards", (.018, .023, .027), .1, .48)
rim = material("Guard rims", (.035, .043, .048), .15, .4)
gold = material("Motor bells", (.65, .37, .08), .8, .3)
metal = material("Fasteners", (.32, .38, .42), .85, .3)
prop = material("Propellers", (.018, .026, .03), .15, .45)
green = material("Battery sleeve", (.38, .72, .2), .05, .48)
black = material("Camera", (.018, .025, .033), .15, .35)
glass = material("Lens", (.015, .09, .13), .85, .12)

# Each motor has an open annular guard and a separate propeller pivot.
guard_mesh = lathe([(.023, -.006), (.0235, -.0055), (.0235, .001), (.023, .0015),
                    (.0215, .0015), (.021, .001), (.021, -.0055), (.0215, -.006)], guard)
rim_mesh = lathe([(.0235, .0007), (.0235, .0018), (.0211, .0018), (.0211, .0007)], rim)
motor = cylinder(.0052, .0055, gold)
cap = cylinder(.0053, .0012, frame)
screw = cylinder(.0009, .0007, metal)
arm = plate(.005, .034, .002, frame, .001)
blade_points = [(.002, -.001), (.015, -.0038), (.0197, -.0022), (.0185, .0005), (.004, .0022)]
blades = []
for i in range(3):
    a = TAU*i/3
    blades.append([(x*math.cos(a)-z*math.sin(a), x*math.sin(a)+z*math.cos(a)) for x,z in blade_points])
# One mesh keeps each propeller's animation centred on its own shaft.
blade_meshes = [solid(points, -.00045, .00045, prop) for points in blades]
document["meshes"].append(dict(primitives=[document["meshes"][i]["primitives"][0] for i in blade_meshes]))
propeller = len(document["meshes"])-1

for i, (x, z) in enumerate(((-.0265, -.0265), (.0265, -.0265), (-.0265, .0265), (.0265, .0265))):
    node(f"guard-{i+1}", guard_mesh, (x, -.001, z))
    node(f"rim-{i+1}", rim_mesh, (x, -.001, z))
    node(f"motor-{i+1}", motor, (x, .0035, z))
    node(f"motor-cap-{i+1}", cap, (x, .0069, z))
    node(f"propeller-{i+1}", propeller, (x, -.0025, z), [0, math.sin(i*.4), 0, math.cos(i*.4)])
    angle = math.atan2(x, z)
    node(f"arm-{i+1}", arm, (x*.52, .0055, z*.52), [0, math.sin(angle/2), 0, math.cos(angle/2)])
    for dx, dz in ((-.002, 0), (.002, 0)):
        node(f"bolt-{i}-{dx}", screw, (x+dx, .0077, z+dz))

node("centre-frame", plate(.024, .041, .002, frame), (0, .0055, .002))
node("electronics", plate(.022, .026, .006, black), (0, -.001, .002))
node("battery", plate(.014, .029, .009, green, .002), (0, .0115, .004))
node("battery-strap", plate(.016, .006, .0016, frame), (0, .0167, .004))
node("front-camera", plate(.017, .013, .012, black, .002), (0, .005, -.028))
forward = [-math.sqrt(.5), 0, 0, math.sqrt(.5)]
node("lens-barrel", cylinder(.0063, .004, frame), (0, .005, -.036), forward)
node("lens-rim", lathe([(.0059,-.0007),(.0059,.0007),(.0048,.0007),(.0048,-.0007)], metal, 40),
     (0, .005, -.0385), forward)
node("lens-glass", cylinder(.0048, .0005, glass), (0, .005, -.0392), forward)
node("antenna", cylinder(.00065, .019, black), (0, .015, .029))
node("antenna-tip", cylinder(.002, .005, frame), (0, .026, .029))

document["buffers"] = [dict(byteLength=len(binary))]
header = json.dumps(document, separators=(",", ":")).encode()
header += b" " * (-len(header) % 4)
glb = struct.pack("<III", 0x46546c67, 2, 12+8+len(header)+8+len(binary))
glb += struct.pack("<II", len(header), 0x4e4f534a) + header
glb += struct.pack("<II", len(binary), 0x004e4942) + binary
Path(__file__).with_suffix(".glb").write_bytes(glb)
