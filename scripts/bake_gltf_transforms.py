"""Bakes the world transform of every mesh node of a glTF into its vertices and flattens the nodes.

Usage:
    python scripts/bake_gltf_transforms.py INPUT_DIR OUTPUT_DIR

INPUT_DIR holds scene.gltf, scene.bin, and the folders it refers to (textures). OUTPUT_DIR gets the
flattened scene.gltf and scene.bin, and a copy of everything else.

Why: the ALTIUS 700M model nests nodes under parents with non-uniform scales (a parent scaled 4.55
along one axis, its child scaled 0.22 so that the pair is a plain 1). Unreal's glTF import composes
such chains wrongly: the tail surfaces came out 4.3 and 4.55 times too large and the propeller
blades 1.2 to 1.5 times too long. With the transforms baked into the vertices, every mesh node is a
child of one root with no transform, so there is no chain to compose.

Positions are transformed by the world matrix. Normals are transformed by the inverse transpose of
its linear part and renormalized. A world matrix that mirrors (a negative determinant) reverses the
triangle winding. Texture coordinates, materials, and node and mesh names are kept. Only stdlib.
"""

import copy
import json
import math
import os
import shutil
import struct
import sys

FLOAT = 5126
USHORT = 5123
UINT = 5125
ARRAY_BUFFER = 34962
ELEMENT_ARRAY_BUFFER = 34963


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def local_matrix(node):
    if "matrix" in node:
        m = node["matrix"]
        return [[m[c * 4 + r] for c in range(4)] for r in range(4)]
    t = node.get("translation", [0.0, 0.0, 0.0])
    x, y, z, w = node.get("rotation", [0.0, 0.0, 0.0, 1.0])
    s = node.get("scale", [1.0, 1.0, 1.0])
    rot = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    return [[rot[i][j] * s[j] for j in range(3)] + [t[i]] for i in range(3)] + [[0, 0, 0, 1]]


def det3(m):
    return (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def inverse_transpose3(m):
    d = det3(m)
    cof = [
        [
            m[(i + 1) % 3][(j + 1) % 3] * m[(i + 2) % 3][(j + 2) % 3]
            - m[(i + 1) % 3][(j + 2) % 3] * m[(i + 2) % 3][(j + 1) % 3]
            for j in range(3)
        ]
        for i in range(3)
    ]
    return [[cof[i][j] / d for j in range(3)] for i in range(3)]


def read_accessor(gltf, blob, index):
    acc = gltf["accessors"][index]
    view = gltf["bufferViews"][acc["bufferView"]]
    components = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[acc["type"]]
    code = {FLOAT: "f", USHORT: "H", UINT: "I"}[acc["componentType"]]
    size = struct.calcsize(code)
    stride = view.get("byteStride", components * size)
    base = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    rows = []
    for i in range(acc["count"]):
        rows.append(struct.unpack_from("<" + code * components, blob, base + i * stride))
    return rows


def append_data(gltf, blob, rows, code, acc_type, component_type, target, minmax=False):
    """Appends rows to the blob as a new buffer view and accessor; returns the accessor index."""
    components = len(rows[0])
    data = b"".join(struct.pack("<" + code * components, *row) for row in rows)
    while len(blob) % 4:
        blob.append(0)
    gltf["bufferViews"].append(
        {"buffer": 0, "byteOffset": len(blob), "byteLength": len(data), "target": target}
    )
    blob.extend(data)
    accessor = {
        "bufferView": len(gltf["bufferViews"]) - 1,
        "componentType": component_type,
        "count": len(rows),
        "type": acc_type,
    }
    if minmax:
        accessor["min"] = [min(r[k] for r in rows) for k in range(components)]
        accessor["max"] = [max(r[k] for r in rows) for k in range(components)]
    gltf["accessors"].append(accessor)
    return len(gltf["accessors"]) - 1


def bake(input_dir, output_dir):
    with open(os.path.join(input_dir, "scene.gltf"), encoding="utf-8") as handle:
        gltf = json.load(handle)
    with open(os.path.join(input_dir, "scene.bin"), "rb") as handle:
        blob = bytearray(handle.read())

    nodes = gltf["nodes"]
    parent = {}
    for i, node in enumerate(nodes):
        for child in node.get("children", []):
            parent[child] = i

    def world(i):
        m = local_matrix(nodes[i])
        while i in parent:
            i = parent[i]
            m = mat_mul(local_matrix(nodes[i]), m)
        return m

    mesh_nodes = [i for i, n in enumerate(nodes) if "mesh" in n]
    used = [nodes[i]["mesh"] for i in mesh_nodes]
    if len(set(used)) != len(used):
        raise SystemExit("a mesh is used by more than one node; instancing is not handled")

    flat = [{"name": "RootNode", "children": list(range(1, len(mesh_nodes) + 1))}]
    for slot, i in enumerate(mesh_nodes, start=1):
        w = world(i)
        linear = [row[:3] for row in w[:3]]
        mirrored = det3(linear) < 0
        normal_matrix = inverse_transpose3(linear)
        mesh = gltf["meshes"][nodes[i]["mesh"]]
        for primitive in mesh["primitives"]:
            attributes = primitive["attributes"]
            pos = read_accessor(gltf, blob, attributes["POSITION"])
            moved = [
                tuple(sum(w[r][c] * p[c] for c in range(3)) + w[r][3] for r in range(3))
                for p in pos
            ]
            attributes["POSITION"] = append_data(
                gltf, blob, moved, "f", "VEC3", FLOAT, ARRAY_BUFFER, minmax=True
            )
            if "NORMAL" in attributes:
                normals = read_accessor(gltf, blob, attributes["NORMAL"])
                turned = []
                for n in normals:
                    v = [sum(normal_matrix[r][c] * n[c] for c in range(3)) for r in range(3)]
                    length = math.sqrt(sum(a * a for a in v)) or 1.0
                    turned.append(tuple(a / length for a in v))
                attributes["NORMAL"] = append_data(
                    gltf, blob, turned, "f", "VEC3", FLOAT, ARRAY_BUFFER
                )
            attributes.pop("TANGENT", None)
            if mirrored and "indices" in primitive:
                values = [row[0] for row in read_accessor(gltf, blob, primitive["indices"])]
                for k in range(0, len(values) - 2, 3):
                    values[k + 1], values[k + 2] = values[k + 2], values[k + 1]
                old = gltf["accessors"][primitive["indices"]]
                code = "H" if old["componentType"] == USHORT else "I"
                primitive["indices"] = append_data(
                    gltf, blob, [(v,) for v in values], code, "SCALAR", old["componentType"],
                    ELEMENT_ARRAY_BUFFER,
                )
                print(f"  {nodes[i].get('name')}: mirrored, winding reversed")
        flat.append({"name": nodes[i].get("name", f"mesh{slot}"), "mesh": nodes[i]["mesh"]})

    gltf["nodes"] = flat
    gltf["scenes"] = [{"name": gltf["scenes"][0].get("name", "scene"), "nodes": [0]}]
    gltf["scene"] = 0
    gltf["buffers"][0]["byteLength"] = len(blob)
    gltf["buffers"][0]["uri"] = "scene.bin"

    os.makedirs(output_dir, exist_ok=True)
    for name in os.listdir(input_dir):
        if name in ("scene.gltf", "scene.bin"):
            continue
        source = os.path.join(input_dir, name)
        target = os.path.join(output_dir, name)
        if os.path.isdir(source):
            shutil.copytree(source, target, dirs_exist_ok=True)
        else:
            shutil.copy2(source, target)
    with open(os.path.join(output_dir, "scene.bin"), "wb") as handle:
        handle.write(bytes(blob))
    with open(os.path.join(output_dir, "scene.gltf"), "w", encoding="utf-8") as handle:
        json.dump(gltf, handle, indent=1)
    print(f"baked {len(mesh_nodes)} mesh nodes into {output_dir}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    bake(sys.argv[1], sys.argv[2])
