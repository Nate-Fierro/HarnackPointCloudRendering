"""Precompute intrinsic Voronoi areas for the existing sampled Stanford bunny.

Uses the cached source mesh and the optional C++ precompute_areas executable.
The original OBJ's point positions and normals are never rewritten.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import random
import subprocess
import tarfile

from prepare_samples import ROOT, cross, sub


def prepare_job(source, archive, job_path):
    points = []
    for line in source.read_text(encoding="ascii").splitlines():
        fields = line.split()
        if fields and fields[0] == "v": points.append(tuple(map(float, fields[1:4])))
    with tarfile.open(archive, "r:gz") as tar:
        lines = iter(tar.extractfile("bunny/reconstruction/bun_zipper.ply").read().decode("ascii").splitlines())
        for line in lines:
            if line.startswith("element vertex "): nv = int(line.split()[2])
            if line.startswith("element face "): nf = int(line.split()[2])
            if line == "end_header": break
        vertices = [tuple(map(float, next(lines).split()[:3])) for _ in range(nv)]
        triangles = [tuple(map(int, next(lines).split()[1:4])) for _ in range(nf)]
    faces, cumulative, areas = [], [], []
    total = 0.
    for tri in triangles:
        a, b, c = (vertices[j] for j in tri)
        n = cross(sub(b, a), sub(c, a))
        area = .5 * math.sqrt(sum(x*x for x in n))
        if area <= 0: continue
        faces.append(tri)
        areas.append(area)
        total += area
        cumulative.append(total)

    # Reproduce the sampling provenance; use the exact interior points for MMP.
    rng = random.Random(20261114)
    seeds = []
    max_deviation = 0.
    for point in points:
        face_id = bisect.bisect_left(cumulative, rng.random() * total)
        u = math.sqrt(rng.random())
        v = rng.random()
        bary = (1-u, u*(1-v), u*v)
        seed = tuple(sum(bary[j] * vertices[faces[face_id][j]][k] for j in range(3)) for k in range(3))
        max_deviation = max(max_deviation, *(abs(a-b) for a, b in zip(seed, point)))
        seeds.append((face_id, seed, point))
    if not points or max_deviation > 1e-9:
        raise ValueError("OBJ does not match the deterministic Stanford bunny sampling; cannot recover seed faces")

    # Verify every edge-connected component has a source, and reject nonmanifold edges.
    parent = list(range(len(faces)))
    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    edge_faces = {}
    for fi, face in enumerate(faces):
        for i in range(3):
            edge = tuple(sorted((face[i], face[(i+1) % 3])))
            neighbors = edge_faces.setdefault(edge, [])
            neighbors.append(fi)
            if len(neighbors) > 2: raise ValueError("Source mesh has a nonmanifold edge")
            if len(neighbors) == 2: parent[find(fi)] = find(neighbors[0])
    components = {find(i) for i in range(len(faces))}
    seeded = {find(f) for f, _, _ in seeds}
    if components != seeded: raise ValueError("Source mesh has an unseeded disconnected component")

    # The library disallows unused vertices, which are present in many scan files.
    used = sorted({i for face in faces for i in face})
    remap = {old: new for new, old in enumerate(used)}
    job_path.parent.mkdir(parents=True, exist_ok=True)
    with job_path.open("w", encoding="ascii", newline="\n") as out:
        out.write(f"HARNACK_AREA_JOB 1\n{len(used)} {len(faces)} {len(seeds)}\n")
        for i in used: out.write(" ".join(f"{x:.17g}" for x in vertices[i]) + "\n")
        for face in faces: out.write(" ".join(str(remap[i]) for i in face) + "\n")
        for face, seed, identity in seeds:
            out.write(str(face) + " " + " ".join(f"{x:.17g}" for x in (*seed, *identity)) + "\n")
    return {"point_count": len(points), "source_mesh_area": total,
            "source_vertices": len(used), "source_faces": len(faces), "components": len(components),
            "seed_position_rounding_error": max_deviation,
            "source_archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
            "point_obj_sha256": hashlib.sha256(source.read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT / "data/bunny-points.obj")
    parser.add_argument("--tool", type=Path, default=ROOT / "build/Release/precompute_areas.exe")
    parser.add_argument("--depth", type=int, default=8)
    args = parser.parse_args()
    archive = ROOT / "data/downloads/bunny.tar.gz"
    if not archive.exists(): raise FileNotFoundError("Run scripts/prepare_samples.py to download the bunny archive first")
    job = ROOT / "build/bunny-area-job.txt"
    metadata = prepare_job(args.input, archive, job)
    print(json.dumps(metadata, indent=2), flush=True)
    candidate = ROOT / "build/bunny-points.areas.candidate"
    subprocess.run([str(args.tool), str(job), str(candidate), str(args.depth)], check=True)
    with candidate.open("a", encoding="ascii") as out:
        for key, value in metadata.items(): out.write(f"# {key}: {value}\n")
    destination = args.input.with_suffix(".areas")
    candidate.replace(destination)
    print(f"Saved {destination}; original OBJ unchanged")


if __name__ == "__main__": main()
