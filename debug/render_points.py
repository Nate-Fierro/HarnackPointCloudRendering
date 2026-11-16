"""Render OBJ sample positions as dots using the viewer's default camera.

Requires Pillow. This preview never evaluates the winding field or traces rays.
"""
import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def unit(v):
    length = math.sqrt(dot(v, v))
    return tuple(x / length for x in v)


def cross(a, b):
    return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0])


def render(source, destination, size):
    points = []
    for line in source.read_text(encoding="ascii").splitlines():
        fields = line.split()
        if fields and fields[0] == "v":
            points.append(tuple(map(float, fields[1:4])))
    if not points:
        raise ValueError("No OBJ point positions found")

    # Match CustomPtCloud::Normalize and main.cpp's initial perspective camera.
    center = tuple((min(p[k] for p in points) + max(p[k] for p in points)) / 2 for k in range(3))
    radius = max(math.sqrt(dot(sub(p, center), sub(p, center))) for p in points)
    points = [tuple(x / radius for x in sub(p, center)) for p in points]
    camera = (0., .25, 3.)
    forward = unit(tuple(-x for x in camera))
    right = unit(cross(forward, (0., 1., 0.)))
    up = unit(cross(right, forward))
    focal = size / (2 * math.tan(math.radians(45) / 2))
    projected = []
    for point in points:
        relative = sub(point, camera)
        depth = dot(relative, forward)
        if depth <= 0:
            continue
        projected.append((depth,
                          size / 2 + focal * dot(relative, right) / depth,
                          size / 2 - focal * dot(relative, up) / depth))
    far = max(p[0] for p in projected)
    near = min(p[0] for p in projected)
    scale = 4
    image = Image.new("RGB", (size * scale, size * scale), (9, 14, 22))
    draw = ImageDraw.Draw(image)
    for depth, x, y in sorted(projected, reverse=True):
        # Depth tint supplies a 3D cue while leaving gaps and back points visible.
        t = (far - depth) / max(far - near, 1e-12)
        color = tuple(round(a + t * (b - a)) for a, b in zip((24, 75, 101), (105, 223, 255)))
        r = 1.6 * scale
        x *= scale
        y *= scale
        draw.ellipse((x-r, y-r, x+r, y+r), fill=color)
    image = image.resize((size, size), Image.Resampling.LANCZOS)
    destination.parent.mkdir(parents=True, exist_ok=True)
    image.save(destination)
    print(f"Rendered {len(projected)} of {len(points)} sample positions to {destination}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", nargs="?", type=Path, default=ROOT / "data/bunny-points.obj")
    parser.add_argument("--output", type=Path, default=ROOT / "build/bunny-points.png")
    parser.add_argument("--size", type=int, default=640)
    args = parser.parse_args()
    render(args.source, args.output, args.size)
