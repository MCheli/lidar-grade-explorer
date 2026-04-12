#!/usr/bin/env python3
"""
LiDAR Grade Explorer — Flask backend
Reads PLY/E57 point clouds without open3d; uses plyfile + numpy only.
"""
import os, io, base64, tempfile
import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter
from PIL import Image
from flask import Flask, request, jsonify, send_from_directory

app = Flask(__name__, static_folder="static")
UPLOAD_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "uploads")
os.makedirs(UPLOAD_DIR, exist_ok=True)


# ── point-cloud I/O ───────────────────────────────────────────────────────────

def read_ply(filepath):
    """Return (N,3) float32 positions and (N,3) float32 colors 0-1."""
    from plyfile import PlyData
    ply = PlyData.read(filepath)
    v   = ply["vertex"]
    pts = np.column_stack([
        v["x"].astype(np.float32),
        v["y"].astype(np.float32),
        v["z"].astype(np.float32),
    ])
    dtype_names = v.data.dtype.names
    if "red" in dtype_names:
        colors = np.column_stack([
            v["red"].astype(np.float32)   / 255.0,
            v["green"].astype(np.float32) / 255.0,
            v["blue"].astype(np.float32)  / 255.0,
        ])
    else:
        colors = np.full((len(pts), 3), 0.55, dtype=np.float32)
    return pts, colors


def read_e57(filepath):
    """Return (N,3) float32 positions and (N,3) float32 colors 0-1."""
    try:
        import pye57
    except ImportError:
        raise ValueError(
            "pye57 is not installed — run: pip install pye57\n"
            "Or convert your .e57 to .ply first (use CloudCompare, free)."
        )
    e57  = pye57.E57(filepath)
    data = e57.read_scan_raw(0)
    pts  = np.column_stack([
        data["cartesianX"].astype(np.float32),
        data["cartesianY"].astype(np.float32),
        data["cartesianZ"].astype(np.float32),
    ])
    if "colorRed" in data:
        colors = np.column_stack([
            data["colorRed"].astype(np.float32)   / 255.0,
            data["colorGreen"].astype(np.float32) / 255.0,
            data["colorBlue"].astype(np.float32)  / 255.0,
        ])
    else:
        colors = np.full((len(pts), 3), 0.55, dtype=np.float32)
    return pts, colors


def voxel_downsample(pts, colors, voxel_size):
    """Numpy-only voxel downsampling: average points within each voxel."""
    coords = np.floor(pts / voxel_size).astype(np.int32)
    mn = coords.min(axis=0)
    coords -= mn
    mx = coords.max(axis=0) + 1
    keys = (coords[:, 0].astype(np.int64) * mx[1] + coords[:, 1]) * mx[2] + coords[:, 2]
    order  = np.argsort(keys)
    keys_s = keys[order]
    pts_s  = pts[order]
    col_s  = colors[order]
    _, first, counts = np.unique(keys_s, return_index=True, return_counts=True)
    n = len(first)
    new_pts = np.zeros((n, 3), dtype=np.float32)
    new_col = np.zeros((n, 3), dtype=np.float32)
    np.add.at(new_pts, np.repeat(np.arange(n), counts), pts_s)
    np.add.at(new_col, np.repeat(np.arange(n), counts), col_s)
    new_pts /= counts[:, None]
    new_col /= counts[:, None]
    return new_pts, new_col.clip(0, 1)


# ── grid building ─────────────────────────────────────────────────────────────

def process_scan(filepath, voxel_size=0.03, grid_res=0.10):
    """Load and process a scan file; return (elev_rel, rgb_map, x_edges, z_edges)."""
    ext = os.path.splitext(filepath)[1].lower()
    if ext == ".ply":
        pts, colors = read_ply(filepath)
    elif ext == ".e57":
        pts, colors = read_e57(filepath)
    else:
        raise ValueError(f"Unsupported format: {ext}")

    if len(pts) == 0:
        raise ValueError("No points loaded — check file format")

    pts, colors = voxel_downsample(pts, colors, voxel_size)

    stds = pts.std(axis=0)
    vert = 1 if (stds[1] < stds[0] * 0.7 and stds[1] < stds[2] * 0.7) else int(stds.argmin())
    h0, h1 = [i for i in range(3) if i != vert]

    yv   = pts[:, vert]
    ylow = np.percentile(yv, 2)
    mask = (yv >= ylow - 0.5) & (yv <= ylow + 5.0)
    gpts, gcols = pts[mask], colors[mask]

    if len(gpts) < 100:
        raise ValueError("Too few ground points — try a larger voxel or grid size")

    res = grid_res
    xe  = np.arange(gpts[:, h0].min() - res, gpts[:, h0].max() + res * 2, res)
    ze  = np.arange(gpts[:, h1].min() - res, gpts[:, h1].max() + res * 2, res)
    nx, nz = len(xe) - 1, len(ze) - 1

    xi = (np.digitize(gpts[:, h0], xe) - 1).clip(0, nx - 1)
    zi = (np.digitize(gpts[:, h1], ze) - 1).clip(0, nz - 1)

    elev = np.full((nz, nx), np.nan)
    rgb  = np.zeros((nz, nx, 3))

    df  = pd.DataFrame({"xi": xi, "zi": zi, "y": gpts[:, vert],
                        "r": gcols[:, 0], "g": gcols[:, 1], "b": gcols[:, 2]})
    grp = df.groupby(["zi", "xi"])

    for _, row in grp["y"].quantile(0.05).reset_index().iterrows():
        elev[int(row["zi"]), int(row["xi"])] = row["y"]
    for _, row in grp[["r", "g", "b"]].mean().reset_index().iterrows():
        rgb[int(row["zi"]), int(row["xi"])] = [row["r"], row["g"], row["b"]]

    smooth = gaussian_filter(np.nan_to_num(elev), sigma=1.2)
    smooth[np.isnan(elev)] = np.nan
    elev_rel = smooth - np.nanmin(smooth)

    return elev_rel, rgb, xe, ze


# ── image generation ──────────────────────────────────────────────────────────

def _terrain_color(t):
    """Dramatic terrain colormap: deep blue → teal → green → yellow → orange → dark red."""
    stops = [
        (0.00, (  0,  40, 120)),
        (0.12, ( 20, 100, 180)),
        (0.28, ( 30, 160, 120)),
        (0.45, ( 60, 180,  60)),
        (0.60, (210, 200,  50)),
        (0.75, (240, 120,  20)),
        (0.88, (210,  45,  25)),
        (1.00, (140,  10,  10)),
    ]
    for i in range(len(stops) - 1):
        t0, c0 = stops[i]
        t1, c1 = stops[i + 1]
        if t <= t1:
            u = (t - t0) / (t1 - t0)
            return tuple(c0[j] + (c1[j] - c0[j]) * u for j in range(3))
    return stops[-1][1]


def elev_to_rgba(e):
    valid = ~np.isnan(e)
    vmin, vmax = np.nanpercentile(e, 2), np.nanpercentile(e, 98)
    norm = np.clip((e - vmin) / (vmax - vmin + 1e-9), 0, 1)
    H, W = e.shape
    out = np.zeros((H, W, 4), np.uint8)
    out[~valid] = [15, 15, 15, 255]
    r, g, b = zip(*[_terrain_color(float(x)) for x in norm[valid]])
    out[valid, 0] = np.clip(r, 0, 255).astype(np.uint8)
    out[valid, 1] = np.clip(g, 0, 255).astype(np.uint8)
    out[valid, 2] = np.clip(b, 0, 255).astype(np.uint8)
    out[valid, 3] = 255
    return out


def rgb_to_rgba(rgb, elev):
    H, W = elev.shape
    out  = np.zeros((H, W, 4), np.uint8)
    valid = ~np.isnan(elev)
    out[~valid] = [15, 15, 15, 255]
    out[valid, 0] = (rgb[valid, 0] * 255).clip(0, 255).astype(np.uint8)
    out[valid, 1] = (rgb[valid, 1] * 255).clip(0, 255).astype(np.uint8)
    out[valid, 2] = (rgb[valid, 2] * 255).clip(0, 255).astype(np.uint8)
    out[valid, 3] = 255
    return out


def to_b64png(rgba):
    img = Image.fromarray(rgba.astype(np.uint8))
    buf = io.BytesIO()
    img.save(buf, format="PNG", optimize=True)
    return base64.b64encode(buf.getvalue()).decode()


def build_response(elev_rel, rgb, xe, ze):
    nz, nx = elev_rel.shape
    cell   = round(float(xe[1] - xe[0]), 6)
    flat   = [-1 if np.isnan(v) else round(float(v), 3) for v in elev_rel.flatten()]
    return {
        "elev_img": to_b64png(elev_to_rgba(elev_rel)),
        "rgb_img":  to_b64png(rgb_to_rgba(rgb, elev_rel)),
        "grid": {
            "data": flat, "rows": nz, "cols": nx,
            "xmin": float(xe[0]),  "xmax": float(xe[-1]),
            "zmin": float(ze[0]),  "zmax": float(ze[-1]),
            "emax": float(np.nanmax(elev_rel)),
            "cell": cell,
        },
    }


# ── Flask routes ──────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return send_from_directory("static", "index.html")


@app.route("/api/upload", methods=["POST"])
def upload():
    if "file" not in request.files:
        return jsonify({"error": "No file provided"}), 400
    f   = request.files["file"]
    ext = os.path.splitext(f.filename or "")[1].lower()
    if ext not in (".ply", ".e57"):
        return jsonify({"error": f"Unsupported format '{ext}'. Use .ply or .e57"}), 400

    tmp = tempfile.NamedTemporaryFile(suffix=ext, delete=False, dir=UPLOAD_DIR)
    f.save(tmp.name)
    tmp.close()

    try:
        voxel = float(request.form.get("voxel", "0.03"))
        res   = float(request.form.get("res",   "0.10"))
        elev_rel, rgb, xe, ze = process_scan(tmp.name, voxel, res)
        return jsonify(build_response(elev_rel, rgb, xe, ze))
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500
    finally:
        try:
            os.unlink(tmp.name)
        except OSError:
            pass


if __name__ == "__main__":
    print("LiDAR Grade Explorer  →  http://localhost:5050")
    app.run(debug=True, port=5050, host="0.0.0.0")
