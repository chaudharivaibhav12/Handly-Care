"""Move to (OVERVIEW x, y, Z) at OVERVIEW_THETA, grab color+depth, and map pixels <-> table.
Usage: view_map.py Z OUTDIR [move|nomove]"""
import asyncio, io, json, os, struct, sys
sys.path.insert(0, os.getcwd())
from dotenv import load_dotenv
load_dotenv(".env")
import config
from robot import RealRobot
from viam.proto.common import Pose, PoseInFrame

def depth_at(dep, u, v, w, h):
    vals = []
    for dv in range(-3, 4):
        for du in range(-3, 4):
            uu, vv = int(u) + du, int(v) + dv
            if 0 <= uu < w and 0 <= vv < h:
                (d,) = struct.unpack_from(">H", dep, 24 + 2 * (vv * w + uu))
                if d: vals.append(d)
    vals.sort()
    return vals[len(vals) // 2] if vals else None

async def to(m, frame_from, xyz, frame_to):
    p = await m.transform_pose(PoseInFrame(reference_frame=frame_from, pose=Pose(x=xyz[0], y=xyz[1], z=xyz[2], o_z=1)), frame_to)
    return (p.pose.x, p.pose.y, p.pose.z)

async def main(z, out, move):
    r = await RealRobot.connect()
    m = r.machine
    try:
        if move:
            await r.move_to(float(os.environ.get("OX", config.OVERVIEW[0])), float(os.environ.get("OY", config.OVERVIEW[1])), z, theta=config.OVERVIEW_THETA)
        p = await r.where()
        print(f"gripper at {p.x:.0f} {p.y:.0f} {p.z:.0f} theta {p.theta:.0f}")
        imgs, _ = await r.camera.get_images()
        color = next(i for i in imgs if "jpeg" in i.mime_type or "png" in i.mime_type)
        dep = next((i for i in imgs if "dep" in i.mime_type), None)
        open(os.path.join(out, "color.jpg"), "wb").write(color.data)
        props = await r.camera.get_properties()
        ip = props.intrinsic_parameters
        fx, fy, cx, cy, W, H = ip.focal_x_px, ip.focal_y_px, ip.center_x_px, ip.center_y_px, ip.width_px, ip.height_px
        print("intrinsics", fx, fy, cx, cy, W, H)
        info = {"fx": fx, "fy": fy, "cx": cx, "cy": cy, "W": W, "H": H}
        cam_o = await to(m, "cam", (0, 0, 0), "world")
        print("camera origin in world", [round(v) for v in cam_o])
        table_z = None
        if dep is not None:
            dw, dh = struct.unpack_from(">QQ", dep.data, 8)
            print("depth", dw, dh, dep.data[:8])
            d = depth_at(dep.data, dw / 2, dh / 2, dw, dh)
            c = await to(m, "cam", (0, 0, d), "world")
            table_z = c[2]
            print(f"center depth {d} mm -> world {[round(v) for v in c]}")
            info["center_depth_world"] = c
        # Corners of the image projected onto the table plane
        tz = table_z if table_z is not None else -10
        info["table_z"] = tz
        corners = {}
        for name, (u, v) in {"tl": (0, 0), "tr": (W, 0), "br": (W, H), "bl": (0, H), "c": (cx, cy)}.items():
            ray = await to(m, "cam", ((u - cx) / fx * 1000, (v - cy) / fy * 1000, 1000), "world")
            t = (tz - cam_o[2]) / (ray[2] - cam_o[2])
            corners[name] = [cam_o[i] + t * (ray[i] - cam_o[i]) for i in range(3)]
            print(name, [round(v) for v in corners[name][:2]])
        info["corners"] = corners
        # Spots to mark
        spots = json.loads(os.environ.get("SPOTS", "{}"))
        px = {}
        for name, (x, y) in spots.items():
            c = await to(m, "world", (x, y, tz), "cam")
            if c[2] > 0:
                u, v = fx * c[0] / c[2] + cx, fy * c[1] / c[2] + cy
                px[name] = (u, v)
                chk = depth_at(dep.data, u, v, dw, dh) if dep is not None and 0 <= u < W and 0 <= v < H else None
                print(f"{name} ({x},{y}) -> pixel ({u:.0f},{v:.0f}) cam z {c[2]:.0f} depth there {chk}")
        info["px"] = px
        json.dump(info, open(os.path.join(out, "map.json"), "w"), indent=1)
    finally:
        await r.close()

asyncio.run(main(float(sys.argv[1]), sys.argv[2], len(sys.argv) < 4 or sys.argv[3] == "move"))
