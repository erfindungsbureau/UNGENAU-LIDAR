#!/usr/bin/env python3
"""
UNGENAU LiDAR Scanner - Web Interface
Jetson TX2 / ROS Melodic / Ouster OS1-64 / SBG Ellipse 2
"""
import os, re, subprocess, signal, glob, time, threading, json
from datetime import datetime
from flask import Flask, render_template, jsonify, send_file, Response, request

app = Flask(__name__, template_folder="templates", static_folder="static")

DATA_DIR  = "/media/nvidia/SSD/lidar_data"
BAGS_DIR  = os.path.join(DATA_DIR, "bags")
MAPS_DIR  = os.path.join(DATA_DIR, "maps")
ROS_SETUP = "/opt/ros/melodic/setup.bash"
WS_SETUP  = "/home/nvidia/catkin_ws/devel/setup.bash"

recording_process    = None
recording_start_time = None
recording_lock       = threading.Lock()


def ros_env():
    env = os.environ.copy()
    env["ROS_MASTER_URI"] = "http://localhost:11311"
    env["ROS_IP"] = "127.0.0.1"
    return env


def is_lidar_publishing():
    try:
        return (time.time() - os.path.getmtime("/tmp/pts.json")) < 3.0
    except Exception:
        return False


def is_recording():
    with recording_lock:
        return recording_process is not None and recording_process.poll() is None


def get_bags():
    bags = []
    for path in sorted(glob.glob(os.path.join(BAGS_DIR, "*.bag")), reverse=True):
        s = os.stat(path)
        bags.append({
            "name":    os.path.basename(path),
            "size_mb": round(s.st_size / 1024.0 / 1024.0, 1),
            "mtime":   datetime.fromtimestamp(s.st_mtime).strftime("%Y-%m-%d %H:%M"),
        })
    return bags


def get_disk_info():
    try:
        s = os.statvfs(DATA_DIR)
        free_gb  = round(s.f_frsize * s.f_bavail / 1e9, 1)
        total_gb = round(s.f_frsize * s.f_blocks / 1e9, 1)
        used_pct = round(100 * (1 - float(s.f_bavail) / s.f_blocks))
        return {"free_gb": free_gb, "total_gb": total_gb, "used_pct": used_pct}
    except Exception:
        return {"free_gb": 0, "total_gb": 0, "used_pct": 0}


def sanitize_name(name):
    name = re.sub(r'[^\w\- ]', '', name).strip().replace(' ', '_')
    return name[:80] if name else None


# ─── Routes ───────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template("lidar_viewer.html")


@app.route("/api/status")
def api_status():
    rec = is_recording()
    duration = int(time.time() - recording_start_time) if rec and recording_start_time else None
    return jsonify({
        "ros":        os.path.exists("/tmp/pts.json"),
        "lidar":      is_lidar_publishing(),
        "imu":        os.path.exists("/tmp/imu.json"),
        "recording":  rec,
        "duration_s": duration,
        "disk":       get_disk_info(),
        "time":       datetime.now().strftime("%H:%M:%S"),
    })


@app.route("/api/points")
def api_points():
    try:
        with open("/tmp/pts.json") as f:
            data = f.read()
    except Exception:
        data = "[]"
    resp = Response(data, mimetype="application/json")
    resp.headers["Cache-Control"] = "no-cache, no-store"
    return resp


@app.route("/api/imu")
def api_imu():
    try:
        with open("/tmp/imu.json") as f:
            data = f.read()
        resp = Response(data, mimetype="application/json")
        resp.headers["Cache-Control"] = "no-cache, no-store"
        return resp
    except Exception:
        return jsonify({"roll": 0, "pitch": 0, "yaw": 0})


@app.route("/api/record/start", methods=["POST"])
def api_record_start():
    global recording_process, recording_start_time
    with recording_lock:
        if is_recording():
            return jsonify({"ok": False, "msg": "Läuft bereits"})
        os.makedirs(BAGS_DIR, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(BAGS_DIR, "scan_{}.bag".format(ts))
        try:
            cmd = (
                "source {ros} && source {ws} && "
                "rosbag record -O {bag} "
                "/ouster/points /ouster/imu /tf /tf_static"
            ).format(ros=ROS_SETUP, ws=WS_SETUP, bag=bag_path)
            recording_process = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            recording_start_time = time.time()
            bag_name = os.path.basename(bag_path)
            return jsonify({"ok": True, "bag": bag_name,
                            "msg": "Recording: {}".format(bag_name)})
        except Exception as e:
            return jsonify({"ok": False, "msg": str(e)})


@app.route("/api/record/stop", methods=["POST"])
def api_record_stop():
    global recording_process, recording_start_time
    with recording_lock:
        if not is_recording():
            return jsonify({"ok": False, "msg": "Nicht aktiv"})
        try:
            recording_process.send_signal(signal.SIGINT)
            recording_process.wait(timeout=10)
        except Exception:
            pass
        recording_process    = None
        recording_start_time = None
        return jsonify({"ok": True, "msg": "Gestoppt"})


@app.route("/api/bags")
def api_bags():
    return jsonify(get_bags())


@app.route("/api/bags/<filename>", methods=["DELETE"])
def api_delete_bag(filename):
    if "/" in filename or not filename.endswith(".bag"):
        return jsonify({"ok": False, "msg": "Ungültig"}), 400
    path = os.path.join(BAGS_DIR, filename)
    if not os.path.exists(path):
        return jsonify({"ok": False, "msg": "Nicht gefunden"}), 404
    os.remove(path)
    return jsonify({"ok": True})


@app.route("/api/bags/<filename>/rename", methods=["POST"])
def api_rename_bag(filename):
    if "/" in filename or not filename.endswith(".bag"):
        return jsonify({"ok": False, "msg": "Ungültig"}), 400
    data     = request.get_json() or {}
    new_name = sanitize_name(data.get("name", ""))
    if not new_name:
        return jsonify({"ok": False, "msg": "Kein Name"}), 400
    if not new_name.endswith(".bag"):
        new_name += ".bag"
    src = os.path.join(BAGS_DIR, filename)
    dst = os.path.join(BAGS_DIR, new_name)
    if not os.path.exists(src):
        return jsonify({"ok": False, "msg": "Nicht gefunden"}), 404
    os.rename(src, dst)
    return jsonify({"ok": True, "name": new_name})


@app.route("/api/bags/<filename>/export", methods=["POST"])
def api_export_bag(filename):
    if "/" in filename or not filename.endswith(".bag"):
        return jsonify({"ok": False, "msg": "Ungültig"}), 400
    bag_path = os.path.join(BAGS_DIR, filename)
    if not os.path.exists(bag_path):
        return jsonify({"ok": False, "msg": "Nicht gefunden"}), 404
    xyz_name = filename.replace(".bag", ".xyz")
    xyz_path = os.path.join(BAGS_DIR, xyz_name)
    cmd = (
        "source {ros} && source {ws} && "
        "python2 /home/nvidia/bag_to_xyz.py {bag} {xyz} 5"
        " > /tmp/export.log 2>&1"
    ).format(ros=ROS_SETUP, ws=WS_SETUP, bag=bag_path, xyz=xyz_path)
    subprocess.Popen(["bash", "-c", cmd])
    return jsonify({"ok": True, "file": xyz_name,
                    "msg": "Export gestartet (log: /tmp/export.log)"})


@app.route("/api/download/<filename>")
def api_download(filename):
    if "/" in filename:
        return jsonify({"error": "Ungültig"}), 400
    allowed_ext = (".bag", ".xyz", ".e57", ".pcd", ".las")
    if not any(filename.endswith(e) for e in allowed_ext):
        return jsonify({"error": "Nicht erlaubt"}), 403
    path = os.path.join(BAGS_DIR, filename)
    if not os.path.exists(path):
        return jsonify({"error": "Nicht gefunden"}), 404
    return send_file(path, as_attachment=True)


if __name__ == "__main__":
    os.makedirs(BAGS_DIR, exist_ok=True)
    os.makedirs(MAPS_DIR, exist_ok=True)
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
