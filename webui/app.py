#!/usr/bin/env python3
"""
UNGENAU LiDAR Scanner - Web Interface
Jetson TX2 / ROS Melodic / Ouster OS1-64
"""
import os, subprocess, signal, glob, time, threading, json
from datetime import datetime
from flask import Flask, render_template, jsonify, send_file, Response

app = Flask(__name__, template_folder="templates", static_folder="static")

DATA_DIR  = "/media/nvidia/SSD/lidar_data"
BAGS_DIR  = os.path.join(DATA_DIR, "bags")
MAPS_DIR  = os.path.join(DATA_DIR, "maps")
ROS_SETUP = "/opt/ros/melodic/setup.bash"
WS_SETUP  = "/home/nvidia/catkin_ws/devel/setup.bash"

recording_process = None
recording_start_time = None
recording_lock = threading.Lock()


def ros_env():
    env = os.environ.copy()
    env["ROS_MASTER_URI"] = "http://localhost:11311"
    env["ROS_IP"] = "127.0.0.1"
    return env


def run_cmd(cmd, timeout=2):
    try:
        r = subprocess.run(cmd, env=ros_env(), timeout=timeout,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return r.returncode == 0
    except Exception:
        return False


def is_ros_running():
    return run_cmd(["rostopic", "list"], timeout=2)


def is_recording():
    with recording_lock:
        return recording_process is not None and recording_process.poll() is None


def get_bags():
    bags = []
    for path in sorted(glob.glob(os.path.join(BAGS_DIR, "*.bag")), reverse=True):
        s = os.stat(path)
        bags.append({
            "name": os.path.basename(path),
            "size_mb": round(s.st_size / 1024.0 / 1024.0, 1),
            "mtime": datetime.fromtimestamp(s.st_mtime).strftime("%Y-%m-%d %H:%M"),
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


# ─── Routes ───────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template("lidar_viewer.html")


@app.route("/api/status")
def api_status():
    ros_ok = is_ros_running()
    rec = is_recording()
    duration = int(time.time() - recording_start_time) if rec and recording_start_time else None
    lidar_ok = ros_ok and run_cmd(["rostopic", "hz", "/ouster/points", "--window=1"], timeout=1)
    imu_ok   = ros_ok and run_cmd(["rostopic", "hz", "/ouster/imu", "--window=1"], timeout=1)
    return jsonify({
        "ros": ros_ok,
        "lidar": lidar_ok,
        "imu": imu_ok,
        "recording": rec,
        "duration_s": duration,
        "disk": get_disk_info(),
        "time": datetime.now().strftime("%H:%M:%S"),
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
            recording_process = subprocess.Popen(
                ["bash", "-c",
                 "source {} && source {} && rosbag record -O {} /ouster/points /ouster/imu /sbg/ekf_euler /tf /tf_static".format(
                     ROS_SETUP, WS_SETUP, bag_path)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            recording_start_time = time.time()
            return jsonify({"ok": True, "msg": "Recording: {}".format(os.path.basename(bag_path)), "bag": os.path.basename(bag_path)})
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
        recording_process = None
        recording_start_time = None
        return jsonify({"ok": True, "msg": "Gestoppt"})


@app.route("/api/bags")
def api_bags():
    return jsonify(get_bags())


@app.route("/api/download/<filename>")
def api_download(filename):
    path = os.path.join(BAGS_DIR, filename)
    if not os.path.exists(path) or not filename.endswith(".bag"):
        return jsonify({"error": "Not found"}), 404
    return send_file(path, as_attachment=True)


if __name__ == "__main__":
    os.makedirs(BAGS_DIR, exist_ok=True)
    os.makedirs(MAPS_DIR, exist_ok=True)
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
